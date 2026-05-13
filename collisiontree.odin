package collisiontree

import "base:runtime"
import enc "core:encoding/json"
import "core:fmt"
import "core:log"
import "core:math"
import la "core:math/linalg"
import "core:mem"
import "core:os"
import "core:sync"
import "core:sync/chan"
import "core:thread"
import time "core:time"

scanSize: u32

dyn_pool: mem.Dynamic_Pool
pool_allocator: mem.Allocator
pool: thread.Pool
runners: [dynamic]TaskRunner
contexts: [dynamic]ThreadContext
commsAllocator: mem.Allocator
comms: chan.Chan(ThreadContext)
g_logger: log.Logger
runChanAlloc: mem.Allocator
runChan: chan.Chan(mem.Allocator)
freeMutex: ^sync.Mutex
completeGroup: ^sync.Wait_Group


collisionTreeInit :: proc(threadCount: int = 1) {
	commsAllocator = _newArenaAllocator()
	completeGroup = new(sync.Wait_Group)
	freeMutex = new(sync.Mutex)
	runChanAlloc = _newArenaAllocator()
	err: runtime.Allocator_Error
	runChan, err = chan.create_buffered(chan.Chan(mem.Allocator), os.get_processor_core_count(), runChanAlloc)
	assert(err == .None)
	fmt.println(threadCount)
	fmt.println(chan.can_send(runChan))
	for i in 0 ..< threadCount {
		chan.send(chan.as_send(runChan), _newArenaAllocator())
	}
	fmt.println("finished prepping runChan")
	mem.dynamic_pool_init(&dyn_pool)
	pool_allocator = mem.dynamic_pool_allocator(&dyn_pool)
	thread.pool_init(&pool, pool_allocator, threadCount)
	thread.pool_start(&pool)
}

collistionTreeCleanup :: proc() {
	delete(runners)
	delete(contexts)
	for run in runners {
		free_all(run.allocator)
		mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)run.allocator.data)
	}
	thread.pool_shutdown(&pool)
	thread.pool_destroy(&pool)
	free_all(pool_allocator)
	mem.dynamic_pool_destroy(cast(^mem.Dynamic_Pool)pool_allocator.data)
	chan.destroy(comms)
	free_all(commsAllocator)
	mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)commsAllocator.data)
}

waitForBatch :: proc() {
	sync.wait(completeGroup)
	fmt.println("complete")
}

//default wait is 1 ms
isBatchComplete :: proc(timeOut: time.Duration = 1_000_000) -> bool {
	return sync.wait_group_wait_with_timeout(completeGroup, timeOut)
}

//Don't call this again until the chanReceiver is done
collisionTreeBatchedRayScan :: proc(
	colTree: ^CollisionTree,
	rays: []Ray,
	batchCount: int,
) -> (
	chan.Chan(ThreadContext, .Recv),
	int,
) {
	// free_all(pool_allocator)
	free_all(commsAllocator)
	chan.close(comms)
	chan.destroy(comms)
	delete(runners)
	for c in contexts {
		delete(c.hit)
	}
	delete(contexts)
	offset: u32 = 0
	maxEnd := u32(len(rays))
	tCount := u32(batchCount)
	scanSize = maxEnd / tCount
	scanSize = tCount * scanSize < maxEnd ? scanSize + 1 : scanSize
	c, err := chan.create_buffered(chan.Chan(ThreadContext), tCount, commsAllocator)
	assert(err == .None)
	comms = c
	runners = make([dynamic]TaskRunner, tCount, tCount)
	contexts = make([dynamic]ThreadContext, tCount, tCount)
	// adding to the group before spawning the tasks to ensure that any quick tasks actually subtract from the wait group.
	// fmt.println("adding to waitgroup")
	sync.wait_group_add(completeGroup, int(tCount))
	// fmt.printfln("added %v to wait group", tCount)
	for &run, i in runners {
		ok: bool
		run.task = _threadScan
		run.allocator, ok = chan.recv(runChan)
		assert(ok)
		contexts[i].offset = 0
		end := math.min(offset + scanSize, maxEnd)
		contexts[i].colTree = colTree
		contexts[i].rays = rays[offset:end]
		contexts[i].offset = offset
		offset += scanSize
		contexts[i].hit = make([dynamic]Hit, 0, scanSize)
		thread.pool_add_task(&pool, run.allocator, run.task, rawptr(&contexts[i]), i)
	}
	return chan.as_recv(comms), int(tCount)
}

_newArenaAllocator :: proc() -> mem.Allocator {
	a := new(mem.Dynamic_Arena)
	mem.dynamic_arena_init(a, alignment = 64)
	return mem.dynamic_arena_allocator(a)
}

_threadScan :: proc(task: thread.Task) {
	// ok:bool
	context.allocator = task.allocator //chan.recv(runChan)
	// assert(ok)
	tc := (cast(^ThreadContext)task.data)^
	tc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	tb: u32 = 0
	tt: u32 = 0
	for &ray, i in tc.rays {
		b, t, sID := _intersectBVH(tc.colTree, &ray) //, tc.colTree.rootNodeIdx) // this switchs it back to recursive search
		tb += b
		tt += t
		if ray.t < math.F32_MAX && sID >= 0 {
			key := u32(i) + tc.offset
			if key < tc.offset || key > tc.offset + u32(len(tc.rays)) {
				fmt.printfln(
					"task %v is attempting to write key %v outside of its range[%v,%v)",
					task.user_index,
					key,
					tc.offset,
					tc.offset + u32(len(tc.rays)),
				)
			}
			append(&tc.hit, Hit{key, sID, ray.t})
		}
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
	chan.send(comms, tc)
	free_all(runners[task.user_index].allocator)
	chan.send(runChan, runners[task.user_index].allocator)
	sync.wait_group_done(completeGroup) //signal that this task is complete
}

saveBVH :: proc(path: string, colTree: ^CollisionTree) -> int {
	file, err := os.open(path, {.Create, .Write, .Read})
	defer os.close(file)
	if err != nil {
		fmt.printfln("file error: %v", err)
		return 0
	}
	binary, encError := enc.marshal(colTree^, {})
	if encError != nil {
		fmt.printfln("encoding error:%v", encError)
		return 0
	}
	defer delete(binary)
	written, writeEr := os.write(file, binary)
	if writeEr != nil {
		fmt.printfln("write error: %v", writeEr)
		return 0
	}
	return written
}

loadBVH :: proc(path: string, inputTri: []Shape) -> ^CollisionTree {
	file, err := os.open(path, {.Create, .Write, .Read})
	defer os.close(file)
	if err != nil do return nil
	binary, _ := os.read_entire_file(file, context.allocator)
	colTree := new(CollisionTree)
	unerr := enc.unmarshal(binary, colTree)
	if unerr != nil do return nil
	colTree.tri = inputTri
	return colTree
}
