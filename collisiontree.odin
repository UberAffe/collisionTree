package collisiontree

import "base:runtime"
import enc "core:encoding/json"
import "core:fmt"
import "core:log"
import "core:math"
import la "core:math/linalg"
import "core:mem"
import "core:os"
import "core:prof/spall"
import "core:sync"
import "core:sync/chan"
import "core:thread"
import time "core:time"

dyn_pool: mem.Dynamic_Pool
pool_allocator: mem.Allocator
pool: thread.Pool
runners: [dynamic]TaskRunner
contexts: [dynamic]ThreadContext
commsAllocator: mem.Allocator
comms: chan.Chan(^ResponseContext)
g_logger: log.Logger
runChanAlloc: mem.Allocator
runChan: chan.Chan(mem.Allocator)
freeMutex: ^sync.Mutex
completeGroup: ^sync.Wait_Group


_batchCleanup :: proc() {
	free_all(commsAllocator)
	chan.close(comms)
	chan.destroy(comms)
	for c in contexts {
		delete(c.rc.hits)
		free(c.rc)
	}
	for &r in runners{
		r={}
	}
}
// This will automatically call the cleanup process as if it was a defer call from
// the scope of the calling function
@(deferred_none = _batchCleanup)
collisionTreeBatchedRayScan :: proc(
	colTree: ^CollisionTree,
	rays: []Ray,
	batchCount: int,
	loc := #caller_location,
) -> (
	chan.Chan(^ResponseContext, .Recv),
	int,
) {
	offset: u32 = 0
	maxEnd := u32(len(rays))
	tCount := u32(batchCount)
	scanSize:= maxEnd / tCount
	scanSize = tCount * scanSize < maxEnd ? scanSize + 1 : scanSize
	err: runtime.Allocator_Error
	comms, err = chan.create_buffered(chan.Chan(^ResponseContext), tCount, commsAllocator)
	assert(err == .None)
	for len(runners) < int(tCount) {
		append(&runners, TaskRunner{})
		append(&contexts, ThreadContext{})
	}
	// adding to the group before spawning the tasks to ensure that any quick tasks actually subtract from the wait group.
	sync.wait_group_add(completeGroup, int(tCount))
	for i in 0 ..< tCount {
		run := &runners[i]
		ok: bool
		if run==nil do deref()
		run.task = _threadScan
		run.allocator, ok = chan.recv(runChan)
		assert(ok)
		end := math.min(offset + scanSize, maxEnd)
		contexts[i].colTree = colTree
		if offset > end {
			fmt.println(offset, ":", end)
		}
		contexts[i].rays = rays[offset:end]
		contexts[i].offset = offset
		offset = end
		contexts[i].rc = new(ResponseContext)
		contexts[i].rc.hits = make([dynamic]Hit, 0, scanSize)//, loc = loc)
		contexts[i].rc.searchTime = 0
		thread.pool_add_task(&pool, run.allocator, run.task, rawptr(&contexts[i]), int(i))
	}
	fmt.printfln("added %v tasks",tCount)
	return chan.as_recv(comms), int(tCount)
}

collisionTreeInit :: proc(threadCount: int = 1) {
	commsAllocator = _newArenaAllocator()
	completeGroup = new(sync.Wait_Group)
	freeMutex = new(sync.Mutex)
	runChanAlloc = _newArenaAllocator()
	err: runtime.Allocator_Error
	runChan, err = chan.create_buffered(
		chan.Chan(mem.Allocator),
		os.get_processor_core_count(),
		runChanAlloc,
	)
	assert(err == .None)
	for i in 0 ..< threadCount {
		chan.send(chan.as_send(runChan), _newArenaAllocator())
	}
	mem.dynamic_pool_init(&dyn_pool)
	pool_allocator = mem.dynamic_pool_allocator(&dyn_pool)
	thread.pool_init(&pool, pool_allocator, threadCount)
	thread.pool_start(&pool)
	runners = make([dynamic]TaskRunner)
	contexts = make([dynamic]ThreadContext)
}

collistionTreeCleanup :: proc() {
	chan.close(&comms)
	chan.destroy(comms)
	free_all(commsAllocator)
	mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)commsAllocator.data)
	free(completeGroup)
	free(freeMutex)
	for run in runners {
		free_all(run.allocator)
		mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)run.allocator.data)
	}
	delete(runners)
	chan.close(&runChan)
	chan.destroy(runChan)
	free_all(runChanAlloc)
	mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)runChanAlloc.data)
	delete(contexts)
	thread.pool_shutdown(&pool)
	thread.pool_destroy(&pool)
	free_all(pool_allocator)
	mem.dynamic_pool_destroy(cast(^mem.Dynamic_Pool)pool_allocator.data)

}

waitForBatch :: proc() {
	sync.wait(completeGroup)
}

//default wait is 1 ms
isBatchComplete :: proc(timeOut: time.Duration = 1_000) -> bool {
	return sync.wait_group_wait_with_timeout(completeGroup, timeOut)
}

_newArenaAllocator :: proc(loc := #caller_location) -> mem.Allocator {
	a := new(mem.Dynamic_Arena, loc = loc)
	mem.dynamic_arena_init(a, alignment = 64)
	return mem.dynamic_arena_allocator(a)
}

_threadScan :: proc(task: thread.Task) {
	when PROFILING {
		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	context.allocator = task.allocator
	if task.data==nil do deref()
	tc := (cast(^ThreadContext)task.data)^
	tc.rc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	tb: u32 = 0
	tt: u32 = 0
	for &ray, i in tc.rays {
		b, t, sID := _intersectBVH(tc.colTree, &ray) //, tc.colTree.rootNodeIdx) // this switchs it back to recursive search
		tb += b
		tt += t
		if ray.t < MAX && sID >= 0 {
			key := u32(i) + tc.offset
			append(&tc.rc.hits, Hit{key, sID, ray.t})
		}
	}
	time.stopwatch_stop(&sw)
	tc.rc.searchTime = time.stopwatch_duration(sw)
	chan.send(comms, tc.rc)
	fmt.printf("|Task %v sent comms|",task.user_index)
	free_all(runners[task.user_index].allocator)
	chan.send(runChan, runners[task.user_index].allocator)
	fmt.printf("|Task %v sent alloc|",task.user_index)
	sync.wait_group_done(completeGroup) //signal that this task is complete
	fmt.printfln("|Task %v completed",task.user_index)
}

saveBVH :: proc(path: string, colTree: ^CollisionTree) -> int {
	file, err := os.open(path, {.Create, .Write, .Read})
	if err != nil {
		fmt.printfln("file error: %v", err)
		return 0
	}
	defer os.close(file)
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
