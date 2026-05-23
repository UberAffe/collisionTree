package collisiontree

import "base:intrinsics"
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

AUTO_CLEAN::#config(ct_auto_clean,true)

pool: thread.Pool
contexts: [dynamic]ThreadContext
comms: chan.Chan(^ResponseContext)
g_logger: log.Logger
freeMutex: ^sync.Mutex
completeGroup: ^sync.Wait_Group
tCount:u32


_batchCleanup :: proc() {
	chan.close(comms)
	chan.destroy(comms)
	for i in 0..<tCount {
		delete(contexts[i].rc.hits)
		free(contexts[i].rc)
	}
}

@(deferred_none = _batchCleanup)
collisionTreeBatchedRayScan :: proc(
	colTree: ^CollisionTree,
	rays: []Ray,
	batchCount: int,
	commsAllocator:=context.allocator,
	loc := #caller_location,
) -> (
	chan.Chan(^ResponseContext, .Recv),
	int,
){
	offset: u32 = 0
	maxEnd := u32(len(rays))
	tCount = u32(batchCount)
	scanSize:= maxEnd / tCount
	scanSize = tCount * scanSize < maxEnd ? scanSize + 1 : scanSize
	err: runtime.Allocator_Error
	comms, err = chan.create_buffered(chan.Chan(^ResponseContext), tCount, commsAllocator)
	assert(err == .None)
	for len(contexts) < int(tCount) {
		append(&contexts, ThreadContext{})
	}
	// adding to the group before spawning the tasks to ensure that any quick tasks actually subtract from the wait group.
	sync.wait_group_add(completeGroup, int(tCount))
	for i in 0 ..< tCount {
		end := math.min(offset + scanSize, maxEnd)
		contexts[i].colTree = colTree
		if offset > end {
			log.warn(offset, ":", end)
		}
		contexts[i].rays = rays[offset:end]
		contexts[i].offset = offset
		offset = end
		contexts[i].rc = new(ResponseContext)
		contexts[i].rc.hits = make([dynamic]Hit, 0, scanSize)//, loc = loc)
		contexts[i].rc.searchTime = 0
		contexts[i].rc.batchID=i
		thread.pool_add_task(&pool, context.allocator, _threadScan, rawptr(&contexts[i]), int(i))
	}
	return chan.as_recv(comms), int(tCount)
}

collisionTreeInit :: proc(threadCount: int = 1, pool_allocator:=context.allocator) {
	g_logger=context.logger
	completeGroup = new(sync.Wait_Group)
	freeMutex = new(sync.Mutex)
	thread.pool_init(&pool, pool_allocator, threadCount)
	thread.pool_start(&pool)
	contexts = make([dynamic]ThreadContext)
}

collisionTreeCleanup :: proc() {
	free(completeGroup)
	free(freeMutex)
	delete(contexts)
	thread.pool_shutdown(&pool)
	thread.pool_destroy(&pool)
}

waitForBatch :: proc() {
	sync.wait(completeGroup)
}

//default wait is 1 micro second
isBatchComplete :: proc(timeOut: time.Duration = 1_000) -> bool {
	return sync.wait_group_wait_with_timeout(completeGroup, timeOut)
}

_threadScan :: proc(task: thread.Task) {
	context.logger=g_logger
	when PROFILING {
		buffer_backing, err := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
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
	log.debugf("for %v rays, we checked %v bounds and %v triangles, and hit %v times",len(tc.rays),tb,tt,len(tc.rc.hits))
	time.stopwatch_stop(&sw)
	tc.rc.searchTime = time.stopwatch_duration(sw)
	chan.send(comms, tc.rc)
	sync.wait_group_done(completeGroup) //signal that this task is complete
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
