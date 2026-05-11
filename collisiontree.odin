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

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
scanSize: u32

fl3 :: [3]f32
ui2 :: [2]u32

AABB :: struct {
	upper: fl3 `json:"upper"`,
	lower: fl3 `json:"lower"`,
}

Shape :: struct {
	aabb:     AABB,
	centroid: fl3,
	type:     ShapeType,
}

Tri :: struct {
	vertex0, vertex1, vertex2: fl3,
}

ShapeType :: union {
	Tri,
}

Ray :: struct {
	O, D, rD: fl3,
	t:        f32,
}

Hit :: struct {
	rayID:   u32,
	shapeID: int,
	dist:    f32,
}

BVHNode :: struct #align(32){
	using aabb:      AABB `json:"aabb"`, //3d bounds
	leftFirst: u32 `json:"leftFirst"`,
	triCount:  u32 `json:"triCount"`,
	//total size 32 bytes
}

ThreadContext :: struct {
	offset:     u32,
	searchTime: time.Duration,
	colTree:    ^CollisionTree,
	rays:       []Ray,
	hit:        [dynamic]Hit,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

CollisionTree :: struct #align(64){
	bvhNode:     []BVHNode `json:"bvhNode"`,
	tri:         []^Shape `json:"-"`,
	shapeIdx:    []u32 `json:"shapeIdx"`,
	rootNodeIdx: u32 `json:"rootNodeIdx"`,
	nodesUsed:   u32 `json:"nodesUsed"`,
	splitChecks: u32 `json:"splitChecks"`,
	longestOnly: bool `json:"longestOnly"`
}

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
	runChan, err = chan.create_buffered(chan.Chan(mem.Allocator), threadCount, runChanAlloc)
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
	tCount:= u32(batchCount)
	scanSize = maxEnd/tCount
	scanSize= tCount*scanSize<maxEnd ? scanSize+1 : scanSize
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
		contexts[i].hit = make([dynamic]Hit,0,scanSize)
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
	// context.logger = g_logger
	// fmt.printfln("started %v", task.user_index)
	// if true do return
	tc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	tb: u32 = 0
	tt: u32 = 0
	for &ray, i in tc.rays {
		b, t, sID := _intersectBVH(tc.colTree, &ray) //, tc.colTree.rootNodeIdx)
		tb += b
		tt += t
		if ray.t < MAX_F32 && sID >= 0 {
			key := u32(i) + tc.offset
			if key < tc.offset || key > tc.offset + u32(len(tc.rays)) {
				fmt.printfln(
					"task %v is attempting to write key %v outside of its range[%v,%v)",
					task.user_index,
					key,
					tc.offset,
					tc.offset + u32(len(tc.rays))
				)
			}
			append(&tc.hit, Hit{key, sID, ray.t})
		}
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
	// fmt.printfln(
	// 	"thread %v hit %v/%v rays among %v b and %v s in %v",
	// 	task.user_index,
	// 	len(tc.hit),
	// 	len(tc.rays),
	// 	tb,
	// 	tt,
	// 	tc.searchTime,
	// )
	chan.send(comms, tc)
	free_all(runners[task.user_index].allocator)
	chan.send(runChan, runners[task.user_index].allocator)
	sync.wait_group_done(completeGroup) //signal that this task is complete
	// fmt.printfln("task %v is fully complete", task.user_index)
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

loadBVH :: proc(path: string, inputTri: []^Shape) -> ^CollisionTree {
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

_intersectBVH :: proc {
	_intersectBVHRecursive,
	_intersectBVHLoop,
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
_intersectBVHRecursive :: proc(
	colTree: ^CollisionTree,
	ray: ^Ray,
	nodeIdx: u32,
) -> (
	u32,
	u32,
	int,
) {
	bvhIterations := u32(1)
	triIterations := u32(0)
	if !_intersectAABBBool(ray^, colTree.bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations, 0
	sID := -1
	if colTree.bvhNode[nodeIdx].triCount > 0 {
		prevT := ray.t
		for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
			testSID := int(colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i])
			assert(testSID >= 0)
			_intersectShape(colTree.tri[testSID]^, ray)
			if ray.t < prevT {
				prevT = ray.t
				sID = testSID
			}
			triIterations += 1
		}
	} else {
		b, t: u32
		b, t, sID = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst)
		prevT := ray.t
		testSID: int
		bvhIterations += b
		triIterations += t
		b, t, testSID = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst + 1)
		if ray.t < prevT do sID = testSID
		bvhIterations += b
		triIterations += t
	}
	return bvhIterations, triIterations, sID
}
_intersectBVHLoop :: proc(colTree: ^CollisionTree, ray: ^Ray) -> (u32, u32, int) {
	bvhIterations := u32(1)
	triIterations := u32(0)
	node := &colTree.bvhNode[colTree.rootNodeIdx]
	idStack := make([dynamic]^BVHNode, 0, 64)
	sID := -1
	for {
		if (_isLeaf(node^)) {
			prevT: f32
			for i in 0 ..< node.triCount {
				prevT = ray.t
				curID := int(colTree.shapeIdx[node.leftFirst + i])
				assert(curID >= 0)
				_intersectShape(colTree.tri[curID]^, ray)
				if ray.t < prevT do sID = curID
			}
			triIterations += node.triCount
			if len(idStack) == 0 do break
			node = pop(&idStack)
			continue
		}
		child1 := &colTree.bvhNode[node.leftFirst]
		child2 := &colTree.bvhNode[node.leftFirst + 1]
		dist1 := _intersectAABBFloat(ray^, child1.aabb)
		dist2 := _intersectAABBFloat(ray^, child2.aabb)
		bvhIterations += 2
		if dist1 > dist2 {
			_swap(&dist1, &dist2)
			_swap(&child1, &child2)
		}
		if dist1 == MAX_F32 {
			if len(idStack) == 0 do break
			node = pop(&idStack)
		} else {
			node = child1
			if dist2 != MAX_F32 do append(&idStack, child2)
		}
	}
	return bvhIterations, triIterations, sID
}

_intersectShape :: proc(shape: Shape, ray: ^Ray) {
	switch type in shape.type {
	case Tri:
		_intersectTri(type, ray)
	}
}

_intersectAABBBool :: proc(ray: Ray, b: AABB) -> bool {
	bmin := b.lower
	bmax := b.upper
	tx1 := (bmin.x - ray.O.x) * ray.rD.x
	tx2 := (bmax.x - ray.O.x) * ray.rD.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) * ray.rD.y
	ty2 := (bmax.y - ray.O.y) * ray.rD.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) * ray.rD.z
	tz2 := (bmax.z - ray.O.z) * ray.rD.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return tmax >= tmin && tmin < ray.t && tmax > 0
}
_intersectAABBFloat :: proc(ray: Ray, b: AABB) -> f32 {
	bmin := b.lower
	bmax := b.upper
	tx1 := (bmin.x - ray.O.x) * ray.rD.x
	tx2 := (bmax.x - ray.O.x) * ray.rD.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) * ray.rD.y
	ty2 := (bmax.y - ray.O.y) * ray.rD.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) * ray.rD.z
	tz2 := (bmax.z - ray.O.z) * ray.rD.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin : MAX_F32
}

BuildBVH :: proc(inputTri: []^Shape, divisionChecks:u32=16, longestOnly:bool=false) -> ^CollisionTree {
	colTree := new(CollisionTree)
	colTree.rootNodeIdx = 0
	colTree.nodesUsed = 1
	colTree.tri = inputTri
	length := len(colTree.tri)
	// memPadding:= new(i32)
	colTree.bvhNode = make_slice([]BVHNode,2*length)
	colTree.shapeIdx = make([]u32, length)
	//([]BVHNode, 2 * length)
	for &t, i in colTree.tri {
		colTree.shapeIdx[i] = u32(i)
	}
	// if len(colTree.bvhNode) <= int(colTree.rootNodeIdx) do append(&colTree.bvhNode, BVHNode{})
	colTree.bvhNode[colTree.rootNodeIdx].triCount = u32(length)
	colTree.longestOnly=longestOnly
	colTree.splitChecks=divisionChecks
	_UpdateNodeBounds(colTree, colTree.rootNodeIdx)
	_Subdivide(colTree, colTree.rootNodeIdx)
	return colTree
}

_UpdateNodeBounds :: proc(colTree: ^CollisionTree, nodeIdx: u32) {
	colTree.bvhNode[nodeIdx].aabb = {{-MAX_F32, -MAX_F32, -MAX_F32}, {MAX_F32, MAX_F32, MAX_F32}}
	// fmt.println(colTree.bvhNode[nodeIdx].triCount)
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&colTree.bvhNode[nodeIdx], s.aabb)
	}
}

_GrowAABB :: proc {
	_growAABBWithBox,
	_growAABBWithNode,
}

_growAABBWithNode :: proc(node: ^BVHNode, leaf: AABB) {
	node.aabb.lower = _fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = _fmaxf(node.aabb.upper, leaf.upper)
}
_growAABBWithBox :: proc(node: ^AABB, leaf: AABB) {
	node.lower = _fminf(node.lower, leaf.lower)
	node.upper = _fmaxf(node.upper, leaf.upper)
}

_Subdivide :: proc(colTree: ^CollisionTree, nodeIdx: u32) {
	// fmt.printfln("Division: %v, Count: %v", nodeIdx, colTree.bvhNode[nodeIdx].triCount)
	if colTree.bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(colTree.bvhNode[nodeIdx])
	bestAxis, bestPos, bestCost := _findBestSplitPlane(colTree, nodeIdx)
	if bestCost >= parentCost do return
	//in place partition
	i := int(colTree.bvhNode[nodeIdx].leftFirst)
	j := i + int(colTree.bvhNode[nodeIdx].triCount) - 1
	for i <= j {
		if colTree.tri[colTree.shapeIdx[i]].centroid[bestAxis] < bestPos {
			i += 1
		} else {
			_swap(&colTree.shapeIdx[i], &colTree.shapeIdx[j])
			j -= 1
		}
	}
	//abort split if one side empty
	leftCount := u32(i) - colTree.bvhNode[nodeIdx].leftFirst
	if leftCount == 0 || leftCount == colTree.bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := colTree.nodesUsed
	colTree.nodesUsed += 1
	rightChildIdx := colTree.nodesUsed
	colTree.nodesUsed += 1
	// for len(colTree.bvhNode) <= int(rightChildIdx) do append(&colTree.bvhNode, BVHNode{})
	colTree.bvhNode[leftChildIdx].leftFirst = colTree.bvhNode[nodeIdx].leftFirst
	colTree.bvhNode[nodeIdx].leftFirst = leftChildIdx
	colTree.bvhNode[leftChildIdx].triCount = leftCount
	colTree.bvhNode[rightChildIdx].leftFirst = u32(i)
	colTree.bvhNode[rightChildIdx].triCount = colTree.bvhNode[nodeIdx].triCount - leftCount
	colTree.bvhNode[nodeIdx].triCount = 0
	_UpdateNodeBounds(colTree, leftChildIdx)
	_UpdateNodeBounds(colTree, rightChildIdx)
	_Subdivide(colTree, leftChildIdx)
	_Subdivide(colTree, rightChildIdx)
}

_calculateNodeCost :: proc(node: BVHNode) -> f32 {
	extent := node.aabb.upper - node.aabb.lower
	return f32(node.triCount) * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}
Bin::struct{
	bounds:AABB,
	triCount:u32
}
_findBestSplitPlane :: proc(colTree: ^CollisionTree, nodeIdx: u32) -> (int, f32, f32) {
	node := &colTree.bvhNode[nodeIdx]
	bins := make([dynamic]Bin,colTree.splitChecks,colTree.splitChecks)
	defer delete(bins)
	bestAxis := -1
	bestPos, bestCost: f32 = 0, MAX_F32
	for axis in 0 ..< 3 {
		boundMin :f32= MAX_F32
		boundMax :f32= -MAX_F32
		for i in 0..<node.triCount{
			s:= colTree.tri[colTree.shapeIdx[node.leftFirst+i]]
			boundMin=math.min(boundMin,s.centroid[axis])
			boundMax=math.max(boundMax,s.centroid[axis])
		}
		if boundMin == boundMax do continue
		scale := f32(colTree.splitChecks)/(boundMax - boundMin)
		for i in 0 ..< node.triCount {
			s:=colTree.tri[colTree.shapeIdx[node.leftFirst+i]]
			binIdx:= int(math.min(f32(colTree.splitChecks)-1, (s.centroid[axis]-boundMin)*scale))
			assert(binIdx>=0)
			bins[binIdx].triCount+=0
			_GrowAABB(&bins[binIdx].bounds,s.aabb)
		}
		leftArea:=make([dynamic]f32,colTree.splitChecks,colTree.splitChecks)
		rightArea:=make([dynamic]f32,colTree.splitChecks,colTree.splitChecks)
		leftcount:=make([dynamic]f32,colTree.splitChecks,colTree.splitChecks)
		rightcount:=make([dynamic]f32,colTree.splitChecks,colTree.splitChecks)
		defer delete(leftArea)
		defer delete(rightArea)
		defer delete(leftcount)
		defer delete(rightcount)
		leftBox,rightBox:AABB={},{}
		lSum, rSum:f32
		for i in 0..<colTree.splitChecks-1{
			lSum+=f32(bins[i].triCount)
			leftcount[i]=lSum
			_GrowAABB(&leftBox,bins[i].bounds)
			leftArea[i]=_areaAABB(leftBox)
			rSum+=f32(bins[colTree.splitChecks-2-i].triCount)
			rightcount[colTree.splitChecks-2-i]=rSum
			_GrowAABB(&rightBox,bins[colTree.splitChecks-2-i].bounds)
			rightArea[colTree.splitChecks-2-i]=_areaAABB(rightBox)
		}
		scale=(boundMax-boundMin)/f32(colTree.splitChecks)
		for i in 0..<colTree.splitChecks-1{
			planeCost:= leftcount[i]*leftArea[i]+rightcount[i]*rightArea[i]
			if planeCost < bestCost{
				bestAxis=axis
				bestPos=boundMin+scale*f32((i+1))
				bestCost=planeCost
			}
		}
	}
	return bestAxis, bestPos, bestCost
}

_evaluateSAH :: proc(colTree: ^CollisionTree, node: ^BVHNode, axis: int, pos: f32) -> f32 {
	leftBox, rightBox: AABB
	leftCount, rightCount: f32 = 0, 0
	for i in 0 ..< node.triCount {
		shape := colTree.tri[colTree.shapeIdx[node.leftFirst + i]]
		if shape.centroid[axis] < pos {
			leftCount += 1
			_GrowAABB(&leftBox, shape.aabb)
		} else {
			rightCount += 1
			_GrowAABB(&rightBox, shape.aabb)
		}
	}
	cost := leftCount * _areaAABB(leftBox) + rightCount * _areaAABB(rightBox)
	return cost > 0 ? cost : MAX_F32
}

_areaAABB :: proc(aabb: AABB) -> f32 {
	e := aabb.upper - aabb.lower
	return e.x * e.y + e.y * e.z + e.z * e.x
}

_swap :: proc(first, second: ^$T) {
	t := first^
	first^ = second^
	second^ = t
}
_isLeaf :: proc(node: BVHNode) -> bool {
	return node.triCount > 0
}
_fminf :: proc(first, second: fl3) -> fl3 {
	return {min(first.x, second.x), min(first.y, second.y), min(first.z, second.z)}
}
_fmaxf :: proc(first, second: fl3) -> fl3 {
	return {max(first.x, second.x), max(first.y, second.y), max(first.z, second.z)}
}

_getTriangleAABB :: proc(leaf: Tri) -> AABB {
	bounds: AABB = {{-MAX_F32, -MAX_F32, -MAX_F32}, {MAX_F32, MAX_F32, MAX_F32}}
	bounds.lower = _fminf(bounds.lower, leaf.vertex0)
	bounds.lower = _fminf(bounds.lower, leaf.vertex1)
	bounds.lower = _fminf(bounds.lower, leaf.vertex2)
	bounds.upper = _fmaxf(bounds.upper, leaf.vertex0)
	bounds.upper = _fmaxf(bounds.upper, leaf.vertex1)
	bounds.upper = _fmaxf(bounds.upper, leaf.vertex2)
	return bounds
}

_intersectTri :: proc(triangle: Tri, ray: ^Ray) {
	edge1 := triangle.vertex1 - triangle.vertex0
	edge2 := triangle.vertex2 - triangle.vertex0
	h := la.cross(ray.D, edge2)
	a := la.dot(edge1, h)
	if a > -0.0001 && a < 0.0001 do return
	f := 1 / a
	s := ray.O - triangle.vertex0
	u := f * la.dot(s, h)
	if u < 0 || u > 1 do return
	q := la.cross(s, edge1)
	v := f * la.dot(ray.D, q)
	if v < 0 || u + v > 1 do return
	t := f * la.dot(edge2, q)
	if t > 0.0001 do ray.t = min(ray.t, t)
}
