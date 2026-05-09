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
import ch "core:sync/chan"
import "core:thread"
import time "core:time"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
scanSize: uint

fl3 :: [3]f32
ui2 :: [2]uint

AABB :: struct #align (4) {
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
	O, D: fl3,
	t:    f32,
}

BVHNode :: struct #align (4) {
	aabb:      AABB `json:"aabb"`, //3d bounds
	leftFirst: uint `json:"leftFirst"`,
	triCount:  uint `json:"triCount"`,
	//total size 32 bytes
}

ThreadContext :: struct {
	offset:     uint,
	searchTime: time.Duration,
	colTree:    ^CollisionTree,
	rays:       []Ray,
	Hit:        map[uint]uint,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

CollisionTree :: struct {
	tri:         []^Shape `json:"-"`,
	shapeIdx:    []uint `json:"shapeIdx"`,
	bvhNode:     [dynamic]BVHNode `json:"bvhNode"`,
	rootNodeIdx: uint `json:"rootNodeIdx"`,
	nodesUsed:   uint `json:"nodesUsed"`,
}

num_CPU: int
dyn_pool: mem.Dynamic_Pool
pool_allocator: mem.Allocator
pool: thread.Pool
runners: [dynamic]TaskRunner
contexts: [dynamic]ThreadContext
commsAllocator: mem.Allocator
comms: ch.Chan(^ThreadContext)
g_logger: log.Logger
freeAllocators: [dynamic]mem.Allocator
completeGroup: ^sync.Wait_Group
// jsMarshalOptions := js.Marshal_Options {
// 	.JSON5,
// 	false,
// 	false,
// 	0,
// 	false,
// 	true,
// 	false,
// 	false,
// 	true,
// 	0,
// 	false,
// 	false,
// }


collisionTreeInit :: proc(threadCount: int = 1) {
	commsAllocator = _newArenaAllocator()
	completeGroup = new(sync.Wait_Group)
	freeAllocators = make([dynamic]mem.Allocator)
	num_CPU = threadCount
	mem.dynamic_pool_init(&dyn_pool)
	pool_allocator = mem.dynamic_pool_allocator(&dyn_pool)
	thread.pool_init(&pool, pool_allocator, num_CPU)
	thread.pool_start(&pool)
}

collistionTreeCleanup :: proc() {
	delete(runners)
	delete(contexts)
	for run in runners {
		free_all(run.allocator)
		mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)run.allocator.data)
	}
	mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)commsAllocator.data)
	for al in freeAllocators {
		mem.dynamic_arena_destroy(cast(^mem.Dynamic_Arena)al.data)
	}
	delete(freeAllocators)
	thread.pool_shutdown(&pool)
	thread.pool_destroy(&pool)
	free_all(pool_allocator)
	mem.dynamic_pool_destroy(cast(^mem.Dynamic_Pool)pool_allocator.data)
	ch.destroy(comms)
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

collisionTreeBatchedRayScan :: proc(
	colTree: ^CollisionTree,
	rays: []Ray,
	maxBatchSize := 640,
) -> (
	ch.Chan(^ThreadContext, .Recv),
	int,
) {
	free_all(commsAllocator)
	ch.destroy(comms)
	delete(runners)
	delete(contexts)
	offset: uint = 0
	maxEnd := uint(len(rays))
	scanSize = math.min(maxEnd / uint(num_CPU), uint(maxBatchSize))
	tCount := maxEnd / scanSize
	c, err := ch.create_buffered(ch.Chan(^ThreadContext), tCount, commsAllocator)
	assert(err == .None)
	comms = c
	runners = make([dynamic]TaskRunner, tCount, tCount)
	contexts = make([dynamic]ThreadContext, tCount, tCount)
	// adding to the group before spawning the tasks to ensure that any quick tasks actually subtract from the wait group.
	fmt.println("adding to waitgroup")
	sync.wait_group_add(completeGroup, int(tCount))
	fmt.printfln("added %v to wait group", tCount)
	for &run, i in runners {
		run.task = _threadScan
		run.allocator = len(freeAllocators) > 0 ? pop(&freeAllocators) : _newArenaAllocator()
		contexts[i].offset = 0
		end := math.min(offset + scanSize, maxEnd)
		contexts[i].colTree = colTree
		contexts[i].rays = rays[offset:end]
		contexts[i].offset = offset
		offset += scanSize
		contexts[i].Hit = make(map[uint]uint, run.allocator)
		fmt.printfln("creating %v", i)
		thread.pool_add_task(&pool, run.allocator, run.task, rawptr(&contexts[i]), i)
	}
	return ch.as_recv(comms), int(tCount)
}

_newArenaAllocator :: proc() -> mem.Allocator {
	a := new(mem.Dynamic_Arena)
	mem.dynamic_arena_init(a, alignment = 64)
	return mem.dynamic_arena_allocator(a)
}

_threadScan :: proc(task: thread.Task) {
	// context.logger = g_logger
	fmt.printfln("started %v", task.user_index)
	// if true do return
	context.allocator = task.allocator
	tc := cast(^ThreadContext)task.data
	tc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	tb: uint = 0
	tt: uint = 0
	for &ray, i in tc.rays {
		b, t, sID := _intersectBVH(tc.colTree, &ray)
		tb += b
		tt += t
		if ray.t < MAX_F32 do tc.Hit[uint(i) + tc.offset] = sID
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
	fmt.printfln(
		"thread %v searched %v rays among %v b and %v s in %v",
		task.user_index,
		len(tc.rays),
		tb,
		tt,
		tc.searchTime,
	)
	ch.send(comms, tc)
	free_all(runners[task.user_index].allocator)
	append(&freeAllocators, runners[task.user_index].allocator)
	sync.wait_group_done(completeGroup) //signal that this task is complete
	fmt.printfln("task %v is fully complete", task.user_index)
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
_intersectBVHRecursive :: proc(colTree: ^CollisionTree, ray: ^Ray, nodeIdx: uint) -> (uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	if !_intersectAABBBool(ray^, colTree.bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations
	if colTree.bvhNode[nodeIdx].triCount > 0 {
		for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
			_intersectShape(
				colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]^,
				ray,
			)
			triIterations += 1
		}
	} else {
		b, t: uint
		b, t = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst)
		bvhIterations += b
		triIterations += t
		b, t = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst + 1)
		bvhIterations += b
		triIterations += t
	}
	return bvhIterations, triIterations
}
_intersectBVHLoop :: proc(colTree: ^CollisionTree, ray: ^Ray) -> (uint, uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	node := &colTree.bvhNode[colTree.rootNodeIdx]
	idStack := make([dynamic]^BVHNode, 0, 64)
	sID: uint
	for {
		if (_isLeaf(node^)) {
			prevT: f32
			for i in 0 ..< node.triCount {
				prevT = ray.t
				curID := colTree.shapeIdx[node.leftFirst + i]
				_intersectShape(colTree.tri[sID]^, ray)
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
	tx1 := (bmin.x - ray.O.x) / ray.D.x
	tx2 := (bmax.x - ray.O.x) / ray.D.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) / ray.D.y
	ty2 := (bmax.y - ray.O.y) / ray.D.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) / ray.D.z
	tz2 := (bmax.z - ray.O.z) / ray.D.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return tmax >= tmin && tmin < ray.t && tmax > 0
}
_intersectAABBFloat :: proc(ray: Ray, b: AABB) -> f32 {
	bmin := b.lower
	bmax := b.upper
	tx1 := (bmin.x - ray.O.x) / ray.D.x
	tx2 := (bmax.x - ray.O.x) / ray.D.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) / ray.D.y
	ty2 := (bmax.y - ray.O.y) / ray.D.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) / ray.D.z
	tz2 := (bmax.z - ray.O.z) / ray.D.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin : MAX_F32
}

BuildBVH :: proc(inputTri: []^Shape) -> ^CollisionTree {
	colTree := new(CollisionTree)
	colTree.rootNodeIdx = 0
	colTree.nodesUsed = 2
	colTree.tri = inputTri
	length := len(colTree.tri)
	colTree.shapeIdx = make([]uint, length)
	colTree.bvhNode = make([dynamic]BVHNode, 0, 2 * length)
	for &t, i in colTree.tri {
		colTree.shapeIdx[i] = uint(i)
	}
	if len(colTree.bvhNode) <= int(colTree.rootNodeIdx) do append(&colTree.bvhNode, BVHNode{})
	colTree.bvhNode[colTree.rootNodeIdx].triCount = uint(length)
	_UpdateNodeBounds(colTree, colTree.rootNodeIdx)
	_Subdivide(colTree, colTree.rootNodeIdx)
	return colTree
}

_UpdateNodeBounds :: proc(colTree: ^CollisionTree, nodeIdx: uint) {
	colTree.bvhNode[nodeIdx].aabb = {{-MAX_F32, -MAX_F32, -MAX_F32}, {MAX_F32, MAX_F32, MAX_F32}}
	fmt.println(colTree.bvhNode[nodeIdx].triCount)
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

_Subdivide :: proc(colTree: ^CollisionTree, nodeIdx: uint) {
	fmt.printfln("Division: %v, Count: %v", nodeIdx, colTree.bvhNode[nodeIdx].triCount)
	if colTree.bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	extent := colTree.bvhNode[nodeIdx].aabb.upper - colTree.bvhNode[nodeIdx].aabb.lower
	parentCost :=
		f32(colTree.bvhNode[nodeIdx].triCount) *
		(extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
	// axis := 0
	// if extent.y > extent.x do axis = 1
	// if extent.z > extent[axis] do axis = 2
	// splitPos := bvhNode[nodeIdx].aabb.lower[axis] + extent[axis] * .5
	bestAxis := -1
	bestPos, bestCost: f32 = 0, MAX_F32
	for axis in 0 ..< 3 {
		for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
			shape := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
			candidatePos := shape.centroid[axis]
			cost := _evaluateSAH(colTree, &colTree.bvhNode[nodeIdx], axis, candidatePos)
			if (cost < bestCost) {
				bestPos = candidatePos
				bestCost = cost
				bestAxis = axis
			}
		}
	}
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
	leftCount := uint(i) - colTree.bvhNode[nodeIdx].leftFirst
	if leftCount == 0 || leftCount == colTree.bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := colTree.nodesUsed
	colTree.nodesUsed += 1
	rightChildIdx := colTree.nodesUsed
	colTree.nodesUsed += 1
	for len(colTree.bvhNode) <= int(rightChildIdx) do append(&colTree.bvhNode, BVHNode{})
	colTree.bvhNode[leftChildIdx].leftFirst = colTree.bvhNode[nodeIdx].leftFirst
	colTree.bvhNode[nodeIdx].leftFirst = leftChildIdx
	colTree.bvhNode[leftChildIdx].triCount = leftCount
	colTree.bvhNode[rightChildIdx].leftFirst = uint(i)
	colTree.bvhNode[rightChildIdx].triCount = colTree.bvhNode[nodeIdx].triCount - leftCount
	colTree.bvhNode[nodeIdx].triCount = 0
	_UpdateNodeBounds(colTree, leftChildIdx)
	_UpdateNodeBounds(colTree, rightChildIdx)
	_Subdivide(colTree, leftChildIdx)
	_Subdivide(colTree, rightChildIdx)
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
