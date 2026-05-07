package collisiontree

import tl "../ThreadLogger"
import "base:runtime"
import "core:fmt"
import "core:log"
import "core:math"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import os "core:os"
import "core:strconv"
import "core:strings"
import "core:thread"
import time "core:time"
import rl "vendor:raylib"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
N :: 64
scanSize: uint

fl3 :: [3]f32
ui2 :: [2]uint

AABB :: struct #align (4) {
	upper, lower: fl3,
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
	aabb:                AABB, //3d bounds
	leftFirst, triCount: uint,
	//total size 32 bytes
}

ThreadContext :: struct {
	offset:     uint,
	searchTime: time.Duration,
	rays:       []Ray,
	Pixels:     map[uint]f32,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

tri: []^Shape
shapeIdx: []uint
bvhNode: []BVHNode //N 64 byte pages for optimal loading
rootNodeIdx, nodesUsed: uint = 0, 2
num_CPU: int
remaining: uint
dyn_pool: mem.Dynamic_Pool
pool_allocator: mem.Allocator
pool: thread.Pool
runners: [dynamic]TaskRunner
contexts: [dynamic]ThreadContext
g_logger: log.Logger


main :: proc() {
	file, _ := os.open("logs/latest.log", {.Read, .Write, .Append, .Create})
	defer os.close(file)
	l := log.create_file_logger(file)
	defer log.destroy_file_logger(l)
	// g_logger=tl.CreateThreadedLogger(l)
	// context.logger=g_logger
	fmt.println("Collision Test Started")
	//defer mem.dynamic_pool_destroy(&dyn_pool)
	collisionTreeInit(os.get_processor_core_count())

	fmt.println("Bulding Test Triangles")
	inputTri := buildTestTriangles2()
	fmt.println("triangles built")
	bWatch := time.Stopwatch{}
	fmt.println("Building BVH")
	time.stopwatch_start(&bWatch)
	BuildBVH(inputTri)
	time.stopwatch_stop(&bWatch)
	fmt.println("BVH built")

	rl.InitWindow(640, 640, "test")
	defer rl.CloseWindow()
	searchTime: time.Duration
	camPos := fl3{-1.5, -.2, -2.5}
	p0 := fl3{-2.5, .8, -.5}
	p1 := fl3{-.5, .8, -.5}
	p2 := fl3{-2.5, -1.2, -.5}

	rays := make([dynamic]Ray,640 * 640,640 * 640)
	for &ray,i in rays {
		y := uint(i) / 640
		x := uint(i) % 640
		ray.O = camPos
		ray.D = la.normalize(
			(p0 + (p1 - p0) * (f32(x) / 640) + (p2 - p0) * (f32(y + 0) / 640)) - ray.O,
		)
		ray.t = MAX_F32
	}

	for !rl.WindowShouldClose() {
		// defer mem.dynamic_pool_free_all(&dyn_pool)
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		for &ray in rays{
			ray.t=MAX_F32
		}
		collisionTreeBatchedRayScan(rays[:])
		for thread.pool_num_outstanding(&pool) > 0 {
			searchTime += processThreadOutput(&pool)
		}
		searchTime += processThreadOutput(&pool)
		rl.DrawFPS(10, 10)
		rl.DrawText(
			fmt.ctprintf(
				"build time: %v\ncumulative search time: %v\naverage search time: %v\n409,600 rays across %v threads\ntriangles: %v\nTPR: %v",
				time.stopwatch_duration(bWatch),
				searchTime,
				searchTime / time.Duration(num_CPU),
				num_CPU,
				len(tri),
				searchTime / (640 * 640),
			),
			10,
			40,
			16,
			{0, 0, 0, 255},
		)
		rl.EndDrawing()
	}
	collistionTreeCleanup()
}

collisionTreeInit :: proc(threadCount: int) {
	runners = make([dynamic]TaskRunner, threadCount, threadCount)
	contexts = make([dynamic]ThreadContext, threadCount, threadCount)
	for &r in runners {
		a := new(mem.Dynamic_Arena)
		mem.dynamic_arena_init(a)
		r.task = threadScan
		r.allocator = mem.dynamic_arena_allocator(a)
	}
	num_CPU = threadCount
	mem.dynamic_pool_init(&dyn_pool)
	pool_allocator = mem.dynamic_pool_allocator(&dyn_pool)
	thread.pool_init(&pool, pool_allocator, num_CPU)
	thread.pool_start(&pool)
}

collisionTreeBatchedRayScan :: proc(rays: []Ray) {
	offset: uint = 0
	maxEnd:= uint(len(rays))
	scanSize= maxEnd/uint(num_CPU)
	// fmt.printfln("max: %v, scanSize: %v",maxEnd,scanSize)
	for run, i in runners {
		free_all(run.allocator)
		contexts[i].offset = 0
		end:= math.min(offset+scanSize,maxEnd)
		contexts[i].rays = rays[offset:end]
		// fmt.printfln("context %v rays length: %v",i,len(contexts[i].rays))
		contexts[i].offset = offset
		offset += scanSize
		contexts[i].Pixels = make(map[uint]f32,run.allocator)
		thread.pool_add_task(&pool, run.allocator, run.task, rawptr(&contexts[i]), i)
	}
}

collistionTreeCleanup :: proc() {
	thread.pool_shutdown(&pool)
	thread.pool_destroy(&pool)
}

processThreadOutput :: proc(pool: ^thread.Pool) -> time.Duration {
	task, ok := thread.pool_pop_done(pool)
	if ok {
		// fmt.printfln("drawing thread %v results", task.user_index)
		tc := cast(^ThreadContext)task.data
		for key, value in tc.Pixels {
			v := u8(500 - value * 55)
			x := key % 640
			y := key / 640
			rl.DrawPixelV({f32(x), f32(y)}, {v, v, v, 255})
		}
		free_all(task.allocator)
		return tc.searchTime
	}
	return 0
}

threadScan :: proc(task: thread.Task) {
	// context.logger = g_logger
	context.allocator=task.allocator
	// defer mem.free_all(task.allocator)
	tc := cast(^ThreadContext)task.data
	tc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	tb: uint = 0
	tt: uint = 0
	for &ray, i in tc.rays {
		// fmt.println(ray)
		b, t := intersectBVH(&ray)
		tb += b
		tt += t
		if ray.t < MAX_F32 do tc.Pixels[uint(i) + tc.offset] = ray.t
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
	fmt.printfln("thread %v searched %v b and %v s in %v", task.user_index, tb, tt, tc.searchTime)
}

intersectBVH :: proc {
	intersectBVHRecursive,
	intersectBVHLoop,
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
intersectBVHRecursive :: proc(ray: ^Ray, nodeIdx: uint) -> (uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	if !_intersectAABBBool(ray^, bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations
	if bvhNode[nodeIdx].triCount > 0 {
		for i in 0 ..< bvhNode[nodeIdx].triCount {
			intersectShape(tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]]^, ray)
			triIterations += 1
		}
	} else {
		b, t: uint
		b, t = intersectBVH(ray, bvhNode[nodeIdx].leftFirst)
		bvhIterations += b
		triIterations += t
		b, t = intersectBVH(ray, bvhNode[nodeIdx].leftFirst + 1)
		bvhIterations += b
		triIterations += t
	}
	return bvhIterations, triIterations
}
intersectBVHLoop :: proc(ray: ^Ray) -> (uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	node := &bvhNode[rootNodeIdx]
	idStack := make([dynamic]^BVHNode, 0, 64)
	defer delete_dynamic_array(idStack)
	for {
		if (isLeaf(node^)) {
			for i in 0 ..< node.triCount do intersectShape(tri[shapeIdx[node.leftFirst + i]]^, ray)
			triIterations += node.triCount
			if len(idStack) == 0 do break
			node = pop(&idStack)
		}
		child1 := &bvhNode[node.leftFirst]
		child2 := &bvhNode[node.leftFirst + 1]
		dist1 := _intersectAABBFloat(ray^, child1.aabb)
		dist2 := _intersectAABBFloat(ray^, child2.aabb)
		bvhIterations += 2
		if dist1 > dist2 {
			swap(&dist1, &dist2)
			swap(&child1, &child2)
		}
		if dist1 == MAX_F32 {
			if len(idStack) == 0 do break
			node = pop(&idStack)
		} else {
			node = child1
			if dist2 != MAX_F32 do append(&idStack, child2)
		}
	}
	return bvhIterations, triIterations
}

intersectShape :: proc(shape: Shape, ray: ^Ray) {
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

BuildBVH :: proc(inputTri: []^Shape) {
	tri = inputTri
	length := len(tri)
	shapeIdx = make([]uint, length)
	bvhNode = runtime.make_aligned([]BVHNode, 2 * length, 64)
	for &t, i in tri {
		shapeIdx[i] = uint(i)
	}
	bvhNode[rootNodeIdx].triCount = uint(length)
	UpdateNodeBounds(rootNodeIdx)
	Subdivide(rootNodeIdx)
}

UpdateNodeBounds :: proc(nodeIdx: uint) {
	bvhNode[nodeIdx].aabb = {{-MAX_F32, -MAX_F32, -MAX_F32}, {MAX_F32, MAX_F32, MAX_F32}}
	fmt.println(bvhNode[nodeIdx].triCount)
	for i in 0 ..< bvhNode[nodeIdx].triCount {
		s := tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]]
		GrowAABB(&bvhNode[nodeIdx], s.aabb)
	}
}

GrowAABB :: proc {
	_growAABBWithBox,
	_growAABBWithNode,
}

_growAABBWithNode :: proc(node: ^BVHNode, leaf: AABB) {
	node.aabb.lower = fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = fmaxf(node.aabb.upper, leaf.upper)
}
_growAABBWithBox :: proc(node: ^AABB, leaf: AABB) {
	node.lower = fminf(node.lower, leaf.lower)
	node.upper = fmaxf(node.upper, leaf.upper)
}

Subdivide :: proc(nodeIdx: uint) {
	fmt.printfln("Division: %v, Count: %v", nodeIdx, bvhNode[nodeIdx].triCount)
	if bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	extent := bvhNode[nodeIdx].aabb.upper - bvhNode[nodeIdx].aabb.lower
	parentCost :=
		f32(bvhNode[nodeIdx].triCount) *
		(extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
	// axis := 0
	// if extent.y > extent.x do axis = 1
	// if extent.z > extent[axis] do axis = 2
	// splitPos := bvhNode[nodeIdx].aabb.lower[axis] + extent[axis] * .5
	bestAxis := -1
	bestPos, bestCost: f32 = 0, MAX_F32
	for axis in 0 ..< 3 {
		for i in 0 ..< bvhNode[nodeIdx].triCount {
			shape := tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]]
			candidatePos := shape.centroid[axis]
			cost := evaluateSAH(&bvhNode[nodeIdx], axis, candidatePos)
			if (cost < bestCost) {
				bestPos = candidatePos
				bestCost = cost
				bestAxis = axis
			}
		}
	}
	if bestCost >= parentCost do return
	//in place partition
	i := int(bvhNode[nodeIdx].leftFirst)
	j := i + int(bvhNode[nodeIdx].triCount) - 1
	for i <= j {
		if tri[shapeIdx[i]].centroid[bestAxis] < bestPos {
			i += 1
		} else {
			swap(&shapeIdx[i], &shapeIdx[j])
			j -= 1
		}
	}
	//abort split if one side empty
	leftCount := uint(i) - bvhNode[nodeIdx].leftFirst
	if leftCount == 0 || leftCount == bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := nodesUsed
	nodesUsed += 1
	rightChildIdx := nodesUsed
	nodesUsed += 1
	bvhNode[leftChildIdx].leftFirst = bvhNode[nodeIdx].leftFirst
	bvhNode[nodeIdx].leftFirst = leftChildIdx
	bvhNode[leftChildIdx].triCount = leftCount
	bvhNode[rightChildIdx].leftFirst = uint(i)
	bvhNode[rightChildIdx].triCount = bvhNode[nodeIdx].triCount - leftCount
	bvhNode[nodeIdx].triCount = 0
	UpdateNodeBounds(leftChildIdx)
	UpdateNodeBounds(rightChildIdx)
	Subdivide(leftChildIdx)
	Subdivide(rightChildIdx)
}

evaluateSAH :: proc(node: ^BVHNode, axis: int, pos: f32) -> f32 {
	leftBox, rightBox: AABB
	leftCount, rightCount: f32 = 0, 0
	for i in 0 ..< node.triCount {
		shape := tri[shapeIdx[node.leftFirst + i]]
		if shape.centroid[axis] < pos {
			leftCount += 1
			GrowAABB(&leftBox, shape.aabb)
		} else {
			rightCount += 1
			GrowAABB(&rightBox, shape.aabb)
		}
	}
	cost := leftCount * areaAABB(leftBox) + rightCount * areaAABB(rightBox)
	return cost > 0 ? cost : MAX_F32
}

areaAABB :: proc(aabb: AABB) -> f32 {
	e := aabb.upper - aabb.lower
	return e.x * e.y + e.y * e.z + e.z * e.x
}

swap :: proc(first, second: ^$T) {
	t := first^
	first^ = second^
	second^ = t
}
isLeaf :: proc(node: BVHNode) -> bool {
	return node.triCount > 0
}
fminf :: proc(first, second: fl3) -> fl3 {
	return {min(first.x, second.x), min(first.y, second.y), min(first.z, second.z)}
}
fmaxf :: proc(first, second: fl3) -> fl3 {
	return {max(first.x, second.x), max(first.y, second.y), max(first.z, second.z)}
}

buildTestTriangles :: proc() -> []^Shape {
	input := make([]^Shape, N)
	rand.reset(12345678910)
	rf := rand.float32_uniform
	for &t, i in input {
		triangle := Tri{}
		r0 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r1 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r2 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		triangle.vertex0 = r0 * 9 - fl3{5, 5, 5}
		triangle.vertex1 = triangle.vertex0 + r1 * 2
		triangle.vertex2 = triangle.vertex0 + r2 * 2
		input[i] = new(Shape)
		input[i].aabb = _getTriangleAABB(triangle)
		input[i].type = triangle
		input[i].centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
	}
	return input
}

buildTestTriangles2 :: proc() -> []^Shape {
	data, err := os.read_entire_file("assets/unity.tri", context.allocator)
	defer delete(data, context.allocator)
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	input := make([dynamic]^Shape)
	for line in strings.split_lines_iterator(&iterator) {
		vals: []string
		vals, err = strings.split(line, " ")
		for v, j in vals {
			pointList[j], _ = strconv.parse_f32(v)
		}
		triangle := Tri {
			{pointList[0], pointList[1], pointList[2]},
			{pointList[3], pointList[4], pointList[5]},
			{pointList[6], pointList[7], pointList[8]},
		}
		s := new(Shape)
		s.centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type = triangle
		append(&input, s)
	}
	return input[:]
}

_getTriangleAABB :: proc(leaf: Tri) -> AABB {
	bounds: AABB = {{-MAX_F32, -MAX_F32, -MAX_F32}, {MAX_F32, MAX_F32, MAX_F32}}
	bounds.lower = fminf(bounds.lower, leaf.vertex0)
	bounds.lower = fminf(bounds.lower, leaf.vertex1)
	bounds.lower = fminf(bounds.lower, leaf.vertex2)
	bounds.upper = fmaxf(bounds.upper, leaf.vertex0)
	bounds.upper = fmaxf(bounds.upper, leaf.vertex1)
	bounds.upper = fmaxf(bounds.upper, leaf.vertex2)
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
