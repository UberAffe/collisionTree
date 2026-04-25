package collisiontree

import "core:fmt"
import "core:math"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import os "core:os"
import "core:thread"
import time "core:time"
import rl "vendor:raylib"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
N :: 64
scanSize: uint

fl3 :: [3]f32
ui2 :: [2]uint

Tri :: struct {
	vertex0, vertex1, vertex2, centroid: fl3,
}

Ray :: struct {
	O, D: fl3,
	t:    f32,
}

BVHNode :: struct {
	aabbMin, aabbMax:    fl3, //3d coordinate
	leftFirst, triCount: uint, 
	//total size 32 bytes
}

ThreadContext :: struct {
	rays:                       [dynamic]Ray,
	searchTime:                 time.Duration,
	xStart, yStart, xLen, yLen: uint,
	Pixels:                     map[ui2]f32,
	bvhTree:                    []BVHNode,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

tri := [N]Tri{}
shapeIdx := [N]uint{}
bvhNode := [N * 2]BVHNode{} //N 64 byte pages for optimal loading
rootNodeIdx, nodesUsed: uint = 0, 2
num_CPU: int
remaining: uint
dyn_pool: mem.Dynamic_Pool
pool_allocator: mem.Allocator
runners: [dynamic]TaskRunner
contexts: [dynamic]ThreadContext


main :: proc() {
	mem.dynamic_pool_init(&dyn_pool)
	pool_allocator = mem.dynamic_pool_allocator(&dyn_pool)
	//defer mem.dynamic_pool_destroy(&dyn_pool)
	num_CPU = os.get_processor_core_count()
	remaining = 640
	scanSize = remaining / uint(num_CPU)
	runners = make([dynamic]TaskRunner, num_CPU, num_CPU)
	contexts = make([dynamic]ThreadContext, num_CPU, num_CPU)
	for &r in runners {
		a: mem.Arena
		mem.arena_init(&a, new([64]byte)[:])
		r.task = threadScan
		r.allocator = mem.arena_allocator(&a)
	}

	buildTestTriangles()

	bWatch := time.Stopwatch{}
	time.stopwatch_start(&bWatch)
	BuildBVH()
	time.stopwatch_stop(&bWatch)

	rl.InitWindow(640, 640, "test")
	defer rl.CloseWindow()
	searchTime: time.Duration
	camPos := fl3{0, 0, -18}
	p0 := fl3{-1, 1, -15}
	p1 := fl3{1, 1, -15}
	p2 := fl3{-1, -1, -15}
	pool: thread.Pool
	thread.pool_init(&pool, pool_allocator, num_CPU)
	defer thread.pool_destroy(&pool)
	thread.pool_start(&pool)
	for !rl.WindowShouldClose() {
		// defer mem.dynamic_pool_free_all(&dyn_pool)
		// fmt.println("start of frame")
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		yStart: uint = 0
		xStart: uint = 0
		searchTime = 0
		remaining = 640
		for runner, i in runners {
			// tc := &contexts[i]
			
			contexts[i].xStart = xStart
			contexts[i].yStart = yStart
			contexts[i].yLen = 640
			contexts[i].xLen = math.min(scanSize, remaining)
			contexts[i].rays = make([dynamic]Ray,contexts[i].xLen*contexts[i].yLen)
			for &ray, j in contexts[i].rays{
				y:= uint(j)/contexts[i].xLen
				x:= uint(j)%contexts[i].xLen
				ray.O=camPos
				ray.D= la.normalize((p0 +
				(p1 - p0) * (f32(x + xStart) / 640) +
				(p2 - p0) * (f32(y + yStart) / 640))-ray.O)
				ray.t=MAX_F32
			}
			contexts[i].bvhTree = bvhNode[:]
			remaining -= contexts[i].xLen
			xStart += contexts[i].xLen
			contexts[i].Pixels = make(map[ui2]f32)
			thread.pool_add_task(&pool, runner.allocator, runner.task, rawptr(&contexts[i]), i)
		}
		// fmt.printfln("starting pool: %v", thread.pool_num_outstanding(&pool))

		for thread.pool_num_outstanding(&pool) > 0 {
			// fmt.printf("| %v", thread.pool_num_outstanding(&pool))
			searchTime += processThreadOutput(&pool)
		}
		searchTime += processThreadOutput(&pool)
		rl.DrawFPS(10, 10)
		rl.DrawText(
			fmt.ctprintf(
				"build time: %v\ncumulative search time: %v\naverage search time: %v\n409,600 rays across %v threads",
				time.stopwatch_duration(bWatch),
				searchTime,
				searchTime / time.Duration(num_CPU),
				num_CPU,
			),
			10,
			40,
			16,
			{0, 0, 0, 255},
		)
		rl.EndDrawing()
		// fmt.printfln("cumulative search time: %v", searchTime)
		// fmt.printfln("average search time: %v", searchTime / time.Duration(num_CPU))
	}
	thread.pool_shutdown(&pool)
	// fmt.printfln("Did we actually mean to close? %v", rl.WindowShouldClose())
}

processThreadOutput :: proc(pool: ^thread.Pool) -> time.Duration {
	task, ok := thread.pool_pop_done(pool)
	if ok {
		// fmt.printfln("drawing thread %v results", task.user_index)
		tc := cast(^ThreadContext)task.data
		for key, value in tc.Pixels {
			rl.DrawPixelV(
				{f32(tc.xStart + key.x), f32(tc.yStart + key.y)},
				{u8(value * value), 200, 255, 255},
			)
		}
		return tc.searchTime
	}
	return 0
}

threadScan :: proc(task: thread.Task) {
	// defer mem.free_all(task.allocator)
	tc := cast(^ThreadContext)task.data
	tc.searchTime = 0
	// fmt.printfln("bvh size: %v, start: (%v,%v), end: (%v,%v)",len(bvhNode),tc.xStart,tc.yStart,tc.xStart+tc.xLen-1,tc.yStart+tc.yLen-1)
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	for &ray,i in tc.rays {
		b, t := intersectBVH(&ray, rootNodeIdx)
		if ray.t < MAX_F32 do tc.Pixels[{uint(i)%tc.xLen, uint(i)/tc.xLen}] = ray.t
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
	// fmt.printfln("pixel len: %v, in thread: %v", len(tc.Pixels), task.user_index)
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
intersectBVH :: proc(ray: ^Ray, nodeIdx: uint) -> (uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	if !IntersectAABB(ray^, bvhNode[nodeIdx].aabbMin, bvhNode[nodeIdx].aabbMax) do return bvhIterations, triIterations
	if bvhNode[nodeIdx].triCount > 0 {
		for i in 0 ..< bvhNode[nodeIdx].triCount {
			intersectTri(ray, tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]])
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

IntersectAABB :: proc(ray: Ray, bmin, bmax: fl3) -> bool {
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
	tmin = max(tmin, min(tx1, tx2))
	tmax = min(tmax, max(tx1, tx2))
	return tmax >= tmin && tmin < ray.t && tmax > 0
}

intersectTri :: proc(ray: ^Ray, tri: Tri) {
	edge1 := tri.vertex1 - tri.vertex0
	edge2 := tri.vertex2 - tri.vertex0
	h := la.cross(ray.D, edge2)
	a := la.dot(edge1, h)
	if a > -0.0001 && a < 0.0001 do return
	f := 1 / a
	s := ray.O - tri.vertex0
	u := f * la.dot(s, h)
	if u < 0 || u > 1 do return
	q := la.cross(s, edge1)
	v := f * la.dot(ray.D, q)
	if v < 0 || u + v > 1 do return
	t := f * la.dot(edge2, q)
	if t > 0.0001 do ray.t = min(ray.t, t)
}

BuildBVH :: proc() {
	for &t, i in tri {
		t.centroid = (t.vertex0 + t.vertex1 + t.vertex2) / 3
		shapeIdx[i] = uint(i)
	}
	bvhNode[rootNodeIdx].triCount = N
	UpdateNodeBounds(rootNodeIdx)
	Subdivide(rootNodeIdx)
}

UpdateNodeBounds :: proc(nodeIdx: uint) {
	bvhNode[nodeIdx].aabbMin = {MAX_F32, MAX_F32, MAX_F32}
	bvhNode[nodeIdx].aabbMax = {-MAX_F32, -MAX_F32, -MAX_F32}
	for i in 0 ..< bvhNode[nodeIdx].triCount {
		UpdateTriangleAABB(&bvhNode[nodeIdx], tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]])
	}
}

UpdateTriangleAABB :: proc(node: ^BVHNode, leaf: Tri) {
	node.aabbMin = fminf(node.aabbMin, leaf.vertex0)
	node.aabbMin = fminf(node.aabbMin, leaf.vertex1)
	node.aabbMin = fminf(node.aabbMin, leaf.vertex2)
	node.aabbMax = fmaxf(node.aabbMax, leaf.vertex0)
	node.aabbMax = fmaxf(node.aabbMax, leaf.vertex1)
	node.aabbMax = fmaxf(node.aabbMax, leaf.vertex2)
}

Subdivide :: proc(nodeIdx: uint) {
	if bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	extent := bvhNode[nodeIdx].aabbMax - bvhNode[nodeIdx].aabbMin
	axis := 0
	if extent.y > extent.x do axis = 1
	if extent.z > extent[axis] do axis = 2
	splitPos := bvhNode[nodeIdx].aabbMin[axis] + extent[axis] * .5
	//in place partition
	i := int(bvhNode[nodeIdx].leftFirst)
	j := i + int(bvhNode[nodeIdx].triCount) - 1
	for i <= j {
		if tri[shapeIdx[i]].centroid[axis] < splitPos {
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

swap :: proc(first, second: ^uint) {
	t := first^
	first^ = second^
	second^ = t
}
fminf :: proc(first, second: fl3) -> fl3 {
	return {min(first.x, second.x), min(first.y, second.y), min(first.z, second.z)}
}
fmaxf :: proc(first, second: fl3) -> fl3 {
	return {max(first.x, second.x), max(first.y, second.y), max(first.z, second.z)}
}

buildTestTriangles :: proc() {
	rand.reset(12345678910)
	rf := rand.float32_uniform
	// Create N random triangles and populate the arrays
	// triIdx is a lookup table to avoid moving the data in the tri array.
	for &t, i in shapeIdx {
		triangle := Tri{}
		r0 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r1 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r2 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		triangle.vertex0 = r0 * 9 - fl3{5, 5, 5}
		triangle.vertex1 = triangle.vertex0 + r1 * 2
		triangle.vertex2 = triangle.vertex0 + r2 * 2
		tri[i] = triangle
	}
}
