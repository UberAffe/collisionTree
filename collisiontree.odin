package collisiontree

import "core:fmt"
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
import "base:runtime"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
N :: 64
scanSize: uint

fl3 :: [3]f32
ui2 :: [2]uint

AABB :: struct #align(4){
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

BVHNode :: struct #align(4){
	aabb:                AABB, //3d bounds
	leftFirst, triCount: uint,
	//total size 32 bytes
}

ThreadContext :: struct {
	xStart, yStart, xLen, yLen: uint,
	searchTime:                 time.Duration,
	rays:                       []Ray,
	Pixels:                     map[ui2]f32,
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


main :: proc() {
	fmt.println("Collision Test Started")
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
		mem.arena_init(&a, new([128]byte)[:])
		r.task = threadScan
		r.allocator = mem.arena_allocator(&a)
	}
	fmt.println("Bulding Test Triangles")
	inputTri := buildTestTriangles()
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
	thread.pool_init(&pool, pool_allocator, num_CPU)
	defer thread.pool_destroy(&pool)
	thread.pool_start(&pool)
	for !rl.WindowShouldClose() {
		// defer mem.dynamic_pool_free_all(&dyn_pool)
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		yStart: uint = 0
		xStart: uint = 0
		searchTime = 0
		remaining = 640
		for runner, i in runners {
			contexts[i].xStart = xStart
			contexts[i].yStart = yStart
			contexts[i].yLen = 640
			contexts[i].xLen = math.min(scanSize, remaining)
			contexts[i].rays = make([]Ray, contexts[i].xLen * contexts[i].yLen)
			for &ray, j in contexts[i].rays {
				y := uint(j) / contexts[i].xLen
				x := uint(j) % contexts[i].xLen
				ray.O = camPos
				ray.D = la.normalize(
					(p0 +
						(p1 - p0) * (f32(x + xStart) / 640) +
						(p2 - p0) * (f32(y + yStart) / 640)
					) - ray.O,
				)
				ray.t = MAX_F32
			}
			remaining -= contexts[i].xLen
			xStart += contexts[i].xLen
			contexts[i].Pixels = make(map[ui2]f32)
			thread.pool_add_task(&pool, runner.allocator, runner.task, rawptr(&contexts[i]), i)
		}
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
	thread.pool_shutdown(&pool)
}

beginRayIntersections::proc(rays:..Ray){
	//build tasks
	//start tasks
}

beginShapeIntersections::proc(shapes:..Shape){
	//build tasks
}

allIntersectionsComplete::proc()->bool{return thread.pool_num_outstanding(&pool)==0}

processThreadOutput :: proc(pool: ^thread.Pool) -> time.Duration {
	task, ok := thread.pool_pop_done(pool)
	if ok {
		// fmt.printfln("drawing thread %v results", task.user_index)
		tc := cast(^ThreadContext)task.data
		for key, value in tc.Pixels {
			v := u8(500 - value * 55)
			rl.DrawPixelV({f32(tc.xStart + key.x), f32(tc.yStart + key.y)}, {v, v, v, 255})
		}
		return tc.searchTime
	}
	return 0
}

threadScan :: proc(task: thread.Task) {
	// defer mem.free_all(task.allocator)
	tc := cast(^ThreadContext)task.data
	tc.searchTime = 0
	sw := time.Stopwatch{}
	time.stopwatch_start(&sw)
	for &ray, i in tc.rays {
		b, t := intersectBVH(&ray, rootNodeIdx)
		if ray.t < MAX_F32 do tc.Pixels[{uint(i) % tc.xLen, uint(i) / tc.xLen}] = ray.t
	}
	time.stopwatch_stop(&sw)
	tc.searchTime = time.stopwatch_duration(sw)
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
intersectBVH :: proc(ray: ^Ray, nodeIdx: uint) -> (uint, uint) {
	bvhIterations := uint(1)
	triIterations := uint(0)
	if !IntersectAABB(ray^, bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations
	if bvhNode[nodeIdx].triCount > 0 {
		for i in 0 ..< bvhNode[nodeIdx].triCount {
			s := tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]]
			switch type in s.type {
			case Tri:
				_intersectTri(type, ray)
			}
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

IntersectAABB :: proc(ray: Ray, b: AABB) -> bool {
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
	tmin = max(tmin, min(tx1, tx2))
	tmax = min(tmax, max(tx1, tx2))
	return tmax >= tmin && tmin < ray.t && tmax > 0
}

BuildBVH :: proc(inputTri: []^Shape) {
	tri = inputTri
	length := len(tri)
	shapeIdx = make([]uint, length)
	bvhNode = runtime.make_aligned([]BVHNode, 2 * length,64)
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

GrowAABB :: proc(node: ^BVHNode, leaf: AABB) {
	node.aabb.lower = fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = fmaxf(node.aabb.upper, leaf.upper)
}

Subdivide :: proc(nodeIdx: uint) {
	fmt.printfln("Division: %v, Count: %v", nodeIdx, bvhNode[nodeIdx].triCount)
	if bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	extent := bvhNode[nodeIdx].aabb.upper - bvhNode[nodeIdx].aabb.lower
	axis := 0
	if extent.y > extent.x do axis = 1
	if extent.z > extent[axis] do axis = 2
	splitPos := bvhNode[nodeIdx].aabb.lower[axis] + extent[axis] * .5
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
		input[i].aabb=_getTriangleAABB(triangle)
		input[i].type=triangle
		input[i].centroid=(triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
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
		s:= new(Shape)
		s.centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type=triangle
		append(&input, s)
	}
	return input[:]
}

_getTriangleAABB :: proc(leaf: Tri) -> AABB {
	bounds: AABB = {{MAX_F32, MAX_F32, MAX_F32}, {-MAX_F32, -MAX_F32, -MAX_F32}}
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
