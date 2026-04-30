package collisiontree

import "core:bufio"
import "core:fmt"
import "core:io"
import "core:math"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:thread"
import time "core:time"
import rl "vendor:raylib"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
N :: 12582
scanSize: uint

fl3 :: [3]f32
ui2 :: [2]uint

AABB :: struct {
	min, max: fl3,
}
AABB_Area :: proc(self: AABB) -> f32 {
	e := self.max - self.min
	return e.x * e.y + e.y * e.z + e.z * e.x
}
AABB_Grow :: proc(self: ^AABB, include: fl3) {
	self.min = fminf(self.min, include)
	self.max = fmaxf(self.max, include)
}

Tri :: struct {
	vertex0, vertex1, vertex2, centroid: fl3,
}

Ray :: struct {
	O, D: fl3,
	t:    f32,
}

BVHNode :: struct {
	aabb:                AABB, //3d coordinate
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

tri: [dynamic]Tri
shapeIdx: [dynamic]uint
bvhNode: [dynamic]BVHNode //N 64 byte pages for optimal loading
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
	camPos := fl3{-1.5, -.2, -2.5}
	p0 := fl3{-2.5, .8, -.5}
	p1 := fl3{-.5, .8, -.5}
	p2 := fl3{-2.5, -1.2, -.5}
	pool: thread.Pool
	thread.pool_init(&pool, pool_allocator, num_CPU)
	defer thread.pool_destroy(&pool)
	thread.pool_start(&pool)
	for !rl.WindowShouldClose() {
		rl.BeginDrawing()
		rl.ClearBackground(rl.BLACK)
		yStart: uint = 0
		xStart: uint = 0
		searchTime = 0
		remaining = 640
		for runner, i in runners {
			contexts[i].xStart = xStart
			contexts[i].yStart = yStart
			contexts[i].yLen = 640
			contexts[i].xLen = math.min(scanSize, remaining)
			contexts[i].rays = make([dynamic]Ray, contexts[i].xLen * contexts[i].yLen)
			for &ray, j in contexts[i].rays {
				y := uint(j) / contexts[i].xLen
				x := uint(j) % contexts[i].xLen
				ray.O = camPos
				ray.D = la.normalize(
					(p0 +
						(p1 - p0) * (f32(x + xStart) / 640) +
						(p2 - p0) * (f32(y + yStart) / 640)) -
					ray.O,
				)
				ray.t = MAX_F32
			}
			contexts[i].bvhTree = bvhNode[:]
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
				"build time: %v\ncumulative search time: %v\naverage search time: %v\n409,600 rays across %v threads",
				time.stopwatch_duration(bWatch),
				searchTime,
				searchTime / time.Duration(num_CPU),
				num_CPU,
			),
			10,
			40,
			16,
			{200, 200, 200, 255},
		)
		rl.EndDrawing()
	}
	thread.pool_shutdown(&pool)
}

processThreadOutput :: proc(pool: ^thread.Pool) -> time.Duration {
	task, ok := thread.pool_pop_done(pool)
	if ok {
		tc := cast(^ThreadContext)task.data
		for key, value in tc.Pixels {
			rl.DrawPixelV(
				{f32(tc.xStart + key.x), f32(tc.yStart + key.y)},
				{u8(500 - value * 55), u8(500 - value * 55), u8(500 - value * 55), 255},
			)
		}
		return tc.searchTime
	}
	return 0
}

threadScan :: proc(task: thread.Task) {
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
	if !Ray_IntersectAABB(ray^, bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations
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

Ray_IntersectAABB :: proc(ray: Ray, self: AABB) -> bool {
	tx1 := (self.min.x - ray.O.x) / ray.D.x
	tx2 := (self.max.x - ray.O.x) / ray.D.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (self.min.y - ray.O.y) / ray.D.y
	ty2 := (self.max.y - ray.O.y) / ray.D.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (self.min.z - ray.O.z) / ray.D.z
	tz2 := (self.max.z - ray.O.z) / ray.D.z
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
	bvhNode[nodeIdx].aabb.min = {MAX_F32, MAX_F32, MAX_F32}
	bvhNode[nodeIdx].aabb.max = {-MAX_F32, -MAX_F32, -MAX_F32}
	for i in 0 ..< bvhNode[nodeIdx].triCount {
		UpdateTriangleAABB(&bvhNode[nodeIdx], tri[shapeIdx[bvhNode[nodeIdx].leftFirst + i]])
	}
}

UpdateTriangleAABB :: proc(node: ^BVHNode, leaf: Tri) {
	AABB_Grow(&node.aabb, leaf.vertex0)
	AABB_Grow(&node.aabb, leaf.vertex1)
	AABB_Grow(&node.aabb, leaf.vertex2)
}

Subdivide :: proc(nodeIdx: uint) {
	axis := -1
	splitPos, bestCost: f32 = 0, MAX_F32
	for i in 0..<bvhNode[nodeIdx].triCount {
		for a in 0 ..< 3 {
			t:=tri[shapeIdx[bvhNode[nodeIdx].leftFirst+i]]
			candidatePos:=t.centroid[a]
			cost:=evaluateSAH(&bvhNode[nodeIdx],a,candidatePos)
			if cost<bestCost{
				splitPos=candidatePos
				axis=a
				bestCost=cost
			}
		}
	}
	fmt.printfln("the best cost for node %v is %v",nodeIdx,bestCost)
	if bestCost>=(AABB_Area(bvhNode[nodeIdx].aabb)*f32(bvhNode[nodeIdx].triCount)) do return
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

evaluateSAH :: proc(node: ^BVHNode, axis: int, pos: f32) -> f32 {
	lbox, rbox: AABB
	lcount, rcount: f32 = 0, 0
	for i in 0 ..< node.triCount {
		triangle := tri[shapeIdx[node.leftFirst + i]]
		if triangle.centroid[axis] < pos {
			lcount += 1
			AABB_Grow(&lbox, triangle.vertex0)
			AABB_Grow(&lbox, triangle.vertex1)
			AABB_Grow(&lbox, triangle.vertex2)
		} else {
			rcount += 1
			AABB_Grow(&rbox, triangle.vertex0)
			AABB_Grow(&rbox, triangle.vertex1)
			AABB_Grow(&rbox, triangle.vertex2)
		}
	}
	cost := lcount * AABB_Area(lbox) + rcount * AABB_Area(rbox)
	fmt.printfln("evaluate count: %v, with cost: %v",node.triCount,cost)
	return cost > 0 ? cost : MAX_F32
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
	data, err := os.read_entire_file("assets/unity.tri", context.allocator)
	// assert(err==nil)
	defer delete(data, context.allocator)
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	tri = make([dynamic]Tri)
	shapeIdx = make([dynamic]uint)
	i := 0
	for line in strings.split_lines_iterator(&iterator) {
		vals: []string
		vals, err = strings.split(line, " ")
		for v, j in vals {
			pointList[j], _ = strconv.parse_f32(v)
		}
		append(
			&tri,
			Tri {
				{pointList[0], pointList[1], pointList[2]},
				{pointList[3], pointList[4], pointList[5]},
				{pointList[6], pointList[7], pointList[8]},
				{},
			},
		)
		append(&shapeIdx, uint(i))
		i += 1
	}
	bvhNode = make([dynamic]BVHNode, i * i, i * i, context.allocator)
}
