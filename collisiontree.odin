package collisiontree

import "core:fmt"
import "core:math"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import rl "vendor:raylib"

MAX_F32 :: 1_000_000_000_000_000_000_000_000_000_000
N :: 64

fl3 :: [3]f32

Tri :: struct {
	vertex0, vertex1, vertex2, centroid: fl3,
}

Ray :: struct {
	O, D: fl3,
	t:    f32,
}

tri := [N]Tri{}
triIdx := [N]uint{}
bvhNode := [N * 2 - 1]BVHNode{}
rootNodeIdx, nodesUsed: uint = 0, 1


main :: proc() {
	track: mem.Tracking_Allocator
	mem.tracking_allocator_init(&track, context.allocator)
	defer mem.tracking_allocator_destroy(&track)
	context.allocator = mem.tracking_allocator(&track)

	rf := rand.float32_uniform
	// Create N random triangles and populate the arrays
	// triIdx is a lookup table to avoid moving the data in the tri array.
	for &t, i in triIdx {
		triangle := Tri{}
		r0 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r1 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		r2 := fl3{rf(0, 1), rf(0, 1), rf(0, 1)}
		triangle.vertex0 = r0 * 9 - fl3{5, 5, 5}
		triangle.vertex1 = triangle.vertex0 + r1 * 2
		triangle.vertex2 = triangle.vertex0 + r2 * 2
		tri[i] = triangle
		t = uint(i)
	}
	rl.InitWindow(640, 640, "test")
	camPos := fl3{0, 0, -18}
	p0 := fl3{-1, 1, -15}
	p1 := fl3{1, 1, -15}
	p2 := fl3{-1, -1, -15}
	BuildBVH()
	defer rl.CloseWindow()
	ray := Ray{}
	pixelPos: fl3
	for !rl.WindowShouldClose() {
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		ray.O = camPos
		for y in 0 ..< 640 {
			for x in 0 ..< 640 {
				pixelPos = p0 + (p1 - p0) * (f32(x) / 640) + (p2 - p0) * (f32(y) / 640)
				ray.D = la.normalize(pixelPos - ray.O)
				ray.t = MAX_F32
				intersectBVH(&ray, rootNodeIdx)
				if ray.t > 0 && ray.t < MAX_F32 {
					// not a fast function, but with a stable data set, it should give a consistent impact on FPS, not a scaling one.
					rl.DrawPixelV(
						{pixelPos.x * 320 + 320, pixelPos.y * 320 + 320},
						{u8(ray.t * ray.t), 200, 255, 255},
					)
				}
			}
		}
		rl.DrawFPS(10, 10)
		rl.EndDrawing()
		// I'm not seeing any memory leak, but something is still causing it to slow down over time ...
		for _, leak in track.allocation_map {
			fmt.printf("%v leaked %m\n", leak.location, leak.size)
		}
	}

}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
intersectBVH :: proc(ray: ^Ray, nodeIdx: uint) {
	if !IntersectAABB(ray^, bvhNode[nodeIdx].aabbMin, bvhNode[nodeIdx].aabbMax) do return
	if bvhNode[nodeIdx].triCount > 0 {
		for i in 0 ..< bvhNode[nodeIdx].triCount {
			intersectTri(ray, tri[triIdx[bvhNode[nodeIdx].firstTriIdx + i]])
		}
	} else {
		intersectBVH(ray, bvhNode[nodeIdx].leftNode)
		intersectBVH(ray, bvhNode[nodeIdx].leftNode + 1)
	}

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
	for &t in tri {
		t.centroid = (t.vertex0 + t.vertex1 + t.vertex2) / 3
	}
	bvhNode[rootNodeIdx].triCount = N
	UpdateNodeBounds(rootNodeIdx)
	Subdivide(rootNodeIdx)
}

UpdateNodeBounds :: proc(nodeIdx: uint) {
	bvhNode[nodeIdx].aabbMin = {MAX_F32, MAX_F32, MAX_F32}
	bvhNode[nodeIdx].aabbMax = {-MAX_F32, -MAX_F32, -MAX_F32}
	first := bvhNode[nodeIdx].firstTriIdx
	for i in 0 ..< bvhNode[nodeIdx].triCount {
		UpdateTriangleAABB(&bvhNode[nodeIdx],tri[triIdx[first + i]])
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
	i := int(bvhNode[nodeIdx].firstTriIdx)
	j := i + int(bvhNode[nodeIdx].triCount) - 1
	for i <= j {
		if tri[triIdx[i]].centroid[axis] < splitPos {
			i += 1
		} else {
			swap(&triIdx[i], &triIdx[j])
			j -= 1
		}
	}
	//abort split if one side empty
	leftCount := uint(i) - bvhNode[nodeIdx].firstTriIdx
	if leftCount == 0 || leftCount == bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := nodesUsed
	nodesUsed += 1
	rightChildIdx := nodesUsed
	nodesUsed += 1
	bvhNode[nodeIdx].leftNode = leftChildIdx
	bvhNode[leftChildIdx].firstTriIdx = bvhNode[nodeIdx].firstTriIdx
	bvhNode[leftChildIdx].triCount = leftCount
	bvhNode[rightChildIdx].firstTriIdx = uint(i)
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

BVHNode :: struct {
	aabbMin, aabbMax:                fl3,
	leftNode, firstTriIdx, triCount: uint,
}
