package collisiontree

import "core:fmt"
import "core:math"
import "core:mem"
import "core:thread"
import "core:time"

MAX:: 1_000_000_000_000_000_000_000_000_000_000
// MAX :: math.INF_F32
MIN :: -MAX //math.F32_MAX

ui2 :: [2]u32
fl3 :: [3]f32

AABB :: struct {
	upper: fl3 `json:"upper"`,
	lower: fl3 `json:"lower"`,
}

DEFAULTAABB :: AABB{{MIN, MIN, MIN}, {MAX, MAX, MAX}}

BVHNode :: struct #align (32) {
	using aabb: AABB `json:"aabb"`, //3d bounds
	leftFirst:  u32 `json:"leftFirst"`,
	triCount:   u32 `json:"triCount"`,
	//total size 32 bytes
}

Shape :: struct {
	aabb:     AABB,
	centroid: fl3,
	type:     ShapeType,
}

Tri :: struct {
	vertex: [3]fl3,
}

ShapeType :: union {
	Tri,
}

Ray :: struct {
	O, D, rD: fl3,
	t:        f32,
}

Bin :: struct {
	bounds:   AABB,
	triCount: u32,
}

Hit :: struct {
	rayID:   u32,
	shapeID: int,
	dist:    f32,
}

ThreadContext :: struct {
	offset:  u32,
	colTree: ^CollisionTree,
	rays:    []Ray,
	rc:      ^BatchResponse,
}

BatchResponse :: struct {
	searchTime:   time.Duration,
	batchID:      u32,
	boundsChecks: u32,
	shapeChecks:  u32,
	hits:         [dynamic]Hit,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

CollisionTree :: struct #align (64) {
	bvhNode:     [dynamic]BVHNode `json:"bvhNode"`,
	tri:         []Shape `json:"-"`,
	shapeIdx:    []u32 `json:"shapeIdx"`,
	rootNodeIdx: u32 `json:"rootNodeIdx"`,
	nodesUsed:   u32 `json:"nodesUsed"`,
	splitChecks: u32 `json:"splitChecks"`,
	longestOnly: bool `json:"longestOnly"`,
}

_GrowAABB :: proc {
	_growAABBWithBox,
	_growAABBWithNode,
	_growAABBWithChildren,
}

_growAABBWithBox :: proc(node: ^AABB, leaf: AABB) {
	node.lower = _fminf(node.lower, leaf.lower)
	node.upper = _fmaxf(node.upper, leaf.upper)
}

_growAABBWithNode :: proc(node: ^BVHNode, leaf: AABB) {
	node.aabb.lower = _fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = _fmaxf(node.aabb.upper, leaf.upper)
}

_growAABBWithChildren :: proc(node: ^BVHNode, ct: ^CollisionTree) {
	node.aabb.lower = _fminf(
		ct.bvhNode[node.leftFirst].aabb.lower,
		ct.bvhNode[node.leftFirst + 1].aabb.lower,
	)
	node.aabb.upper = _fmaxf(
		ct.bvhNode[node.leftFirst].aabb.upper,
		ct.bvhNode[node.leftFirst + 1].aabb.upper,
	)
}

_calculateNodeCost :: proc(node: BVHNode) -> f32 {
	extent := node.aabb.upper - node.aabb.lower
	return f32(node.triCount) * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}

_swap :: proc(first, second: ^$T) {
	if first == nil do deref()
	if second == nil do deref()
	t := first^
	first^ = second^
	second^ = t
}

_getTriangleAABB :: proc(leaf: Tri) -> AABB {
	bounds: AABB = {{MIN, MIN, MIN}, {MAX, MAX, MAX}}
	for v, i in leaf.vertex {
		bounds.lower = _fminf(bounds.lower, v)
		bounds.upper = _fmaxf(bounds.upper, v)
	}
	return bounds
}

deref :: proc(loc := #caller_location) {fmt.println("deref at:", loc)}
