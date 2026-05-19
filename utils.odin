package collisiontree

import "core:thread"
import "core:mem"
import "core:time"
import "core:math"
import "core:fmt"

// MAX:: 1_000_000_000_000_000_000_000_000_000_000
MAX:: math.F32_MAX
MIN:: -MAX//math.F32_MAX

ui2 :: [2]u32
fl3 :: [3]f32

AABB :: struct {
	upper: fl3 `json:"upper"`,
	lower: fl3 `json:"lower"`,
}

DEFAULTAABB:: AABB{{MIN, MIN, MIN}, {MAX, MAX, MAX}}

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
	offset:     u32,
	colTree:    ^CollisionTree,
	rays:       []Ray,
	rc:   ^ResponseContext,
}

ResponseContext :: struct{
	searchTime: time.Duration,
	hits: 		[dynamic]Hit,
}

TaskRunner :: struct {
	allocator: mem.Allocator,
	task:      proc(_: thread.Task),
}

CollisionTree :: struct #align (64) {
	bvhNode:     []BVHNode `json:"bvhNode"`,
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
	_growAABBWithChildren
}

_growAABBWithBox :: proc(node: ^AABB, leaf: AABB) {
	node.lower = _fminf(node.lower, leaf.lower)
	node.upper = _fmaxf(node.upper, leaf.upper)
}

_growAABBWithNode :: proc(node: ^BVHNode, leaf: AABB) {
	node.aabb.lower = _fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = _fmaxf(node.aabb.upper, leaf.upper)
}

_growAABBWithChildren::proc(node:^BVHNode, ct:^CollisionTree){
	node.aabb.lower = _fminf(ct.bvhNode[node.leftFirst].aabb.lower, ct.bvhNode[node.leftFirst+1].aabb.lower)
	node.aabb.upper = _fmaxf(ct.bvhNode[node.leftFirst].aabb.upper, ct.bvhNode[node.leftFirst+1].aabb.upper)
}

_swap :: proc(first, second: ^$T) {
	if first==nil do deref()
	if second==nil do deref()
	t := first^
	first^ = second^
	second^ = t
}

deref::proc(loc:=#caller_location){fmt.println("deref at:",loc)}