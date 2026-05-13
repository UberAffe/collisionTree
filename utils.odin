package collisiontree

import "core:thread"
import "core:mem"
import "core:time"

ui2 :: [2]u32
fl3 :: [3]f32

AABB :: struct {
	upper: fl3 `json:"upper"`,
	lower: fl3 `json:"lower"`,
}

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
	searchTime: time.Duration,
	colTree:    ^CollisionTree,
	rays:       []Ray,
	hit:        [dynamic]Hit,
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

_swap :: proc(first, second: ^$T) {
	t := first^
	first^ = second^
	second^ = t
}