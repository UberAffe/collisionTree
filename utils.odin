package collisiontree

import "core:fmt"
import "core:log"
import la "core:math/linalg"
import "core:mem"
import "core:thread"
import "core:time"
import "core:math"

MAX :: math.INF_F32
MIN :: -MAX

ui2 :: [2]u32
fl3 :: [3]f32

AABB :: struct {
	upper: fl3 `json:"upper"`,
	lower: fl3 `json:"lower"`,
}

DEFAULTAABB :: AABB{{MIN, MIN, MIN}, {MAX, MAX, MAX}}

BVHNode :: struct #align (32) {
	using aabb: AABB `json:"aabb"`, 
	leftFirst:  u32 `json:"leftFirst"`,
	triCount:   u32 `json:"triCount"`,
	//total size 32 bytes
}

LeftRight :: struct #align (8) {
	left:  i32,
	right: i32,
}
isLeaf :: proc(lr: LeftRight) -> bool {return transmute(uint)lr == 0}

TLASNode :: struct {
	using aabb:      AABB,
	using leftRight: LeftRight,
	blasIdx:         uint,
}

Shape :: struct {
	aabb:     AABB,
	centroid: fl3,
	type:     ShapeType,
}

Tri :: struct {
	vertex: [3]fl3,
}

Square::AABB

ShapeType :: union {
	Tri,
	Square,
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
	shapeID: u32,
	dist:    f32,
}

ThreadContext :: struct {
	offset:  u32,
	colTree: ^BLAS,
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

BVH :: struct {
	bvhNode:     [dynamic]BVHNode `json:"bvhNode"`,
	tri:         []Shape `json:"-"`,
	shapeIdx:    []u32 `json:"shapeIdx"`,
	rootNodeIdx: u32 `json:"rootNodeIdx"`,
	splitChecks: u32 `json:"splitChecks"`,
	longestOnly: bool `json:"longestOnly"`,
}

BLAS :: struct #align (64) {
	bvhIndex:     uint,
	invTransform: matrix[4, 4]f32,
	bounds:       AABB, //world space
}

TLAS :: struct {
	tlasNode: [dynamic]TLASNode,
	bvhList:  [dynamic]^BVH,
	blas:     [dynamic]BLAS,
}

_GrowAABB :: proc {
	_growAABBWithBox,
	_growAABBWithNode,
	_growAABBWithChildren,
	_growAABBWithPoint,
}

_growAABBWithBox :: proc(node: ^AABB, leaf: AABB) {
	when PROFILING {profileStart()}
	node.lower = _fminf(node.lower, leaf.lower)
	node.upper = _fmaxf(node.upper, leaf.upper)
}

_growAABBWithNode :: proc(node: ^BVHNode, leaf: AABB) {
	when PROFILING {profileStart()}
	node.aabb.lower = _fminf(node.aabb.lower, leaf.lower)
	node.aabb.upper = _fmaxf(node.aabb.upper, leaf.upper)
}

_growAABBWithChildren :: proc(node: ^BVHNode, ct: ^BVH) {
	when PROFILING {profileStart()}
	node.aabb.lower = _fminf(
		ct.bvhNode[node.leftFirst].aabb.lower,
		ct.bvhNode[node.leftFirst + 1].aabb.lower,
	)
	node.aabb.upper = _fmaxf(
		ct.bvhNode[node.leftFirst].aabb.upper,
		ct.bvhNode[node.leftFirst + 1].aabb.upper,
	)
}

_growAABBWithPoint :: proc(bounds: ^AABB, point: fl3) {
	when PROFILING {profileStart()}
	bounds.lower = _fminf(bounds.lower, point)
	bounds.upper = _fmaxf(bounds.upper, point)
}

_calculateNodeCost :: proc(node: BVHNode) -> f32 {
	when PROFILING {profileStart()}
	extent := node.aabb.upper - node.aabb.lower
	return max(f32(node.triCount),1) * (extent.x * extent.y + extent.y * extent.z + extent.z * extent.x)
}

_calculateSurfaceArea :: proc(aabb: AABB) -> f32 {
	when PROFILING {profileStart()}
	extent := aabb.upper - aabb.lower
	return extent.x * extent.y + extent.y * extent.z + extent.z * extent.x
}

_swap :: proc(first, second: ^$T) {
	when PROFILING {profileStart()}
	t := first^
	first^ = second^
	second^ = t
}

_getTriangleAABB :: proc(leaf: Tri) -> AABB {
	when PROFILING {profileStart()}
	bounds: AABB = {{MIN, MIN, MIN}, {MAX, MAX, MAX}}
	for v, i in leaf.vertex {
		bounds.lower = _fminf(bounds.lower, v)
		bounds.upper = _fmaxf(bounds.upper, v)
	}
	return bounds
}

//translates and rotates
_transformPosition :: proc(pos: fl3, m: matrix[4, 4]f32) -> fl3 {return(
		la.matrix_mul_vector(m, [4]f32{pos.x, pos.y, pos.z, 1}).xyz \
	)}
//only translates
_transformVector :: proc(pos: fl3, m: matrix[4, 4]f32) -> fl3 {return(
		la.matrix_mul_vector(m, [4]f32{pos.x, pos.y, pos.z, 0}).xyz \
	)}

_areaAABB :: proc(aabb: AABB) -> f32 {
	when PROFILING {profileStart()}
	e := aabb.upper - aabb.lower
	return e.x * e.y + e.y * e.z + e.z * e.x
}

_fminf :: proc(first, second: fl3) -> fl3 {
	return {min(first.x, second.x), min(first.y, second.y), min(first.z, second.z)}
}
_fmaxf :: proc(first, second: fl3) -> fl3 {
	return {max(first.x, second.x), max(first.y, second.y), max(first.z, second.z)}
}

_balancedBVH :: proc(bvh: BVH) {
	balance :: struct {
		avgInbalance: f32,
		maxInbalance: f32,
		balance: string,
		lB, rB:  int,
		lL, rL:  int,
		lT, rT:  u32,
		maxT:    u32,
	}
	dir :: enum {
		left,
		right,
		first,
	}
	idStack := make([dynamic]u32)
	dirStack := make([dynamic]dir)
	defer{
		delete(idStack)
		delete(dirStack)
	}
	depth := 0
	left, right := 0, 0
	append(&idStack, bvh.rootNodeIdx)
	append(&dirStack, dir.first)
	l:=bvh.bvhNode[bvh.rootNodeIdx].leftFirst
	balanceList:= make([dynamic]f32)
	append(&balanceList,_calculateNodeCost(bvh.bvhNode[l])-_calculateNodeCost(bvh.bvhNode[l+1]))
	for {
		breadth := len(idStack)
		b: balance
		for bal in balanceList{
			b.avgInbalance+=bal
			b.maxInbalance=abs(b.maxInbalance)<abs(bal)? bal: b.maxInbalance
		}
		b.avgInbalance/=f32(len(balanceList))
		b.balance=fmt.aprint(balanceList[:],sep=", ")
		clear(&balanceList)
		defer delete(b.balance)
		#reverse for id, i in idStack {
			if i < breadth {
				if bvh.bvhNode[id].triCount > 0 { 	//this is a leaf node
					b.maxT = max(bvh.bvhNode[id].triCount, b.maxT)
					switch dirStack[i] {
					case .left:
						b.lL += 1
						b.lT += bvh.bvhNode[id].triCount
					case .right:
						b.rL += 1
						b.rT += bvh.bvhNode[id].triCount
					case .first:
					}
					// b.nodes -= 1
				} else { 	//this is a branch node and both children should be added to the stack
					switch dirStack[i] {
					case .left:
						b.lB += 1
					case .right:
						b.rB += 1
					case .first:
					}
					l=bvh.bvhNode[id].leftFirst
					append(&balanceList,_calculateNodeCost(bvh.bvhNode[l])-_calculateNodeCost(bvh.bvhNode[l+1]))
					append(&idStack, l, l + 1)
					append(&dirStack, dir.left, dir.right)
				}
				breadth -= 1
				unordered_remove(&idStack, i)
				unordered_remove(&dirStack, i)
			}
		}
		when LOGGING {log.logf(log.Level(30), "depth %v: %v", depth, b)}
		depth += 1
		if len(idStack) == 0 do break
	}
}
