package collisiontree

import "core:fmt"
import "core:log"
import "core:math"
import "core:strings"

BuildBVH :: proc(
	inputTri: []Shape,
	divisionChecks: u32 = 16,
	longestOnly: bool = false,
	loc := #caller_location,
) -> (
	^CollisionTree,
	f64,
) {
	colTree := new(CollisionTree)
	colTree.rootNodeIdx = 0
	colTree.nodesUsed = 1
	colTree.tri = inputTri
	length := len(colTree.tri)
	colTree.bvhNode = make([dynamic]BVHNode, 0, int(.6 * f32(length)), loc = loc)
	colTree.shapeIdx = make([]u32, length, loc = loc)
	for &t, i in colTree.tri {
		colTree.shapeIdx[i] = u32(i)
	}
	append(&colTree.bvhNode,BVHNode{{},0,u32(length)})
	// append(&colTree.bvhNode,BVHNode{})
	// colTree.bvhNode[colTree.rootNodeIdx].triCount = u32(length)
	colTree.longestOnly = longestOnly
	colTree.splitChecks = math.max(divisionChecks, 2)
	_UpdateNodeBounds(colTree, colTree.rootNodeIdx)
	_Subdivide(colTree, colTree.rootNodeIdx)
	return colTree, calculateBuildCost(colTree)
}

_UpdateNodeBounds :: proc(colTree: ^CollisionTree, nodeIdx: u32, loc:=#caller_location) {
	colTree.bvhNode[nodeIdx].aabb = DEFAULTAABB
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&colTree.bvhNode[nodeIdx], s.aabb)
	}
}

_Subdivide :: proc(colTree: ^CollisionTree, nodeIdx: u32) {
	if colTree.bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(colTree.bvhNode[nodeIdx])
	bestAxis, bestPos, bestCost := _findBestSplitPlane(colTree, nodeIdx)
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
	leftCount := u32(i) - colTree.bvhNode[nodeIdx].leftFirst
	if leftCount == 0 || leftCount == colTree.bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := u32(len(colTree.bvhNode))
	colTree.nodesUsed += 2
	append(
		&colTree.bvhNode,
		BVHNode{{}, colTree.bvhNode[nodeIdx].leftFirst, leftCount},
		BVHNode{{}, u32(i), colTree.bvhNode[nodeIdx].triCount - leftCount},
	)
	colTree.bvhNode[nodeIdx].leftFirst = leftChildIdx
	colTree.bvhNode[nodeIdx].triCount = 0
	_UpdateNodeBounds(colTree, leftChildIdx)
	_UpdateNodeBounds(colTree, leftChildIdx+1)
	_Subdivide(colTree, leftChildIdx)
	_Subdivide(colTree, leftChildIdx+1)
}

calculateBuildCost :: proc(ct: ^CollisionTree) -> f64 {
	cost: f64 = 0
	for n in ct.bvhNode {
		if n.triCount == 0 do continue
		cost += f64(_calculateNodeCost(n))
	}
	return cost
}

_findBestSplitPlane :: proc(colTree: ^CollisionTree, nodeIdx: u32) -> (int, f32, f32) {
	node := &colTree.bvhNode[nodeIdx]
	bins := make([dynamic]Bin, colTree.splitChecks, colTree.splitChecks)
	defer delete(bins)
	bestAxis := -1
	bestPos, bestCost: f32 = 0, MAX
	for axis in 0 ..< 3 {
		boundMin: f32 = MAX
		boundMax: f32 = MIN
		for i in 0 ..< node.triCount {
			s := colTree.tri[colTree.shapeIdx[node.leftFirst + i]]
			boundMin = math.min(boundMin, s.centroid[axis])
			boundMax = math.max(boundMax, s.centroid[axis])
		}
		if boundMin >= boundMax do continue
		scale := f32(colTree.splitChecks) / (boundMax - boundMin)
		for i in 0 ..< node.triCount {
			s := colTree.tri[colTree.shapeIdx[node.leftFirst + i]]
			binIdx := int(
				math.min(f32(colTree.splitChecks) - 1, (s.centroid[axis] - boundMin) * scale),
			)
			assert(binIdx >= 0)
			bins[binIdx].triCount += 0
			_GrowAABB(&bins[binIdx].bounds, s.aabb)
		}
		leftArea := make([dynamic]f32, colTree.splitChecks, colTree.splitChecks)
		rightArea := make([dynamic]f32, colTree.splitChecks, colTree.splitChecks)
		leftcount := make([dynamic]f32, colTree.splitChecks, colTree.splitChecks)
		rightcount := make([dynamic]f32, colTree.splitChecks, colTree.splitChecks)
		defer delete(leftArea)
		defer delete(rightArea)
		defer delete(leftcount)
		defer delete(rightcount)
		leftBox, rightBox: AABB = {}, {}
		lSum, rSum: f32
		for i in 0 ..< colTree.splitChecks - 1 {
			lSum += f32(bins[i].triCount)
			leftcount[i] = lSum
			_GrowAABB(&leftBox, bins[i].bounds)
			leftArea[i] = _areaAABB(leftBox)
			rSum += f32(bins[colTree.splitChecks - 2 - i].triCount)
			rightcount[colTree.splitChecks - 2 - i] = rSum
			_GrowAABB(&rightBox, bins[colTree.splitChecks - 2 - i].bounds)
			rightArea[colTree.splitChecks - 2 - i] = _areaAABB(rightBox)
		}
		scale = (boundMax - boundMin) / f32(colTree.splitChecks)
		for i in 0 ..< colTree.splitChecks - 1 {
			planeCost := leftcount[i] * leftArea[i] + rightcount[i] * rightArea[i]
			if planeCost < bestCost {
				bestAxis = axis
				bestPos = boundMin + scale * f32((i + 1))
				bestCost = planeCost
			}
		}
	}
	return bestAxis, bestPos, bestCost
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
	return cost > 0 ? cost : MAX
}

_areaAABB :: proc(aabb: AABB) -> f32 {
	e := aabb.upper - aabb.lower
	return e.x * e.y + e.y * e.z + e.z * e.x
}

_fminf :: proc(first, second: fl3) -> fl3 {
	return {min(first.x, second.x), min(first.y, second.y), min(first.z, second.z)}
}
_fmaxf :: proc(first, second: fl3) -> fl3 {
	return {max(first.x, second.x), max(first.y, second.y), max(first.z, second.z)}
}
