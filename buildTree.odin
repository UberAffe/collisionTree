package collisiontree

import "core:fmt"
import "core:log"
import "core:math"
import la "core:math/linalg"
import "core:strings"
import "core:time"

BuildBVH :: proc(
	inputTri: []Shape,
	divisionChecks: u32 = 16,
	longestOnly: bool = false,
	loc := #caller_location,
) -> (
	BLAS,
	f64,
) {
	// buildTimer:=time.Stopwatch{}
	// time.stopwatch_start(&buildTimer)
	// defer{
	// 	log.logf(log.Level(17),"Build Time: %v",time.stopwatch_duration(buildTimer))
	// }
	when PROFILING {profileStart()}
	colTree := BLAS{}
	colTree.rootNodeIdx = 0
	// colTree.nodesUsed = 1
	colTree.tri = inputTri
	length := len(colTree.tri)
	colTree.bvhNode = make([dynamic]BVHNode, 0, int(.6 * f32(length)), loc = loc)
	colTree.shapeIdx = make([]u32, length, loc = loc)
	for &t, i in colTree.tri {
		colTree.shapeIdx[i] = u32(i)
	}
	append(&colTree.bvhNode, BVHNode{{}, 0, u32(length)})
	// append(&colTree.bvhNode,BVHNode{})
	// colTree.bvhNode[colTree.rootNodeIdx].triCount = u32(length)
	colTree.longestOnly = longestOnly
	colTree.splitChecks = math.max(divisionChecks, 2)
	_UpdateNodeBounds(&colTree, colTree.rootNodeIdx)
	_Subdivide(&colTree, colTree.rootNodeIdx)
	return colTree, calculateBuildCost(colTree)
}

SetTransform :: proc(colTree: ^CollisionTree, transform: matrix[4, 4]f32) {
	when PROFILING {profileStart()}
	colTree.invTransform = la.matrix4_inverse(transform)
	bmax, bmin := bList[colTree.blasIndex].bvhNode[0].upper, bList[colTree.blasIndex].bvhNode[0].lower
	colTree.bounds = DEFAULTAABB
	for i in 0 ..< 8 {
		_GrowAABB(
			&colTree.bounds,
			_transformPosition(
				fl3 {
					i & 1 > 0 ? bmax.x : bmin.x,
					i & 2 > 0 ? bmax.y : bmin.y,
					i & 4 > 0 ? bmax.z : bmin.z,
				},
				transform,
			),
		)
	}
}

_UpdateNodeBounds :: proc(colTree: ^BLAS, nodeIdx: u32, loc := #caller_location) {
	colTree.bvhNode[nodeIdx].aabb = DEFAULTAABB
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&colTree.bvhNode[nodeIdx], s.aabb)
	}
}

_Subdivide :: proc(colTree: ^BLAS, nodeIdx: u32) {
	when PROFILING {profileStart()}
	if colTree.bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(colTree.bvhNode[nodeIdx])
	cBound := DEFAULTAABB
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&cBound, s.centroid)
	}
	bestAxis, bestPos, bestCost := #force_no_inline _findBestSplitPlane(colTree, nodeIdx, cBound)
	//if no improvement, do not split
	if bestCost >= parentCost do return
	//in place partition, at worst this loops over every spot, but it should average to about half or less
	i := int(colTree.bvhNode[nodeIdx].leftFirst)
	j := i + int(colTree.bvhNode[nodeIdx].triCount) - 1
	scale := (f32(colTree.splitChecks) / (cBound.upper[bestAxis] - cBound.lower[bestAxis]))
	for i <= j {
		binIdx := math.min(
			u32(
				(colTree.tri[colTree.shapeIdx[i]].centroid[bestAxis] - cBound.lower[bestAxis]) *
				scale,
			),
			colTree.splitChecks - 1,
		)
		if binIdx < bestPos {
			i += 1
		} else {
			_swap(&colTree.shapeIdx[i], &colTree.shapeIdx[j])
			j -= 1
		}
	}
	//abort split if one side empty, but the triangles will still have been shuffled
	leftCount := u32(i) - colTree.bvhNode[nodeIdx].leftFirst
	if leftCount == 0 || leftCount == colTree.bvhNode[nodeIdx].triCount do return
	// create child nodes
	leftChildIdx := u32(len(colTree.bvhNode))
	// colTree.nodesUsed += 2
	append(
		&colTree.bvhNode,
		BVHNode{{}, colTree.bvhNode[nodeIdx].leftFirst, leftCount},
		BVHNode{{}, u32(i), colTree.bvhNode[nodeIdx].triCount - leftCount},
	)
	colTree.bvhNode[nodeIdx].leftFirst = leftChildIdx
	colTree.bvhNode[nodeIdx].triCount = 0
	_UpdateNodeBounds(colTree, leftChildIdx)
	_UpdateNodeBounds(colTree, leftChildIdx + 1)
	_Subdivide(colTree, leftChildIdx)
	_Subdivide(colTree, leftChildIdx + 1)
}

calculateBuildCost :: proc(ct: BLAS) -> f64 {
	cost: f64 = 0
	for n in ct.bvhNode {
		if n.triCount == 0 do continue
		cost += f64(_calculateNodeCost(n))
	}
	return cost
}

_findBestSplitPlane :: proc(
	colTree: ^BLAS,
	nodeIdx: u32,
	bound: AABB,
) -> (
	int,
	u32,
	f32,
) {
	when PROFILING {profileStart()}
	node := &colTree.bvhNode[nodeIdx]
	bestAxis := -1
	bestPos: u32 = 0
	bestCost: f32 = MAX
	onlyAxis := 0
	if colTree.longestOnly {
		range := bound.upper - bound.lower
		onlyAxis = max(range[0], range[1]) > range[0] ? 1 : 0
		onlyAxis = max(range[onlyAxis], range[2]) > range[onlyAxis] ? 2 : onlyAxis
	}
	for axis in 0 ..< 3 {
		if colTree.longestOnly && axis != onlyAxis do continue
		when PROFILING {profileStart(fmt.tprint(axis))}
		boundMin := bound.lower[axis]
		boundMax := bound.upper[axis]
		//not flat
		if boundMin >= boundMax do continue
		scale := (f32(colTree.splitChecks) / (boundMax - boundMin))
		bins := make([dynamic]Bin, colTree.splitChecks)
		defer delete(bins)
		//count triangles that fall into each bin and grow the bounds to match
		for i in 0 ..< node.triCount {
			s := colTree.tri[colTree.shapeIdx[node.leftFirst + i]]
			binIdx := math.min(u32((s.centroid[axis] - boundMin) * scale), colTree.splitChecks - 1)
			assert(binIdx >= 0)
			bins[binIdx].triCount += 1
			_GrowAABB(&bins[binIdx].bounds, s.aabb)
		}
		leftArea := make([dynamic]f32, colTree.splitChecks)
		rightArea := make([dynamic]f32, colTree.splitChecks)
		leftcount := make([dynamic]f32, colTree.splitChecks)
		rightcount := make([dynamic]f32, colTree.splitChecks)
		defer delete(leftArea)
		defer delete(rightArea)
		defer delete(leftcount)
		defer delete(rightcount)
		leftBox, rightBox: AABB = {}, {}
		lSum, rSum: f32
		//gather data for each potential split plane
		//filling from the left and the right at the same time
		for i in 0 ..< colTree.splitChecks - 1 {
			lSum += f32(bins[i].triCount)
			leftcount[i] = lSum
			_GrowAABB(&leftBox, bins[i].bounds)
			leftArea[i] = _areaAABB(leftBox)
			rSum += f32(bins[colTree.splitChecks - 1 - i].triCount)
			rightcount[colTree.splitChecks - 1 - i] = rSum
			_GrowAABB(&rightBox, bins[colTree.splitChecks - 1 - i].bounds)
			rightArea[colTree.splitChecks - 1 - i] = _areaAABB(rightBox)
		}
		// scale = (boundMax - boundMin) / f32(colTree.splitChecks)
		//find the best split based on the gathered data
		for i in 0 ..< colTree.splitChecks - 1 {
			planeCost := leftcount[i] * leftArea[i] + rightcount[i] * rightArea[i]
			if planeCost < bestCost {
				bestAxis = axis
				bestPos = i + 1//boundMin  + scale * f32(i + 1)
				bestCost = planeCost
			}
		}
	}
	return bestAxis, bestPos, bestCost
}

_evaluateSAH :: proc(colTree: ^BLAS, node: ^BVHNode, axis: int, pos: f32) -> f32 {
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
