package collisiontree

import "core:fmt"
import "core:math"

_bvh_Create :: proc(shapes: []Shape, alloc := context.allocator, loc := #caller_location) -> BVH {
	bvh: BVH
	bvh.tri = shapes
	length := len(bvh.tri)
	bvh.bvhNode = make([dynamic]BVHNode, 0, int(.6 * f32(length)), allocator = alloc, loc = loc)
	bvh.shapeIdx = make([]u32, length, allocator = alloc, loc = loc)
	return bvh
}

_bvh_Destroy :: proc(bvh: BVH) {
	delete(bvh.shapeIdx)
	delete(bvh.bvhNode)
}

_bvh_Build :: proc(
	colTree: ^BVH,
	divisionChecks: u32 = 16,
	longestOnly: bool = false,
	alloc := context.allocator,
	loc := #caller_location,
) {
	when PROFILING {profileStart()}
	for &t, i in colTree.tri {
		colTree.shapeIdx[i] = u32(i)
	}
	append(&colTree.bvhNode, BVHNode{{}, 0, u32(len(colTree.tri))})
	colTree.longestOnly = longestOnly
	colTree.splitChecks = max(divisionChecks, 2)
	_UpdateNodeBounds(colTree, colTree.rootNodeIdx)
	_Subdivide(colTree, colTree.rootNodeIdx, 0)
}

_UpdateNodeBounds :: proc(colTree: ^BVH, nodeIdx: u32, loc := #caller_location) {
	when PROFILING {profileStart()}
	colTree.bvhNode[nodeIdx].aabb = DEFAULTAABB
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&colTree.bvhNode[nodeIdx], s.aabb)
	}
}

_Subdivide :: proc(colTree: ^BVH, nodeIdx: u32, depth: int) {
	when PROFILING {profileStart(fmt.tprint("Subdivide", depth))}
	if colTree.bvhNode[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(colTree.bvhNode[nodeIdx])
	cBound := DEFAULTAABB
	for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
		s := colTree.tri[colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i]]
		_GrowAABB(&cBound, s.centroid)
	}
	bestAxis, bestPos, bestCost := _findBestSplitPlane(colTree, nodeIdx, cBound)
	//if no improvement, do not split
	if bestCost >= parentCost do return
	//in place partition, at worst this loops over every spot, but it should average to about half or less
	i := int(colTree.bvhNode[nodeIdx].leftFirst)
	j := i + int(colTree.bvhNode[nodeIdx].triCount) - 1
	scale := (f32(colTree.splitChecks) / (cBound.upper[bestAxis] - cBound.lower[bestAxis]))
	split := cBound.lower[bestAxis]
	for i <= j {
		when PROFILING {profileStart("sorting step")}
		binIdx := math.min(
			u32((colTree.tri[colTree.shapeIdx[i]].centroid[bestAxis] - split) * scale),
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
	if leftCount == 0 || leftCount >= colTree.bvhNode[nodeIdx].triCount do return
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
	_Subdivide(colTree, leftChildIdx, depth + 1)
	_Subdivide(colTree, leftChildIdx + 1, depth + 1)
}

_findBestSplitPlane :: proc(colTree: ^BVH, nodeIdx: u32, bound: AABB) -> (int, u32, f32) {
	when PROFILING {profileStart()}
	node := &colTree.bvhNode[nodeIdx]
	bestAxis := -1
	bestPos: u32 = 0
	bestCost: f32 = MAX
	bestBalance: f32 = MAX
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
		//find the best split based on the gathered data
		for i in 0 ..< colTree.splitChecks - 1 {
			planeCost := leftcount[i] * leftArea[i] + rightcount[i] * rightArea[i]
			balance := math.abs(leftcount[i] * leftArea[i] - rightcount[i] * rightArea[i])
			if planeCost < bestCost && balance < bestBalance {
				bestAxis = axis
				bestPos = i + 1
				bestCost = planeCost
				bestBalance = balance
			}
		}
	}
	return bestAxis, bestPos, bestCost
}

_evaluateSAH :: proc(colTree: ^BVH, node: ^BVHNode, axis: int, pos: f32) -> f32 {
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
