package collisiontree

import "core:fmt"
import "core:math"

_bvh_Create :: proc(shapes: []Shape, alloc := context.allocator, loc := #caller_location) -> BVH {
	bvh: BVH
	bvh.shape = shapes
	length := len(bvh.shape)
	bvh.node = make([dynamic]BVHNode, 0, int(.6 * f32(length)), allocator = alloc, loc = loc)
	bvh.leafs = make([dynamic][dynamic]u32, allocator = alloc, loc = loc)
	return bvh
}

_bvh_Destroy :: proc(bvh: BVH) {
	for list in bvh.leafs{
		delete(list)
	}
	delete(bvh.leafs)
	delete(bvh.node)
}

_bvh_Build :: proc(
	colTree: ^BVH,
	divisionChecks: u32 = 16,
	longestOnly: bool = false,
	alloc := context.allocator,
	loc := #caller_location,
) {
	when PROFILING {profileStart()}
	append(&colTree.leafs,make([dynamic]u32,allocator=colTree.leafs.allocator))
	for &t, i in colTree.shape {
		append(&colTree.leafs[i],u32(i))
	}
	append(&colTree.node, BVHNode{})
	colTree.longestOnly = longestOnly
	colTree.splitChecks = max(divisionChecks, 2)
	_UpdateNodeBounds(colTree, colTree.rootNodeIdx)
	_Subdivide(colTree, colTree.rootNodeIdx, 0)
}

_UpdateNodeBounds :: proc(bvh: ^BVH, nodeIdx: u32, loc := #caller_location) {
	when PROFILING {profileStart()}
	bvh.node[nodeIdx].aabb = DEFAULTAABB
	for i in 0 ..< len(bvh.leafs[bvh.node[nodeIdx].leafID]) {
		s := bvh.shape[bvh.leafs[bvh.node[nodeIdx].leafID][i]]
		_GrowAABB(&bvh.node[nodeIdx], s.aabb)
	}
}

_Subdivide :: proc(bvh: ^BVH, nodeIdx: u32, depth: int) {
	when PROFILING {profileStart(fmt.tprint("Subdivide", depth))}
	leafID:= bvh.node[nodeIdx].leafID
	// if bvh.node[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(bvh^,bvh.node[nodeIdx])
	cBound := DEFAULTAABB
	for i in 0 ..< len(bvh.leafs[leafID]) {
		s := bvh.shape[bvh.leafs[leafID][i]]
		_GrowAABB(&cBound, s.centroid)
	}
	bestAxis, bestPos, bestCost := _findBestSplitPlane(bvh, nodeIdx, cBound)
	//if no improvement, do not split
	if bestCost >= parentCost do return
	//in place partition, at worst this loops over every spot, but it should average to about half or less
	leftCount := 0
	rightMinID := len(bvh.leafs[bvh.node[nodeIdx].leafID]) - 1
	scale := (f32(bvh.splitChecks) / (cBound.upper[bestAxis] - cBound.lower[bestAxis]))
	split := cBound.lower[bestAxis]
	for leftCount <= rightMinID {
		when PROFILING {profileStart("sorting step")}
		binIdx := math.min(
			u32((bvh.shape[bvh.leafs[leafID][leftCount]].centroid[bestAxis] - split) * scale),bvh.splitChecks - 1)
		
		if binIdx < bestPos {
			leftCount += 1
		} else {
			_swap(&bvh.leafs[leafID][leftCount], &bvh.leafs[leafID][rightMinID])
			rightMinID -= 1
		}
	}
	//abort split if one side empty, but the triangles will still have been shuffled
	if leftCount == 0 || leftCount >= len(bvh.leafs[leafID]) do return
	// create child nodes
	leftChildIdx := i32(len(bvh.node))
	leftLeafIdx := i32(len(bvh.leafs[leafID]))
	// colTree.nodesUsed += 2
	append(
		&bvh.node,
		BVHNode{{}, -1, leftLeafIdx},
		BVHNode{{}, -1, leftLeafIdx+1},
	)
	bvh.node[nodeIdx].leftChild = leftChildIdx
	_UpdateNodeBounds(bvh, leftChildIdx)
	_UpdateNodeBounds(bvh, leftChildIdx + 1)
	_Subdivide(bvh, leftChildIdx, depth + 1)
	_Subdivide(bvh, leftChildIdx + 1, depth + 1)
}

_findBestSplitPlane :: proc(colTree: ^BVH, nodeIdx: u32, bound: AABB) -> (int, u32, f32) {
	when PROFILING {profileStart()}
	node := &colTree.node[nodeIdx]
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
			s := colTree.shape[colTree.shapeIdx[node.leftChild + i]]
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
		shape := colTree.shape[colTree.shapeIdx[node.leftChild + i]]
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
