package collisiontree

import "core:fmt"
import "core:math"

_bvh_Create :: proc(shapes: []Shape, alloc := context.allocator, loc := #caller_location) -> BVH {
	bvh: BVH
	bvh.shape = shapes
	length := len(bvh.shape)
	bvh.node = make_soa(#soa[dynamic]BVHNode, 0, int(.6 * f32(length)), allocator = alloc, loc = loc)
	return bvh
}

_bvh_Destroy :: proc(bvh: BVH) {
	delete(bvh.node)
}

_bvh_Build :: proc(
	bvh: ^BVH,
	divisionChecks: u32 = 16,
	longestOnly: bool = false,
	alloc := context.allocator,
	loc := #caller_location,
) {
	when PROFILING {profileStart()}
	append(&bvh.node, BVHNode{{},0,make([dynamic]u32)})
	for &t, i in bvh.shape {
		append(&bvh.node[0].shapeIDs,u32(i))
	}
	bvh.longestOnly = longestOnly
	bvh.splitChecks = max(divisionChecks, 2)
	_UpdateNodeBounds(bvh, bvh.rootNodeIdx)
	_Subdivide(bvh, bvh.rootNodeIdx, 0)
}

_UpdateNodeBounds :: proc(bvh: ^BVH, nodeIdx: u32, loc := #caller_location) {
	when PROFILING {profileStart()}
	bvh.node[nodeIdx].aabb = DEFAULTAABB
	for shapeID in bvh.node[nodeIdx].shapeIDs {
		s := bvh.shape[shapeID]
		_GrowAABB(&bvh.node[nodeIdx], s.aabb)
	}
}

_Subdivide :: proc(bvh: ^BVH, nodeIdx: u32, depth: int) {
	when PROFILING {profileStart(fmt.tprint("Subdivide", depth))}
	// if bvh.node[nodeIdx].triCount <= 2 do return
	//determine split axis and position
	parentCost := _calculateNodeCost(bvh.node[nodeIdx])
	cBound := DEFAULTAABB
	for shapeID in bvh.node[nodeIdx].shapeIDs {
		s := bvh.shape[shapeID]
		_GrowAABB(&cBound, s.centroid)
	}
	bestAxis, bestPos, bestCost := _findBestSplitPlane(bvh, nodeIdx, cBound)
	//if no improvement, do not split
	if bestCost >= parentCost do return
	//in place partition, at worst this loops over every spot, but it should average to about half or less
	leftCount := 0
	rightMinID := len(bvh.node[nodeIdx].shapeIDs) - 1
	scale := (f32(bvh.splitChecks) / (cBound.upper[bestAxis] - cBound.lower[bestAxis]))
	split := cBound.lower[bestAxis]
	for leftCount <= rightMinID {
		when PROFILING {profileStart("sorting step")}
		binIdx := math.min(
			u32((bvh.shape[bvh.node[nodeIdx].shapeIDs[leftCount]].centroid[bestAxis] - split) * scale),bvh.splitChecks - 1)
		
		if binIdx < bestPos {
			leftCount += 1
		} else {
			_swap(&bvh.node[nodeIdx].shapeIDs[leftCount], &bvh.node[nodeIdx].shapeIDs[rightMinID])
			rightMinID -= 1
		}
	}
	//abort split if one side empty, but the triangles will still have been shuffled
	if leftCount == 0 || leftCount >= len(bvh.node[nodeIdx].shapeIDs) do return
	// create child nodes
	leftChildIdx := u32(len(bvh.node))
	// colTree.nodesUsed += 2
	append(
		&bvh.node,
		BVHNode{{}, 0, make([dynamic]u32)},
		BVHNode{{}, 0, make([dynamic]u32)},
	)
	bvh.node[nodeIdx].leftChild = leftChildIdx
	_UpdateNodeBounds(bvh, leftChildIdx)
	_UpdateNodeBounds(bvh, leftChildIdx + 1)
	_Subdivide(bvh, leftChildIdx, depth + 1)
	_Subdivide(bvh, leftChildIdx + 1, depth + 1)
}

_findBestSplitPlane :: proc(bvh: ^BVH, nodeIdx: u32, bound: AABB) -> (int, u32, f32) {
	when PROFILING {profileStart()}
	node := &bvh.node[nodeIdx]
	bestAxis := -1
	bestPos: u32 = 0
	bestCost: f32 = MAX
	bestBalance: f32 = MAX
	onlyAxis := 0
	if bvh.longestOnly {
		range := bound.upper - bound.lower
		onlyAxis = max(range[0], range[1]) > range[0] ? 1 : 0
		onlyAxis = max(range[onlyAxis], range[2]) > range[onlyAxis] ? 2 : onlyAxis
	}
	for axis in 0 ..< 3 {
		if bvh.longestOnly && axis != onlyAxis do continue
		when PROFILING {profileStart(fmt.tprint(axis))}
		boundMin := bound.lower[axis]
		boundMax := bound.upper[axis]
		//not flat
		if boundMin >= boundMax do continue
		scale := (f32(bvh.splitChecks) / (boundMax - boundMin))
		bins := make([dynamic]Bin, bvh.splitChecks)
		defer delete(bins)
		//count triangles that fall into each bin and grow the bounds to match
		for shapeID in node.shapeIDs {
			s := bvh.shape[shapeID]
			binIdx := math.min(u32((s.centroid[axis] - boundMin) * scale), bvh.splitChecks - 1)
			assert(binIdx >= 0)
			bins[binIdx].triCount += 1
			_GrowAABB(&bins[binIdx].bounds, s.aabb)
		}
		leftArea := make([dynamic]f32, bvh.splitChecks)
		rightArea := make([dynamic]f32, bvh.splitChecks)
		leftcount := make([dynamic]f32, bvh.splitChecks)
		rightcount := make([dynamic]f32, bvh.splitChecks)
		defer delete(leftArea)
		defer delete(rightArea)
		defer delete(leftcount)
		defer delete(rightcount)
		leftBox, rightBox: AABB = {}, {}
		lSum, rSum: f32
		//gather data for each potential split plane
		//filling from the left and the right at the same time
		for i in 0 ..< bvh.splitChecks - 1 {
			lSum += f32(bins[i].triCount)
			leftcount[i] = lSum
			_GrowAABB(&leftBox, bins[i].bounds)
			leftArea[i] = _areaAABB(leftBox)
			rSum += f32(bins[bvh.splitChecks - 1 - i].triCount)
			rightcount[bvh.splitChecks - 1 - i] = rSum
			_GrowAABB(&rightBox, bins[bvh.splitChecks - 1 - i].bounds)
			rightArea[bvh.splitChecks - 1 - i] = _areaAABB(rightBox)
		}
		//find the best split based on the gathered data
		for i in 0 ..< bvh.splitChecks - 1 {
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

_evaluateSAH :: proc(bvh: BVH, node: BVHNode, axis: int, pos: f32) -> f32 {
	leftBox, rightBox: AABB
	leftCount, rightCount: f32 = 0, 0
	for shapeId in node.shapeIDs {
		shape := bvh.shape[shapeId]
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
