package collisiontree

import "core:math"
_intersectBVH :: proc {
	_intersectBVHRecursive,
	_intersectBVHLoop,
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the first object
_intersectBVHRecursive :: proc(
	colTree: ^CollisionTree,
	ray: ^Ray,
	nodeIdx: u32,
) -> (
	u32,
	u32,
	int,
) {
	bvhIterations := u32(1)
	triIterations := u32(0)
	if !_intersectAABBBool(ray^, colTree.bvhNode[nodeIdx].aabb) do return bvhIterations, triIterations, 0
	sID := -1
	if colTree.bvhNode[nodeIdx].triCount > 0 {
		prevT := ray.t
		for i in 0 ..< colTree.bvhNode[nodeIdx].triCount {
			testSID := int(colTree.shapeIdx[colTree.bvhNode[nodeIdx].leftFirst + i])
			assert(testSID >= 0)
			_intersectShape(colTree.tri[testSID], ray)
			if ray.t < prevT {
				prevT = ray.t
				sID = testSID
			}
			triIterations += 1
		}
	} else {
		b, t: u32
		b, t, sID = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst)
		prevT := ray.t
		testSID: int
		bvhIterations += b
		triIterations += t
		b, t, testSID = _intersectBVH(colTree, ray, colTree.bvhNode[nodeIdx].leftFirst + 1)
		if ray.t < prevT do sID = testSID
		bvhIterations += b
		triIterations += t
	}
	return bvhIterations, triIterations, sID
}
_intersectBVHLoop :: proc(colTree: ^CollisionTree, ray: ^Ray) -> (u32, u32, int) {
	bvhIterations := u32(1)
	triIterations := u32(0)
	node := &colTree.bvhNode[colTree.rootNodeIdx]
	idStack := make([dynamic]^BVHNode, 0, 64)
	sID := -1
	for {
		if (node.triCount > 0) {
			prevT: f32
			for i in 0 ..< node.triCount {
				prevT = ray.t
				curID := int(colTree.shapeIdx[node.leftFirst + i])
				assert(curID >= 0)
				_intersectShape(colTree.tri[curID], ray)
				if ray.t < prevT do sID = curID
			}
			triIterations += node.triCount
			if len(idStack) == 0 do break
			node = pop(&idStack)
			continue
		}
		child1 := &colTree.bvhNode[node.leftFirst]
		child2 := &colTree.bvhNode[node.leftFirst + 1]
		dist1 := _intersectAABBFloat(ray^, child1.aabb)
		dist2 := _intersectAABBFloat(ray^, child2.aabb)
		bvhIterations += 2
		if dist1 > dist2 {
			_swap(&dist1, &dist2)
			_swap(&child1, &child2)
		}
		if dist1 == math.F32_MAX {
			if len(idStack) == 0 do break
			node = pop(&idStack)
		} else {
			node = child1
			if dist2 != math.F32_MAX do append(&idStack, child2)
		}
	}
	return bvhIterations, triIterations, sID
}

_intersectShape :: proc(shape: Shape, ray: ^Ray) {
	switch type in shape.type {
	case Tri:
		_intersectTri(type, ray)
	}
}

_intersectAABBBool :: proc(ray: Ray, b: AABB) -> bool {
	bmin := b.lower
	bmax := b.upper
	tx1 := (bmin.x - ray.O.x) * ray.rD.x
	tx2 := (bmax.x - ray.O.x) * ray.rD.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) * ray.rD.y
	ty2 := (bmax.y - ray.O.y) * ray.rD.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) * ray.rD.z
	tz2 := (bmax.z - ray.O.z) * ray.rD.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return tmax >= tmin && tmin < ray.t && tmax > 0
}
_intersectAABBFloat :: proc(ray: Ray, b: AABB) -> f32 {
	bmin := b.lower
	bmax := b.upper
	tx1 := (bmin.x - ray.O.x) * ray.rD.x
	tx2 := (bmax.x - ray.O.x) * ray.rD.x
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	ty1 := (bmin.y - ray.O.y) * ray.rD.y
	ty2 := (bmax.y - ray.O.y) * ray.rD.y
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	tz1 := (bmin.z - ray.O.z) * ray.rD.z
	tz2 := (bmax.z - ray.O.z) * ray.rD.z
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin : math.F32_MAX
}