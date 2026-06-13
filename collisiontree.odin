package collisiontree

import la "core:math/linalg"

PROFILING :: #config(ctprofiling, false)

Create :: proc {
	_tlas_Create,
	_bvh_Create,
}
Destroy :: proc {
	_tlas_Destroy,
	_bvh_Destroy,
}
Intersect :: proc {
	_tlas_Intersect,
	_blas_Intersect,
	_bvh_Intersect,
	_aabb_Intersect,
	_shape_Intersect,
	_square_Intersect,
	_tri_Intersect,
}
Build :: proc {
	_tlas_Build,
	_bvh_Build,
}

//This updates the BLAS bounds to equal the world space equivalent of the bvh
SetTransform :: proc(blas: ^BLAS, bvhBounds: AABB, transform: matrix[4, 4]f32) {
	when PROFILING {profileStart()}
	blas.invTransform = la.matrix4_inverse(transform)
	blas.bounds = DEFAULTAABB
	for i in 0 ..< 8 {
		_GrowAABB(
			&blas.bounds,
			_transformPosition(
				fl3 {
					i & 1 > 0 ? bvhBounds.upper.x : bvhBounds.lower.x,
					i & 2 > 0 ? bvhBounds.upper.y : bvhBounds.lower.y,
					i & 4 > 0 ? bvhBounds.upper.z : bvhBounds.lower.z,
				},
				transform,
			),
		)
	}
}

CalculateBuildCost :: proc(bvh: BVH) -> f64 {
	cost: f64 = 0
	for node in bvh.node {
		if isLeaf(node) do continue
		cost += f64(_calculateNodeCost(node))
	}
	return cost
}

Refit :: proc(bvh: ^BVH) -> f64 {
	cost: f64 = 0
	for i := u32(len(bvh.node) - 1); i > 0; i -= 1 {
		if i == 1 do continue
		bvh.node[i].aabb = DEFAULTAABB
		if isLeaf(bvh.node[i]) {
			_UpdateNodeBounds(bvh, i)
			cost += f64(_calculateNodeCost(bvh.node[i]))
			continue
		}
		_GrowAABB(&bvh.node[i].aabb, bvh.node[bvh.node[i].leftChild])
		_GrowAABB(&bvh.node[i].aabb, bvh.node[bvh.node[i].leftChild+1])
	}
	return cost
}

Insert :: proc(bvh: ^BVH, updatedShapes: []Shape, startIDX: u32) {
	bvh.shape = updatedShapes
	toInsert := startIDX
	for toInsert < u32(len(bvh.shape)) {
		//DFS into the bvh to find the best leaf to add to
		toCheck := bvh.rootNodeIdx
		_GrowAABB(&bvh.node[toCheck].aabb,bvh.shape[toInsert].aabb)
		for !isLeaf(bvh.node[toCheck]) {
			c1 := bvh.node[toCheck].leftChild
			c2 := c1 + 1
			b1 := bvh.node[c1].aabb
			b2 := bvh.node[c2].aabb
			osa1 := _calculateSurfaceArea(b1)
			osa2 := _calculateSurfaceArea(b2)
			_GrowAABB(&b1, bvh.shape[toInsert].aabb)
			_GrowAABB(&b2, bvh.shape[toInsert].aabb)
			sa1 := _calculateSurfaceArea(b1)
			sa2 := _calculateSurfaceArea(b2)
			if sa1 / osa1 < sa2 / osa2 {
				//this means that c2 is a better match because the surface area grew by a smaller percentage
				_swap(&c1,&c2)
				_swap(&b1,&b2)
			}
			bvh.node[c1].aabb=b1
			toCheck=c1
		}
		//we have identified the leaf node that is the best fit as toCheck
		//now it's AABB needs to grow and the shape can be added.
		_GrowAABB(&bvh.node[toCheck].aabb,bvh.shape[toInsert].aabb)
		append(&bvh.node[toCheck].shapeIDs,toInsert)
		//on to the next addition
		toInsert += 1
	}
}

Remove :: proc() {}
