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

CalculateBuildCost :: proc(ct: BVH) -> f64 {
	cost: f64 = 0
	for n in ct.bvhNode {
		if n.triCount == 0 do continue
		cost += f64(_calculateNodeCost(n))
	}
	return cost
}

RefitBVH :: proc(ct: ^BVH) -> f64 {
	cost: f64 = 0
	for i := int(len(ct.bvhNode) - 1); i >= 0; i -= 1 {
		if i == 1 do continue
		ct.bvhNode[i].aabb = DEFAULTAABB
		if ct.bvhNode[i].triCount > 0 {
			_UpdateNodeBounds(ct, u32(i))
			cost += f64(_calculateNodeCost(ct.bvhNode[i]))
			continue
		}
		_GrowAABB(&ct.bvhNode[i], ct)
	}
	return cost
}

Insert :: proc() {}

Remove :: proc() {}
