package collisiontree

import la "core:math/linalg"

PROFILING :: #config(ctprofiling, false)

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

calculateBuildCost :: proc(ct: BVH) -> f64 {
	cost: f64 = 0
	for n in ct.bvhNode {
		if n.triCount == 0 do continue
		cost += f64(_calculateNodeCost(n))
	}
	return cost
}

Create::proc{TLAS_Create,_bvh_Create}
Destroy::proc{TLAS_Destroy,_bvh_Destroy}
Intersect::proc{_intersect_TLAS,_intersectBVH,_intersectShape}
Build::proc{_bvh_Build,_tlas_Build}