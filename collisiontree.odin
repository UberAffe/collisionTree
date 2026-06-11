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
	for n in ct.node {
		if n.triCount == 0 do continue
		cost += f64(_calculateNodeCost(n))
	}
	return cost
}

Refit :: proc(ct: ^BVH) -> f64 {
	cost: f64 = 0
	for i := int(len(ct.node) - 1); i >= 0; i -= 1 {
		if i == 1 do continue
		ct.node[i].aabb = DEFAULTAABB
		if ct.node[i].triCount > 0 {
			_UpdateNodeBounds(ct, u32(i))
			cost += f64(_calculateNodeCost(ct.node[i]))
			continue
		}
		_GrowAABB(&ct.node[i], ct)
	}
	return cost
}

Insert :: proc(bvh: ^BVH, updatedShapes: []Shape, startIDX: u32) {
	bvh.shape = updatedShapes
	toInsert := startIDX
	for toInsert < u32(len(bvh.shape)) {
		// grow the shapeIdx by the number of new triangles
		index:= u32(len(bvh.shapeIdx))
		append(&bvh.shapeIdx, toInsert)
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
		//I suppose we can put all of the existing shapes at the end of the shapeIdx
		//that will avoid breaking everything else, but it is going to cause the shapeidx to potentially grow quickly.
		//this process is going to leave a lot of useless duplicate spots that nothing else can use.
		for i in bvh.node[toCheck].leftChild..<bvh.node[toCheck].triCount{
			append(&bvh.shapeIdx,i)
		}	
		bvh.node[toCheck].leftChild=index
		bvh.node[toCheck].triCount+=1
		//on to the next addition
		toInsert += 1
	}
}

Remove :: proc() {}
