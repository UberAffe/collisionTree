package collisiontree

import "core:math"
import "core:log"
import la "core:math/linalg"
import "core:fmt"

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the closest object
_intersectBVH :: proc(tlas: TLAS,bidx:uint, originalRay: ^Ray) -> (u32, u32, int) {
	when PROFILING {profileStart()}
	colTree:=tlas.blas[bidx]
	blas:= tlas.bvhList[colTree.bvhIndex]
	log.logf(log.Level(10),"Intersecting %v in the tree",originalRay^)
	log.logf(log.Level(10),"current node is %v",blas.rootNodeIdx)
	bvhIterations := u32(0)
	triIterations := u32(0)
	node := blas.bvhNode[blas.rootNodeIdx]
	idStack := [dynamic;64]u32{}
	sID := -1
	ray := originalRay^
	ray.O=_transformPosition(ray.O,colTree.invTransform)
	ray.D=_transformVector(ray.D,colTree.invTransform)
	ray.rD=1/ray.D
	for {
		when PROFILING {profileStart("Node scan")}
		if (node.triCount > 0) {
			when PROFILING {profileStart("Leaf node")}
			prevT:f32
			for i in 0 ..< node.triCount {
				prevT = ray.t
				curID := int(blas.shapeIdx[node.leftFirst + i])
				log.logf(log.Level(10),"checking triangle %v",curID)
				_intersectTri(blas.tri[curID].type.(Tri), &ray)
				// _intersectShape(colTree.tri[curID], ray)
				if ray.t < prevT  {
					log.logf(log.Level(10),"updating t")
					sID = curID
				}
			}
			triIterations += node.triCount
			if len(idStack) == 0 do break
			id:= pop(&idStack)
			log.logf(log.Level(10),"id %v is the new node",id)
			node = blas.bvhNode[id]
			continue
		}
		when PROFILING {profileStart("Branch node")}
		id:= node.leftFirst
		id2:= id+1
		dist1 := _intersectAABBFloat(ray, blas.bvhNode[id].aabb)
		dist2 := _intersectAABBFloat(ray, blas.bvhNode[id2].aabb)
		bvhIterations += 2
		if dist1 > dist2 {
			_swap(&dist1, &dist2)
			_swap(&id,&id2)
		}
		log.logf(log.Level(10),"id %v is the current node",id)
		if dist1 == MAX {
			if len(idStack) == 0 do break
			id = pop(&idStack)
			node = blas.bvhNode[id]
		} else {
			node = blas.bvhNode[id]
			if dist2 != MAX {
				log.logf(log.Level(10),"id %v added to the stack",id+1)
				append(&idStack, id2)
			}
		}
	}
	originalRay.t=ray.t
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
	log.logf(log.Level(10),"aabb Check with tmin %v, tmax %v, ray.t %v",tmin,tmax,ray.t)
	return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin : MAX
}

//https://docs.google.com/spreadsheets/d/1pIyi3a3e-rDaneXjOIMA3eP6gcS93uybYXGKPdxv_oE/edit?usp=sharing
//implemented in gsheet to inspect the values more directly
_intersectTri :: proc(triangle: Tri, ray: ^Ray) {
	edge1 := triangle.vertex[1] - triangle.vertex[0]
	edge2 := triangle.vertex[2] - triangle.vertex[0]
	ray_cross_e2 := la.cross(ray.D, edge2)
	det := la.dot(edge1, ray_cross_e2)
	if math.abs(det)< math.F32_EPSILON do return //effectively parallel to triangle
	inv_det := 1 / det
	s := ray.O - triangle.vertex[0]
	u := inv_det * la.dot(s, ray_cross_e2)
	if u < -math.F32_EPSILON || u-1 > math.F32_EPSILON do return //ray passes outside edge2's bounds
	s_cross_e1 := la.cross(s, edge1)
	v := inv_det * la.dot(ray.D, s_cross_e1)
	if v < -math.F32_EPSILON || u + v - 1> math.F32_EPSILON do return //ray passes outside edge1's bounds
	t := inv_det * la.dot(edge2, s_cross_e1)
	if t > math.F32_EPSILON do ray.t = min(ray.t, t) //if true this is a ray intersect, if false, there is an intersect on the line outside the ray bounds
	log.logf(log.Level(1), "det: %v, inv_det: %v, u: %v, v: %v, t: %v",det,inv_det,u,v,t)
}
