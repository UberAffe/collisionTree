package collisiontree

import "core:log"
import "core:math"
import la "core:math/linalg"

_blas_Intersect::proc(tlas:TLAS,bidx:uint,originalRay:^Ray)->(u32,u32,int){
	blas := tlas.blas[bidx]
	bvh := tlas.bvhList[blas.bvhIndex]
	ray := originalRay^
	ray.O = _transformPosition(ray.O, blas.invTransform)
	ray.D = _transformVector(ray.D, blas.invTransform)
	ray.rD = 1 / ray.D
	when LOGGING {
		log.logf(log.Level(10), "Intersecting %v in the tree", originalRay^)
		log.logf(log.Level(10), "current node is %v", bvh.rootNodeIdx)
	}
	b,t,s:=Intersect(bvh, &ray)
	originalRay.t = ray.t
	return b,t,s
}

// Currently this just updates ray.t, the distance to first impact, eventually it will be updated to return the index of the closest object
_bvh_Intersect :: proc(bvh: BVH, ray: ^Ray) -> (u32, u32, int) {
	when PROFILING {profileStart()}
	bvhIterations := u32(0)
	triIterations := u32(0)
	sID := -1
	node := bvh.bvhNode[bvh.rootNodeIdx]
	idStack := [dynamic; 64]u32{}
	for {
		when PROFILING {profileStart("Node scan")}
		if (node.triCount > 0) {
			when PROFILING {profileStart("Leaf node")}
			prevT: f32
			for i in 0 ..< node.triCount {
				prevT = ray.t
				curID := int(bvh.shapeIdx[node.leftFirst + i])
				when LOGGING {log.logf(log.Level(10), "checking triangle %v", curID)}
				_shape_Intersect(bvh.tri[curID], ray)
				if ray.t < prevT {
					when LOGGING {log.logf(log.Level(10), "updating t")}
					sID = curID
				}
			}
			triIterations += node.triCount
			if len(idStack) == 0 do break
			id := pop(&idStack)
			when LOGGING {log.logf(log.Level(10), "id %v is the new node", id)}
			node = bvh.bvhNode[id]
			continue
		}
		when PROFILING {profileStart("Branch node")}
		id := node.leftFirst
		id2 := id + 1
		dist1 := _aabb_Intersect(ray^, bvh.bvhNode[id].aabb)
		dist2 := _aabb_Intersect(ray^, bvh.bvhNode[id2].aabb)
		bvhIterations += 2
		if dist1 > dist2 {
			_swap(&dist1, &dist2)
			_swap(&id, &id2)
		}
		when LOGGING {log.logf(log.Level(10), "id %v is the current node", id)}
		if dist1 == MAX {
			if len(idStack) == 0 do break
			id = pop(&idStack)
			node = bvh.bvhNode[id]
		} else {
			node = bvh.bvhNode[id]
			if dist2 != MAX {
				when LOGGING {log.logf(log.Level(10), "id %v added to the stack", id + 1)}
				append(&idStack, id2)
			}
		}
	}
	
	return bvhIterations, triIterations, sID
}

_aabb_Intersect :: proc(ray: Ray, b: AABB) -> f32 {
	bmin := b.lower
	bmax := b.upper
	//find the t where the ray crosses the lower and upper bounds of x
	tx1 := (bmin.x - ray.O.x) * ray.rD.x
	tx2 := (bmax.x - ray.O.x) * ray.rD.x
	//order the t value for the x crossings
	tmin := min(tx1, tx2)
	tmax := max(tx1, tx2)
	//find the t where the ray crosses the lower and upper bounds of y
	ty1 := (bmin.y - ray.O.y) * ray.rD.y
	ty2 := (bmax.y - ray.O.y) * ray.rD.y
	//order the t value for the y crossings and take the highest min, and lowest max
	tmin = max(tmin, min(ty1, ty2))
	tmax = min(tmax, max(ty1, ty2))
	//find the t where the ray crosses the lower and upper bounds of z
	tz1 := (bmin.z - ray.O.z) * ray.rD.z
	tz2 := (bmax.z - ray.O.z) * ray.rD.z
	//order the t value for the z crossings and take the highest min, and the lowest max
	tmin = max(tmin, min(tz1, tz2))
	tmax = min(tmax, max(tz1, tz2))
	when LOGGING {log.logf(
			log.Level(10),
			"aabb Check with tmin %v, tmax %v, ray.t %v",
			tmin,
			tmax,
			ray.t,
		)}
	//if tmax is >= tmin then there is an intersection somewhere on the line
	//if the min is greater than or equal to ray.t then that intersection is after the endpoint for this ray
	//if the max is less than zero then that intersection is before the origin point for this ray
	return (tmax >= tmin && tmin < ray.t && tmax > 0) ? tmin : MAX
}

_shape_Intersect :: proc(shape: Shape, ray: ^Ray) {
	switch type in shape.type {
	case Tri:
		_tri_Intersect(type, ray)
	case Square:
		_square_Intersect(type, ray)
	}
}

_tri_Intersect :: proc(triangle: Tri, ray: ^Ray) {
	edge1 := triangle.vertex[1] - triangle.vertex[0]
	edge2 := triangle.vertex[2] - triangle.vertex[0]
	ray_cross_e2 := la.cross(ray.D, edge2)
	det := la.dot(edge1, ray_cross_e2)
	//effectively parallel to triangle?
	if math.abs(det) < math.F32_EPSILON do return
	inv_det := 1 / det
	s := ray.O - triangle.vertex[0]
	u := inv_det * la.dot(s, ray_cross_e2)
	//ray passes outside edge2's bounds?
	if u < -math.F32_EPSILON || u - 1 > math.F32_EPSILON do return
	s_cross_e1 := la.cross(s, edge1)
	v := inv_det * la.dot(ray.D, s_cross_e1)
	//ray passes outside edge1's bounds?
	if v < -math.F32_EPSILON || u + v - 1 > math.F32_EPSILON do return
	t := inv_det * la.dot(edge2, s_cross_e1)
	//if true this is a ray intersect within bounds, otherwise there is an intersect on the line outside the ray bounds
	if t > 0 do ray.t = min(ray.t, t)
	when LOGGING {log.logf(
			log.Level(1),
			"det: %v, inv_det: %v, u: %v, v: %v, t: %v",
			det,
			inv_det,
			u,
			v,
			t,
		)}
}

_square_Intersect :: proc(square: AABB, ray: ^Ray) {

}
