package collisiontree

import "core:fmt"

_tlas_Create :: proc(alloc:=context.allocator) -> TLAS {
	tlas: TLAS
	tlas.bvhList = make([dynamic]BVH,alloc)
	tlas.blas = make([dynamic]BLAS,alloc)
	tlas.tlasNode = make([dynamic]TLASNode,alloc)
	return tlas
}

_tlas_Destroy :: proc(tlas: TLAS) {
	delete(tlas.bvhList)
	delete(tlas.blas)
	delete(tlas.tlasNode)
}

_tlas_Build :: proc(tlas: ^TLAS) {
	//reserve index 0 for the root
	clear(&tlas.tlasNode)
	append(&tlas.tlasNode, TLASNode{})
	nodeIdx := make([dynamic]i32)
	defer delete(nodeIdx)
	nodeIndices := i32(len(tlas.blas))
	for i in 0 ..< len(tlas.blas) {
		//plus 1 to account for the reserved first node
		append(&nodeIdx, i32(i + 1))
		append(&tlas.tlasNode, TLASNode{tlas.blas[i].bounds, {}, uint(i)})
	}
	A: i32 = 0
	B := _findBestMatch(nodeIdx[:], nodeIndices, A)
	for nodeIndices > 1 {
		C := _findBestMatch(nodeIdx[:], nodeIndices, B)
		//if they are eachother's best match then pair them
		if A == C {
			bounds := DEFAULTAABB
			_GrowAABB(&bounds, tlas.tlasNode[nodeIdx[A]])
			_GrowAABB(&bounds, tlas.tlasNode[nodeIdx[B]])
			append(&tlas.tlasNode, TLASNode{bounds, LeftRight{nodeIdx[A], nodeIdx[B]}, {}})
			//index A now reference the last node
			nodeIdx[A] = i32(len(tlas.tlasNode) - 1)
			nodeIndices -= 1
			nodeIdx[B] = nodeIdx[nodeIndices]
			B = _findBestMatch(nodeIdx[:], nodeIndices, A)
			continue
		}
		A = B
		B = C
	}
	tlas.tlasNode[0] = tlas.tlasNode[nodeIdx[A]]

}

_findBestMatch :: proc(list: []i32, n, a: i32) -> i32 {
	smallest: f32 = MAX
	bestB: i32 = -1
	for b in i32(0) ..< n {
		if b != a {
			bounds := DEFAULTAABB
			_GrowAABB(&bounds, tlas.tlasNode[list[a]])
			_GrowAABB(&bounds, tlas.tlasNode[list[b]])
			sa := _calculateSurfaceArea(bounds)
			if sa < smallest {
				smallest = sa
				bestB = b
			}
		}
	}
	return bestB
}

_tlas_Intersect :: proc(tlas: TLAS, ray: ^Ray) -> (u32, u32, int) {
	when PROFILING {profileStart()}
	node := tlas.tlasNode[0]
	idStack := [dynamic; 64]i32{}
	tb, tt: u32 = 0, 0
	closest := -1
	for {
		when PROFILING {profileStart("TLAS scan")}
		if isLeaf(node.leftRight) {
			when PROFILING {profileStart(fmt.tprint("TLAS leaf", node.blasIdx))}
			bIt, tIt, shapeID := Intersect(tlas, node.blasIdx, ray)
			closest = shapeID
			tb += bIt
			tt += tIt
			if len(idStack) == 0 do break
			i := pop(&idStack)
			node = tlas.tlasNode[i]
			continue
		}
		childIdx1 := node.left
		childIdx2 := node.right
		tb += 2
		dist1 := _aabb_Intersect(ray^, tlas.tlasNode[childIdx1].aabb)
		dist2 := _aabb_Intersect(ray^, tlas.tlasNode[childIdx2].aabb)
		//ensure that the nearest child is child1
		if dist1 > dist2 {
			_swap(&dist1, &dist2)
			_swap(&childIdx1, &childIdx2)
		}
		// if the closest is max, then either pull from the stack or break
		if dist1 == MAX {
			if len(idStack) == 0 do break
			i := pop(&idStack)
			node = tlas.tlasNode[i]
		} else {
			// switch to the closest child
			node = tlas.tlasNode[childIdx1]
			// if the further child is valid, add it to the stack
			if dist2 != MAX {
				append(&idStack, childIdx2)
			}
		}
	}
	return tb, tt, closest
}
