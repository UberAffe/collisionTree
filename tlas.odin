package collisiontree

import "core:fmt"
import "core:log"
import "core:sync"
Create_TLAS :: proc() -> TLAS {
	tlas: TLAS
	tlas.bvhList = make([dynamic]BVH)
	tlas.blas = make([dynamic]BLAS)
	tlas.tlasNode = make([dynamic]TLASNode)
	return tlas
}

Destroy_TLAS :: proc(tlas: TLAS) {
	delete(tlas.bvhList)
	delete(tlas.blas)
	delete(tlas.tlasNode)
}

Build_TLAS :: proc(tlas: ^TLAS) {
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
	B := FindBestMatch(nodeIdx[:], nodeIndices, A)
	for nodeIndices > 1 {
		C := FindBestMatch(nodeIdx[:], nodeIndices, B)
		// fmt.println(A,B,C)
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
			B = FindBestMatch(nodeIdx[:], nodeIndices, A)
			continue
		}
		A = B
		B = C
	}
	tlas.tlasNode[0] = tlas.tlasNode[nodeIdx[A]]

}

FindBestMatch :: proc(list: []i32, n, a: i32) -> i32 {
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

Intersect_TLAS :: proc(tlas: TLAS, ray: ^Ray) -> (u32, u32, int) {
	when PROFILING {profileStart()}
	node := tlas.tlasNode[0]
	idStack := [dynamic; 64]i32{}
	// append(&idStack, node.left)
	// i := 0
	tb, tt: u32 = 0, 0
	closest := -1
	for {
		log.logf(
			log.Level(40),
			"thread %v: %v, blas %v",
			sync.current_thread_id(),
			node.leftRight,
			node.blasIdx,
		)
		when PROFILING {profileStart("TLAS scan")}
		if isLeaf(node.leftRight) {
			when PROFILING {profileStart(fmt.tprint("TLAS leaf", node.blasIdx))}
			bIt, tIt, shapeID := _intersectBVH(tlas, tlas.blas[node.blasIdx].bvhIndex, ray)
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
		dist1 := _intersectAABBFloat(ray^, tlas.tlasNode[childIdx1].aabb)
		dist2 := _intersectAABBFloat(ray^, tlas.tlasNode[childIdx2].aabb)
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
