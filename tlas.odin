package collisiontree

Create_TLAS::proc()->TLAS{
    tlas:TLAS
    tlas.bvhList=make([dynamic]BVH)
    tlas.blas=make([dynamic]BLAS)
    tlas.tlasNode=make([dynamic]TLASNode)
    return tlas
}

Destroy_TLAS::proc(tlas:TLAS){
    delete(tlas.bvhList)
    delete(tlas.blas)
    delete(tlas.tlasNode)
}

Build_TLAS::proc(tlas:^TLAS){
    append(&tlas.tlasNode,TLASNode{DEFAULTAABB,2,0},TLASNode{},TLASNode{DEFAULTAABB,0,1},TLASNode{DEFAULTAABB,1,1})
}

Intersect_TLAS::proc(tlas:TLAS, ray:^Ray){
    node:= &tlas.tlasNode[0]
    idStack:= [dynamic;64]uint{}
    append(&idStack, node.leftBLAS)
    i:=0
    breadth:=len(idStack)
    for{
        if node.isLeaf>0{
            _intersectBVH(tlas,tlas.blas[node.leftBLAS].bvhIndex,ray)
            if len(idStack)==0 do break
            node= &tlas.tlasNode[idStack[i]]
            unordered_remove(&idStack,i)
            i+=1
            continue
        }
        childIdx1:= node.leftBLAS
        childIdx2:= childIdx1+1
        dist1:= _intersectAABBFloat(ray^,tlas.tlasNode[childIdx1])
        dist2:= _intersectAABBFloat(ray^,tlas.tlasNode[childIdx2])
        //put the closest as child1
        if dist1>dist2{
            _swap(&dist1,&dist2)
            _swap(&childIdx1,&childIdx2)
        }
        // if the closest is max, then either pull from the stack or break
        if dist1==MAX{
            if len(idStack)==0 do break
            node= &tlas.tlasNode[idStack[i]]
            unordered_remove(&idStack,i)
            i+=1
        }else{
            // switch to the closest child
            node= &tlas.tlasNode[childIdx1]
            // if the further child is valid, add it to the stack
            if dist2 != MAX{
                append(&idStack,childIdx2)
            }
        }
    }
}