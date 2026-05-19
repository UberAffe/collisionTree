package collisiontree

RefitBVH::proc(ct:^CollisionTree) -> f64{
    cost:f64=0
    for i:=int(ct.nodesUsed-1);i>=0;i-=1{
        if i == 1 do continue
        ct.bvhNode[i].aabb=DEFAULTAABB
        if ct.bvhNode[i].triCount>0 {
            _UpdateNodeBounds(ct,u32(i))
            cost+= f64(_calculateNodeCost(ct.bvhNode[i]))
            continue
        }
        _GrowAABB(&ct.bvhNode[i], ct)
    }
    return cost
}