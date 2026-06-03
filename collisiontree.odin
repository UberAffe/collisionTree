package collisiontree

import "base:runtime"
import enc "core:encoding/json"
import "core:fmt"
import "core:os"
import time "core:time"

batchedScan::proc(colTree:CollisionTree,rc:^BatchResponse,rays:[]Ray,offset:u32=0){
	rc.searchTime = 0
	sw := time.Stopwatch{}
	tb: u32 = 0
	tt: u32 = 0
	for &ray, i in rays {
		time.stopwatch_start(&sw)
		b, t, sID := _intersectBVH(colTree, &ray)
		time.stopwatch_stop(&sw)
		tb += b
		tt += t
		if ray.t < MAX && sID >= 0 {
			key := u32(i) + offset
			append(&rc.hits, Hit{key, sID, ray.t})
		}
	}
	
	rc.searchTime = time.stopwatch_duration(sw)
	rc.boundsChecks=tb
	rc.shapeChecks=tt
}

saveBVH :: proc(path: string, colTree: ^CollisionTree) -> int {
	file, err := os.open(path, {.Create, .Write, .Read})
	if err != nil {
		fmt.printfln("file error: %v", err)
		return 0
	}
	defer os.close(file)
	binary, encError := enc.marshal(colTree^, {})
	if encError != nil {
		fmt.printfln("encoding error:%v", encError)
		return 0
	}
	defer delete(binary)
	written, writeEr := os.write(file, binary)
	if writeEr != nil {
		fmt.printfln("write error: %v", writeEr)
		return 0
	}
	return written
}

loadBVH :: proc(path: string, inputTri: []Shape) -> ^CollisionTree {
	file, err := os.open(path, {.Create, .Write, .Read})
	defer os.close(file)
	if err != nil do return nil
	binary, _ := os.read_entire_file(file, context.allocator)
	colTree := new(CollisionTree)
	// unerr := enc.unmarshal(binary, colTree)
	// if unerr != nil do return nil
	// colTree.tri = inputTri
	return colTree
}
