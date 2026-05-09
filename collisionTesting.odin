package collisiontree

import "core:fmt"
import "core:log"
import la "core:math/linalg"
import "core:math/rand"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:sync/chan"
import "core:time"
import rl "vendor:raylib"

import ct "../collisionTree"
N :: 640
TEST:: 64

pixelMutex: ^sync.Mutex
pixels: [N * N]rl.Color
searchTime: time.Duration
recvComms: chan.Chan(^ThreadContext, .Recv)


main :: proc() {
	fmt.println("Collision Test Started")
	collisionTreeInit(12)

	fmt.println("Bulding Test Triangles")
	inputTri := buildTestTriangles2()
	fmt.println("triangles built")
	bWatch := time.Stopwatch{}
	tree :^ct.CollisionTree= loadBVH("model.bvh", inputTri)
	if tree == nil {
		fmt.println("Building BVH")
		time.stopwatch_start(&bWatch)
		tree = ct.BuildBVH(inputTri)
		time.stopwatch_stop(&bWatch)
		fmt.println("BVH built")
		ct.saveBVH("model.bvh", tree)
	}


	pixelMutex = new(sync.Mutex)
	rl.InitWindow(640, 640, "test")
	defer rl.CloseWindow()
	camPos := fl3{-1.5, -.2, -2.5}
	p0 := fl3{-2.5, .8, -.5}
	p1 := fl3{-.5, .8, -.5}
	p2 := fl3{-2.5, -1.2, -.5}

	rays := make([dynamic]Ray, 640 * 640, 640 * 640)
	for &ray, i in rays {
		y := uint(i) / 640
		x := uint(i) % 640
		ray.O = camPos
		ray.D = la.normalize(
			(p0 + (p1 - p0) * (f32(x) / 640) + (p2 - p0) * (f32(y + 0) / 640)) - ray.O,
		)
		ray.t = MAX_F32
	}

	for !rl.WindowShouldClose() {
		fmt.println("frame start")
		searchTime = 0
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		for &ray in rays {
			ray.t = MAX_F32
		}
		count: int
		recvComms, count = ct.collisionTreeBatchedRayScan(tree, rays[:], 1280)
		for i in 0 ..< count {
            //this call blocks until a message is recieved.
			data, ok := chan.recv(recvComms)
			assert(ok)
            fmt.printfln("ray hits: %v",len(data.Hit))
			for rayID, shapeID in data.Hit {
				v := u8(500 - data.rays[rayID-data.offset].t * 55)
				rl.DrawPixel(i32(rayID)%N,i32(rayID)/N, {v, v, v, 255})
			}
			searchTime += data.searchTime
		}
		rl.DrawFPS(10, 10)
		rl.DrawText(
			fmt.ctprintf(
				"build time: %v\ncumulative search time: %v\naverage search time: %v\n409,600 rays across %v threads\ntriangles: %v\nTPR: %v",
				time.stopwatch_duration(bWatch),
				searchTime,
				searchTime / time.Duration(num_CPU),
				num_CPU,
				len(inputTri),
				searchTime / (640 * 640),
			),
			10,
			40,
			16,
			{0, 0, 0, 255},
		)
		rl.EndDrawing()
	}
	ct.saveBVH("model.bvh", tree)
	ct.collistionTreeCleanup()
}

buildTestTriangles :: proc() -> []^Shape {
	input := make([]^Shape, TEST)
	rand.reset(12345678910)
	rf := rand.float32_uniform
	for &t, i in input {
		triangle := Tri{}
		r0 := fl3{rf(-3, 1), rf(-3, 1), rf(-3, 1)}
		r1 := fl3{rf(-3, 1), rf(-3, 1), rf(-3, 1)}
		r2 := fl3{rf(-3, 1), rf(-3, 1), rf(-3, 1)}
		triangle.vertex0 = r0//r0 * 9 - fl3{5, 5, 5}
		triangle.vertex1 = r1//triangle.vertex0 + r1 * 2
		triangle.vertex2 = r2//triangle.vertex0 + r2 * 2
		input[i] = new(Shape)
		input[i].aabb = _getTriangleAABB(triangle)
		input[i].type = triangle
		input[i].centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
	}
	return input
}

buildTestTriangles2 :: proc() -> []^Shape {
	data, err := os.read_entire_file("assets/unity.tri", context.allocator)
	defer delete(data, context.allocator)
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	input := make([dynamic]^Shape)
	for line in strings.split_lines_iterator(&iterator) {
		vals: []string
		vals, err = strings.split(line, " ")
		for v, j in vals {
			pointList[j], _ = strconv.parse_f32(v)
		}
		triangle := Tri {
			{pointList[0], pointList[1], pointList[2]},
			{pointList[3], pointList[4], pointList[5]},
			{pointList[6], pointList[7], pointList[8]},
		}
		s := new(Shape)
		s.centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type = triangle
		append(&input, s)
	}
	return input[:]
}
