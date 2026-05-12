package collisiontree

import "core:math"
import "core:fmt"
import "core:log"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import "core:os"
import "core:strconv"
import "core:strings"
import "core:sync"
import "core:sync/chan"
import "core:thread"
import "core:time"
import rl "vendor:raylib"

import ct "../collisionTree"
N :: 640
TEST :: 64

pixels: [dynamic]rl.Color
rays: [dynamic]ct.Ray
ready := false
texUpdate:= false
tree: ^ct.CollisionTree
searchTime: time.Duration
searchWatch: time.Stopwatch
recvComms: chan.Chan(ThreadContext, .Recv)
customPool: ^thread.Pool
customPoolAlloc: mem.Allocator
tex:rl.Texture2D
textureUpdating:^sync.Mutex
batchCount:=1
original:[]ct.Shape


main :: proc() {
	fmt.println("Collision Test Started")
	threadCount := os.get_processor_core_count()-2
	a := new(mem.Dynamic_Arena)
	mem.dynamic_arena_init(a)
	// defer mem.dynamic_arena_destroy(a)
	customPoolAlloc = mem.dynamic_arena_allocator(a)
	// defer free_all(customPoolAlloc)

    textureUpdating= new(sync.Mutex)
    // defer free(textureUpdating)

	customPool = new(thread.Pool)
	// defer thread.pool_destroy(customPool)
	thread.pool_init(customPool, customPoolAlloc, 1)
	thread.pool_start(customPool)
	// defer thread.pool_shutdown(customPool)
	// defer thread.pool_destroy(customPool)
	collisionTreeInit(threadCount)

    pixels=make([dynamic]rl.Color,N*N)
    for _,i in pixels{
        pixels[i]={30,30,30,255}
    }
    im:= rl.Image{raw_data(pixels),N,N,1,rl.PixelFormat.UNCOMPRESSED_R8G8B8A8}

	fmt.println("Bulding Test Triangles")
	original= buildTestTriangles2()
	inputTri:= original[:]
	fmt.println("triangles built")
	bWatch := time.Stopwatch{}
	// tree = loadBVH("model.bvh", inputTri)
	// if tree == nil {
		fmt.println("Building BVH")
		time.stopwatch_start(&bWatch)
		tree = ct.BuildBVH(inputTri, 24, true)
		time.stopwatch_stop(&bWatch)
		fmt.println("BVH built")
		ct.saveBVH("model.bvh", tree)
	// }

    rl.SetTargetFPS(30)
    thread.pool_add_task(customPool, context.allocator, FullDepthScan, nil, 0)

	rl.InitWindow(640, 640, "test")
    tex=rl.LoadTextureFromImage(im)
    // defer rl.UnloadTexture(tex)
	// defer rl.CloseWindow()

	for !rl.WindowShouldClose() {
		// fmt.println("frame start")
        if rl.IsKeyDown(.UP) do batchCount= math.min(N,batchCount+1)
        if rl.IsKeyDown(.DOWN) do batchCount= math.max(1,batchCount-1)

		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		if rl.IsMouseButtonPressed(.LEFT) || rl.IsKeyPressed(.SPACE) {
			for _,i in pixels {
				pixels[i]={30,30,30,255}
			}
            texUpdate=true
			ready = false
			thread.pool_add_task(customPool, context.allocator, FullDepthScan, nil, 0)
            time.stopwatch_reset(&searchWatch)
            time.stopwatch_start(&searchWatch)
		}
        if texUpdate {
            texUpdate=false
            rl.UpdateTexture(tex,raw_data(pixels))
        }
		rl.DrawTexture(tex,0,0,rl.WHITE)
        rl.DrawRectangle(0,0,3,i32(batchCount),{180,140,140,180})
		rl.DrawFPS(10, 10)
		if ready {
			rl.DrawText(
				fmt.ctprintf(
					"build time: %v\ncumulative search time: %v\naverage search time per batch: %v\ntime to display: %v\ntriangles: %v\nTPR: %v",
					time.stopwatch_duration(bWatch),
					searchTime,
					searchTime / time.Duration(batchCount),
					time.stopwatch_duration(searchWatch),
					len(inputTri),
					searchTime / (640 * 640),
				),
				10,
				40,
				16,
				{150, 180, 150, 255},
			)
		}

		rl.EndDrawing()
	}
	ct.saveBVH("model.bvh", tree)
    // fmt.println("why is this still open?")
	// ct.collistionTreeCleanup()
}

FullDepthScan :: proc(task: thread.Task) {
	camPos := fl3{0,3.5,-4.5}
	p0 := fl3{-1,1,2}
	p1 := fl3{1,1,2}
	p2 := fl3{-1,-1,2}
	rays := make([dynamic]Ray, 640 * 640, 640 * 640)
	for &ray, i in rays {
		y := uint(i) / 640
		x := uint(i) % 640
		ray.O = camPos
		ray.D = la.normalize(
			(camPos+p0 + (p1 - p0) * (f32(x) / 640) + (p2 - p0) * (f32(y + 0) / 640)) - ray.O,
		)
        ray.rD = 1/ray.D
		ray.t = MAX_F32
	}
	for &ray in rays {
		ray.t = MAX_F32
	}
	count: int
	searchTime = 0
	recvComms, count = ct.collisionTreeBatchedRayScan(tree, rays[:], batchCount)
	for i in 0 ..< count {
		//this call blocks until a message is recieved.
		data, ok := chan.recv(recvComms)
		assert(ok)
		for hit in data.hit {
			// hit := pop(&data.hit)
			if hit.rayID < data.offset || hit.rayID - data.offset >= u32(len(data.rays)) {
				fmt.printfln(
					"ray %v with offset %v from task %v",
					hit.rayID,
					data.offset,
					data.offset / u32(len(data.rays))
				)
			}
			v: u8 = u8(255 - (data.rays[hit.rayID - data.offset].t-4) * 180)
			pixels[hit.rayID] = rl.Color{v, v, v, 255}.rgba
		}
		searchTime += data.searchTime
	}
    time.stopwatch_stop(&searchWatch)
    texUpdate=true
    ready=true
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
		triangle.vertex0 = r0 //r0 * 9 - fl3{5, 5, 5}
		triangle.vertex1 = r1 //triangle.vertex0 + r1 * 2
		triangle.vertex2 = r2 //triangle.vertex0 + r2 * 2
		input[i] = new(Shape)
		input[i].aabb = _getTriangleAABB(triangle)
		input[i].type = triangle
		input[i].centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
	}
	return input
}

buildTestTriangles2 :: proc() -> []Shape {
	data, err := os.read_entire_file("assets/bigben.tri", context.allocator)
	defer delete(data, context.allocator)
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	input := make([dynamic]Shape)
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
		s := Shape{}
		s.centroid = (triangle.vertex0 + triangle.vertex1 + triangle.vertex2) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type = triangle
		append(&input, s)
	}
	return input[:]
}
