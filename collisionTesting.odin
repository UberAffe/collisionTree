package collisiontree

import "base:runtime"


import "../ThreadLogger"
import "core:fmt"
import "core:log"
import "core:math"
import la "core:math/linalg"
import "core:math/rand"
import "core:mem"
import "core:os"
import "core:prof/spall"
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
PROFILING :: #config(profiling, false)
LOGLEVEL :: #config(llevel,20)

when PROFILING {
	spall_ctx: spall.Context
	@(thread_local)
	spall_buffer: spall.Buffer

	@(instrumentation_enter)
	spall_enter :: proc "contextless" (
		proc_address, call_site_return_address: rawptr,
		loc: runtime.Source_Code_Location
	) {
		spall._buffer_begin(&spall_ctx, &spall_buffer, "", "", loc)
	}

	@(instrumentation_exit)
	spall_exit :: proc "contextless" (
		proc_address, call_site_return_address: rawptr,
		loc: runtime.Source_Code_Location,
	) {
		spall._buffer_end(&spall_ctx, &spall_buffer)
	}
}

pixels: [dynamic]rl.Color
rays: [dynamic]ct.Ray
ready := false
texUpdate := false
tree: ^ct.CollisionTree
searchTime: time.Duration
searchWatch: time.Stopwatch
frameWatch: time.Stopwatch
recvComms: chan.Chan(^ResponseContext, .Recv)
customPool: ^thread.Pool
customPoolAlloc: mem.Allocator
tex: rl.Texture2D
textureUpdating: ^sync.Mutex
batchCount := 32
original: [dynamic]ct.Shape
inputTri: [dynamic]ct.Shape
bWatch: time.Stopwatch
r: f32 = 0
lastBuildCost: f64 = 0
lastRefitCost: f64 = 0
threadLogger: log.Logger

main :: proc() {
	file, _ := os.open("logs/latest.log", {.Read, .Write, .Append, .Create})
	defer os.close(file)
	fLogger := log.create_file_logger(file, log.Level(LOGLEVEL))
	defer log.destroy_file_logger(fLogger)
	threadLogger = ThreadLogger.CreateThreadedLogger(fLogger)
	defer ThreadLogger.destroy()
	context.logger = threadLogger
	when PROFILING {
		spall_ctx = spall.context_create("profiling/ct.spall")
		defer spall.context_destroy(&spall_ctx)

		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	when ODIN_DEBUG {
		track: mem.Tracking_Allocator
		mem.tracking_allocator_init(&track, context.allocator)
		context.allocator = mem.tracking_allocator(&track)
		defer {
			log.warnf("tracking allocator output starts here: %v", len(track.allocation_map))
			if (len(track.allocation_map) > 0) {
				for err, entry in track.allocation_map {
					log.warnf("%v leaked %v bytes\n", entry.location, entry.size)
					// if err!=nil do fmt.println(err)
				}
			}
			mem.tracking_allocator_destroy(&track)
		}
	}
	threadCount := os.get_processor_core_count() - 2
	a := new(mem.Dynamic_Arena)
	defer free(a)
	mem.dynamic_arena_init(a)
	customPoolAlloc = mem.dynamic_arena_allocator(a)
	defer free_all(customPoolAlloc)
	textureUpdating = new(sync.Mutex)
	defer free(textureUpdating)
	customPool = new(thread.Pool)
	defer free(customPool)
	thread.pool_init(customPool, customPoolAlloc, 1)
	thread.pool_start(customPool)
	collisionTreeInit(threadCount)

	pixels = make([dynamic]rl.Color, N * N)
	defer delete(pixels)
	for _, i in pixels {
		pixels[i] = {30, 30, 30, 255}
	}
	im := rl.Image{raw_data(pixels), N, N, 1, rl.PixelFormat.UNCOMPRESSED_R8G8B8A8}

	original = buildTestTriangles2()
	inputTri = make([dynamic]ct.Shape, len(original))
	copy(inputTri[:], original[:])

	defer delete(original)
	defer delete(inputTri)
	bWatch = time.Stopwatch{}

	tree, lastBuildCost = ct.BuildBVH(inputTri[:], 8, true)
	defer {
		delete(tree.bvhNode)
		delete(tree.shapeIdx)
		free(tree)
	}

	// rl.SetTargetFPS(30)
	// thread.pool_add_task(customPool, context.allocator, FullDepthScan, nil, 0)

	rl.InitWindow(640, 640, "test")
	tex = rl.LoadTextureFromImage(im)

	frame: u64 = 0
	for !rl.WindowShouldClose() {
		// r=f32(rl.GetMouseX()/320)*math.TAU-math.TAU
		log.logf(log.Level(17),"frame %v start", frame)
		time.stopwatch_reset(&frameWatch)
		time.stopwatch_start(&frameWatch)
		frame += 1
		if rl.IsKeyDown(.UP) do batchCount = math.min(N, batchCount + 1)
		if rl.IsKeyDown(.DOWN) do batchCount = math.max(1, batchCount - 1)
		rl.BeginDrawing()
		rl.ClearBackground(rl.WHITE)
		for _, i in pixels {
			pixels[i] = {30, 30, 30, 255}
		}
		time.stopwatch_reset(&searchWatch)
		time.stopwatch_start(&searchWatch)
		FullDepthScan()
		time.stopwatch_stop(&searchWatch)
		animate()
		if texUpdate {
			texUpdate = false
			rl.UpdateTexture(tex, raw_data(pixels))
		}

		rl.DrawTexture(tex, 0, 0, rl.WHITE)
		rl.DrawRectangle(0, 0, 3, i32(batchCount), {180, 140, 140, 180})
		rl.DrawFPS(10, 10)
		time.stopwatch_stop(&frameWatch)
		if ready {
			rl.DrawText(
				fmt.ctprintf(
					"lastBuildCost: %v\nlastRefitCost: %v\ntime to display: %v",
					lastBuildCost,
					lastRefitCost,
					time.stopwatch_duration(frameWatch),
				),
				10,
				40,
				16,
				{150, 180, 150, 255},
			)
			log.infof("Frame time %v",time.stopwatch_duration(frameWatch))
		}

		rl.EndDrawing()
	}
	log.info("exiting program")
}

FullDepthScan :: proc {
	_threadedFullDepthScan,
	_fullDepthScan,
}

_threadedFullDepthScan :: proc(task: thread.Task) {
	when PROFILING {
		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	context.logger = threadLogger
	context.allocator = task.allocator
	_fullDepthScan()
}

_fullDepthScan :: proc() {
	time.stopwatch_reset(&bWatch)
	time.stopwatch_start(&bWatch)
	if math.abs(lastBuildCost - lastRefitCost) < lastBuildCost * .2 {
		lastRefitCost = ct.RefitBVH(tree)
		time.stopwatch_stop(&bWatch)
		log.logf(log.Level(17),
			"refitting with a cost of %v and it took %v",
			lastRefitCost,
			time.stopwatch_duration(bWatch),
		)
	} else {
		if tree!=nil{
			delete(tree.bvhNode)
			delete(tree.shapeIdx)
			free(tree)
		}
		tree, lastBuildCost = ct.BuildBVH(inputTri[:], 8, true)
		time.stopwatch_stop(&bWatch)
		lastRefitCost = lastBuildCost
		log.logf(log.Level(17),
			"rebuilding with a cost of %v and it took %v",
			lastBuildCost,
			time.stopwatch_duration(bWatch),
		)
	}

	camPos := fl3{0, 3.5, -4.5}
	p0 := fl3{-1, 1, 2}
	p1 := fl3{1, 1, 2}
	p2 := fl3{-1, -1, 2}
	rays := make([dynamic]Ray, 640 * 640, 640 * 640)
	defer delete(rays)
	for &ray, i in rays {
		y := uint(i) / 640
		x := uint(i) % 640
		ray.O = camPos
		ray.D = la.normalize(
			(camPos + p0 + (p1 - p0) * (f32(x) / 640) + (p2 - p0) * (f32(y + 0) / 640)) - ray.O,
		)
		ray.rD = 1 / ray.D
		ray.t = MAX
	}
	count: int
	searchTime = 0
	time.stopwatch_reset(&bWatch)
	time.stopwatch_start(&bWatch)
	recvComms, count = ct.collisionTreeBatchedRayScan(tree, rays[:], batchCount)

	for count > 0 {
		data := chan.recv(recvComms) or_break
		log.logf(log.Level(15),"batch %v took %v to hit %v times", data.batchID, data.searchTime, len(data.hits))
		for hit in data.hits {
			v: u8 = u8(255 - (rays[hit.rayID].t - 4) * 180)
			pixels[hit.rayID] = rl.Color{v, v, v, 255}.rgba
		}
		searchTime += data.searchTime
		count -= 1
	}
	time.stopwatch_stop(&bWatch)
	log.logf(log.Level(17),"summed search time of %v with observed search of %v",searchTime, time.stopwatch_duration(bWatch))
	texUpdate = true
	ready = true
}

animate :: proc() {
	r += .05
	if r > math.TAU do r -= math.TAU
	a := math.sin(r) * .1 //.5*.2
	for i in 0 ..< len(original) {
		for j in 0 ..< 3 {
			o: ct.fl3
			switch type in original[i].type {
			case Tri:
				o = type.vertex[j]
			}
			s := a * (o.y - .2)
			x := o.x * math.cos(s) - o.y * math.sin(s)
			y := o.x * math.sin(s) + o.y * math.cos(s)
			#partial switch &type in inputTri[i].type {
			case Tri:
				type.vertex[j] = ct.fl3{x, y, o.z}
				if j == 2 do inputTri[i].aabb = _getTriangleAABB(type)
			}
		}

	}
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
		triangle.vertex[0] = r0 //r0 * 9 - fl3{5, 5, 5}
		triangle.vertex[1] = r1 //triangle.vertex0 + r1 * 2
		triangle.vertex[2] = r2 //triangle.vertex0 + r2 * 2
		input[i] = new(Shape)
		input[i].aabb = _getTriangleAABB(triangle)
		input[i].type = triangle
		input[i].centroid = (triangle.vertex[0] + triangle.vertex[1] + triangle.vertex[2]) / 3
	}
	return input
}

buildTestTriangles2 :: proc() -> [dynamic]Shape {
	err: os.Error
	data := #load("assets/bigben.tri", []byte) or_else []byte{}
	deletedata := false
	// if data==nil{
	// 	data, err = os.read_entire_file("assets/bigben.tri", context.allocator)
	// 	deletedata=true		
	// }
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	defer delete(pointList)
	input := make([dynamic]Shape)
	for line in strings.split_lines_iterator(&iterator) {
		vals: []string
		defer delete(vals)
		vals, err = strings.split(line, " ")
		for v, j in vals {
			pointList[j], _ = strconv.parse_f32(v)
		}
		triangle := ct.Tri{}
		triangle.vertex[0] = {pointList[0], pointList[1], pointList[2]}
		triangle.vertex[1] = {pointList[3], pointList[4], pointList[5]}
		triangle.vertex[2] = {pointList[6], pointList[7], pointList[8]}
		s := Shape{}
		s.centroid = (triangle.vertex[0] + triangle.vertex[1] + triangle.vertex[2]) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type = triangle
		append(&input, s)
	}
	if deletedata do delete(data)
	return input
}

_getTriangleAABB :: proc(leaf: ct.Tri) -> AABB {
	bounds: AABB = {{MIN, MIN, MIN}, {MAX, MAX, MAX}}
	for v, i in leaf.vertex {
		bounds.lower = _fminf(bounds.lower, v)
		bounds.upper = _fmaxf(bounds.upper, v)
	}
	return bounds
}
