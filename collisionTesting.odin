package collisiontree

import "base:runtime"


import TL "../ThreadLogger"
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
BINS :: 8
// PROFILING :: #config(profiling, false)
LOGLEVEL :: #config(llevel, 20)

when PROFILING {
	spall_ctx: spall.Context
	@(thread_local)
	spall_buffer: spall.Buffer

	profileStart :: proc {
		spall_enter,
		spall_enter_named,
	}

	@(deferred_none = spall_exit)
	spall_enter :: proc(loc := #caller_location) {
		spall._buffer_begin(&spall_ctx, &spall_buffer, "", "", loc)
	}
	@(deferred_none = spall_exit)
	spall_enter_named :: proc(name: string) {
		spall._buffer_begin(
			&spall_ctx,
			&spall_buffer,
			"",
			"",
			runtime.Source_Code_Location{procedure = name},
		)
	}

	spall_exit :: proc() {
		spall._buffer_end(&spall_ctx, &spall_buffer)
	}
}

pixels: [dynamic]rl.Color
rays: [dynamic]Ray
ready := false
texUpdate := false
searchTime: time.Duration
searchWatch: time.Stopwatch
frameWatch: time.Stopwatch
recvComms: chan.Chan(^BatchResponse, .Recv)
customPool: ^thread.Pool
customPoolAlloc: mem.Allocator
tex: rl.Texture2D
textureUpdating: ^sync.Mutex
batchCount := 32
original: [dynamic]Shape
inputTri: [dynamic]Shape
bWatch: time.Stopwatch
r: f32 = 0
lastBuildCost: f64 = 0
lastRefitCost: f64 = 0
tLogger: log.Logger
height, width: uint = 640, 640
xOff, yOff: uint = 0, 0
click, release: [2]f32
deepLog := false
pixelPeek: [5]uint = {}
peekColor: [5]rl.Color = {}
p: [3]fl3 = {{-1, 1, 2}, {1, 1, 2}, {-1, -1, 2}}
camPos := fl3{0, .5, -4.5} //fl3{0, 3.5, -4.5}
tlas: TLAS

main :: proc() {
	file, _ := os.open("logs/latest.log", {.Read, .Write, .Append, .Create})
	defer os.close(file)
	fLogger := log.create_file_logger(file, log.Level(LOGLEVEL))
	defer log.destroy_file_logger(fLogger)
	tLogger = TL.CreateThreadedLogger(fLogger)
	defer TL.destroy()
	context.logger = tLogger
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
	// collisionTreeInit(threadCount)

	pixels = make([dynamic]rl.Color, N * N)
	defer delete(pixels)
	for _, i in pixels {
		pixels[i] = {30, 30, 30, 255}
	}
	im := rl.Image{raw_data(pixels), N, N, 1, rl.PixelFormat.UNCOMPRESSED_R8G8B8A8}

	original = buildTestTriangles2()
	inputTri = make([dynamic]Shape, len(original))
	copy(inputTri[:], original[:])

	defer delete(original)
	defer delete(inputTri)
	bWatch = time.Stopwatch{}

	bvh: BVH
	bvh, lastBuildCost = BuildBVH(inputTri[:], BINS, false)
	tlas= Create_TLAS()
	append(&tlas.bvhList, bvh)
	append(&tlas.blas, BLAS{bvhIndex = 0}, BLAS{bvhIndex = 0})
	Build_TLAS(&tlas)

	m1 := la.matrix4_translate(fl3{-1.3, 0, 0})
	m2 := la.matrix4_translate(fl3{1.3, 0, 0})
	SetTransform(&tlas.blas[0],tlas.bvhList[tlas.blas[0].bvhIndex].bvhNode[0].aabb, m1)
	SetTransform(&tlas.blas[1],tlas.bvhList[tlas.blas[1].bvhIndex].bvhNode[0].aabb, m2)

	balancedBVH(tlas.bvhList[0])
	defer {
		Destroy_TLAS(tlas)
		// delete(tree.bvhNode)
		// delete(tree.shapeIdx)
		// free(tree)
	}

	rl.InitWindow(640, 640, "test")
	tex = rl.LoadTextureFromImage(im)

	frame: u64 = 0
	when !PROFILING {
		for !rl.WindowShouldClose() {
			Frame(frame)
			frame += 1
		}
	}
	when PROFILING {Frame(0)}

}

Frame :: proc(frame: u64) {
	if rl.IsMouseButtonPressed(.LEFT) {
		click = rl.GetMousePosition()
	} else if rl.IsMouseButtonReleased(.LEFT) {
		release = rl.GetMousePosition()
		xOff = uint(min(click.x, release.x))
		yOff = uint(min(click.y, release.y))
		width = uint(max(click.x, release.x)) - xOff
		height = uint(max(click.y, release.y)) - yOff
	}
	if rl.IsKeyReleased(.L) do deepLog = !deepLog
	if rl.IsMouseButtonReleased(.RIGHT) {
		pos := rl.GetMousePosition()
		center := uint(pos.x + pos.y * 640)
		pixelPeek[0] = center
		pixelPeek[1] = center + 1
		pixelPeek[2] = center - 1
		pixelPeek[3] = center + 640
		pixelPeek[4] = center - 640
		for i in 0 ..< len(peekColor) {
			peekColor[i] = rl.Color{200, 100, 100, 255}
		}
		texUpdate = true
	}
	if rl.IsKeyReleased(.SPACE) {
		if deepLog {
			tLogger.lowest_level = log.Level(0)

		} else {
			tLogger.lowest_level = log.Level(LOGLEVEL)
		}
		context.logger = tLogger
		fmt.println(deepLog)
		fmt.println(context.logger.lowest_level)
		for i in 0 ..< len(pixelPeek) {
			// _immediateIntersect(tree, pixelPeek[i] % 640, pixelPeek[i] / 640)
			peekColor[i] = pixels[pixelPeek[i]]
			peekColor[i].g += 30
			peekColor[i].r -= 5
			peekColor[i].b += 10
		}
		texUpdate = true
		tLogger.lowest_level = log.Level(LOGLEVEL)
		context.logger = tLogger
	}

	log.logf(log.Level(11), "frame %v start", frame)
	time.stopwatch_reset(&frameWatch)
	time.stopwatch_start(&frameWatch)

	rl.BeginDrawing()
	rl.ClearBackground(rl.WHITE)
	// if rl.IsKeyPressed(.ENTER) || frame == 1 {
	for _, i in pixels {
		pixels[i] = {30, 30, 30, 255}
	}
	time.stopwatch_reset(&searchWatch)
	time.stopwatch_start(&searchWatch)
	// FullDepthScan()
	_tick(f32(frame) / 10)
	time.stopwatch_stop(&searchWatch)
	// animate()
	if texUpdate {
		if peekColor[0].a != 0 {
			for i in 0 ..< len(pixelPeek) {
				pixels[pixelPeek[i]] = peekColor[i]
			}
		}
		rl.UpdateTexture(tex, raw_data(pixels))
		texUpdate = false
	}


	rl.DrawTexture(tex, 0, 0, rl.WHITE)
	rl.DrawFPS(10, 10)
	time.stopwatch_stop(&frameWatch)
	rl.DrawText(
		fmt.ctprintf(
			"lastBuildCost: %v\nlastRefitCost: %v\ntime to display: %v\ndeepLog? %v",
			lastBuildCost,
			lastRefitCost,
			time.stopwatch_duration(frameWatch),
			deepLog,
		),
		10,
		40,
		16,
		{150, 180, 150, 255},
	)
	log.logf(log.Level(1), "Frame time %v", time.stopwatch_duration(frameWatch))

	rl.EndDrawing()
}

FullDepthScan :: proc {
	_threadedFullDepthScan,
// _fullDepthScan,
}

_threadedFullDepthScan :: proc(task: thread.Task) {
	when PROFILING {
		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	context.logger = tLogger
	context.allocator = task.allocator
	// _fullDepthScan()
}

_tick :: proc(dt: f32) {
	when PROFILING {profileStart()}
	angle := math.sin(dt)
	m1 := la.matrix4_translate(fl3{-1.3, 0, 0})
	m2 := la.matrix4_translate(fl3{1.3, 0, 0}) * la.matrix4_rotate(angle, fl3{0, 1, 0})
	SetTransform(&tlas.blas[0],tlas.bvhList[tlas.blas[0].bvhIndex].bvhNode[0].aabb, m1)
	SetTransform(&tlas.blas[1],tlas.bvhList[tlas.blas[1].bvhIndex].bvhNode[0].aabb, m2)
	tb, tt: u32
	for tile in 0 ..< 6400 {
		x, y := tile % 80, tile / 80
		ray: Ray
		ray.O = camPos
		for v in 0 ..< 8 {
			for u in 0 ..< 8 {
				when PROFILING {profileStart("Pixel")}
				pixelPos :=
					ray.O +
					p.x +
					(p.y - p.x) * (f32(x * 8 + u) / 640) +
					(p.z - p.x) * (f32(y * 8 + v) / 640)
				ray.D = la.normalize(pixelPos - ray.O)
				ray.rD = 1 / ray.D
				ray.t = MAX

				bIt, tIt, shapeID := _intersectBVH(tlas,0, &ray)
				tb += bIt
				tt += tIt

				bIt, tIt, shapeID = _intersectBVH(tlas,1, &ray)
				tb += bIt
				tt += tIt
				c: u8 = u8(255 - (ray.t - 3) * 80)
				pixels[(x * 8 + u) + (y * 8 + v) * 640] = rl.Color{c, c, c, 255}.rgba
			}
		}
	}
	log.logf(log.Level(19), "Total AABB checks: %v, Total Tri checks: %v", tb, tt)
	texUpdate = true
	// }
}

// _immediateIntersect :: proc(bvh: ^CollisionTree, x, y: uint) {
// 	ray: Ray
// 	ray.O = camPos
// 	pixelPos := ray.O + p.x + (p.y - p.x) * (f32(x) / 640) + (p.z - p.x) * (f32(y) / 640)
// 	ray.D = la.normalize(pixelPos - ray.O)
// 	ray.t = MAX
// 	ray.rD = 1 / ray.D
// 	b, t, id := _intersectBVH(bvh, &ray)
// 	c: u8 = u8(255 - (ray.t - 4) * 180)
// 	pixels[(x) + (y) * 640] = rl.Color{c, c, c, 255}.rgba
// }

// _fullDepthScan :: proc() {
// 	time.stopwatch_reset(&bWatch)
// 	time.stopwatch_start(&bWatch)
// 	if math.abs(lastBuildCost - lastRefitCost) < lastBuildCost * .2 {
// 		lastRefitCost = RefitBVH(tree)
// 		time.stopwatch_stop(&bWatch)
// 		log.logf(
// 			log.Level(17),
// 			"refitting with a cost of %v and it took %v",
// 			lastRefitCost,
// 			time.stopwatch_duration(bWatch),
// 		)
// 	} else {
// 		if tree != nil {
// 			delete(tree.bvhNode)
// 			delete(tree.shapeIdx)
// 			free(tree)
// 		}
// 		tree, lastBuildCost = BuildBVH(inputTri[:], BINS, true)
// 		time.stopwatch_stop(&bWatch)
// 		lastRefitCost = lastBuildCost
// 		log.logf(
// 			log.Level(17),
// 			"rebuilding with a cost of %v and it took %v",
// 			lastBuildCost,
// 			time.stopwatch_duration(bWatch),
// 		)
// 	}

// 	rays := make([dynamic]Ray, int(width * height))
// 	defer delete(rays)
// 	for &ray, i in rays {
// 		y := uint(i) / uint(width) + uint(yOff)
// 		x := uint(i) % uint(width) + uint(xOff)
// 		ray.O = camPos
// 		ray.D = la.normalize(
// 			(camPos + p.x + (p.y - p.x) * (f32(x) / 640) + (p.z - p.x) * (f32(y + 0) / 640)) -
// 			ray.O,
// 		)
// 		ray.rD = 1 / ray.D
// 		ray.t = MAX
// 	}
// 	count: int
// 	searchTime = 0
// 	time.stopwatch_reset(&bWatch)
// 	time.stopwatch_start(&bWatch)
// 	response := BatchResponse{}
// 	batchedScan(tree, &response, rays[:])
// 	time.stopwatch_stop(&bWatch)
// 	for hit in response.hits {
// 		v := u8(255 - (hit.dist - 3.5946209) * 122.2661821)
// 		pixels[hit.rayID] = rl.Color{v, v, v, 255}.rgba
// 	}
// 	log.logf(
// 		log.Level(17),
// 		"summed search time of %v with observed search of %v | %v AABB checks and %v shape checks performed",
// 		response.searchTime,
// 		time.stopwatch_duration(bWatch),
// 		response.boundsChecks,
// 		response.shapeChecks,
// 	)
// 	texUpdate = true
// 	ready = true
// 	batchCount -= 1
// 	if batchCount == 0 do batchCount = 32
// }

animate :: proc() {
	r += .05
	if r > math.TAU do r -= math.TAU
	a := math.sin(r) * .1 //.5*.2
	for i in 0 ..< len(original) {
		for j in 0 ..< 3 {
			o: fl3
			switch type in original[i].type {
			case Tri:
				o = type.vertex[j]
			}
			s := a * (o.y - .2)
			x := o.x * math.cos(s) - o.y * math.sin(s)
			y := o.x * math.sin(s) + o.y * math.cos(s)
			#partial switch &type in inputTri[i].type {
			case Tri:
				type.vertex[j] = fl3{x, y, o.z}
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
	data, err := os.read_entire_file("assets/armadillo.tri", context.allocator)
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
		triangle := Tri{}
		triangle.vertex[0] = {pointList[0], pointList[1], pointList[2]}
		triangle.vertex[1] = {pointList[3], pointList[4], pointList[5]}
		triangle.vertex[2] = {pointList[6], pointList[7], pointList[8]}
		s := Shape{}
		s.centroid = (triangle.vertex[0] + triangle.vertex[1] + triangle.vertex[2]) / 3
		s.aabb = _getTriangleAABB(triangle)
		s.type = triangle
		append(&input, s)
	}
	delete(data)
	return input
}
