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
TILEW :: 80
TILEH :: 80
SCALEW :: N / TILEW
SCALEH :: N / TILEH
LOGLEVEL :: #config(llevel, 0)
LOGGING :: #config(log, false)

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
taskGroup: sync.Wait_Group
tex: rl.Texture2D
batchCount := 32
mesh: [dynamic]Shape
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
baseP: [3]fl3 = {{-1, 1, 2}, {1, 1, 2}, {-1, -1, 2}}
p := baseP
baseCamPos := fl3{0, .5, -4.5}
camPos := baseCamPos
tlas: TLAS

main :: proc() {
	//logging
	file, _ := os.open("logs/latest.log", {.Read, .Write, .Append, .Create})
	defer os.close(file)
	fLogger := log.create_file_logger(file, log.Level(LOGLEVEL))
	defer log.destroy_file_logger(fLogger)
	tLogger = TL.CreateThreadedLogger(fLogger)
	defer TL.destroy()
	context.logger = tLogger
	//profiling
	when PROFILING {
		spall_ctx = spall.context_create("profiling/ct.spall")
		defer spall.context_destroy(&spall_ctx)

		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		defer delete(buffer_backing)

		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer spall.buffer_destroy(&spall_ctx, &spall_buffer)

		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	//debugging
	when ODIN_DEBUG {
		track: mem.Tracking_Allocator
		mem.tracking_allocator_init(&track, context.allocator)
		context.allocator = mem.tracking_allocator(&track)
		defer {
			if len(track.allocation_map) > 0 {
				log.warnf("tracking allocator output starts here: %v", len(track.allocation_map))
			}
			for err, entry in track.allocation_map {
				log.warnf("%v leaked %v bytes\n", entry.location, entry.size)
			}
			mem.tracking_allocator_destroy(&track)
		}
	}
	//prepare threadpool
	threadCount := os.get_processor_core_count() - 2
	a := new(mem.Dynamic_Arena)
	defer free(a)
	mem.dynamic_arena_init(a)
	customPoolAlloc = mem.dynamic_arena_allocator(a)
	defer free_all(customPoolAlloc)
	customPool = new(thread.Pool)
	defer free(customPool)
	thread.pool_init(customPool, customPoolAlloc, threadCount)
	thread.pool_start(customPool)
	//prepare image texture
	pixels = make([dynamic]rl.Color, N * N)
	defer delete(pixels)
	for _, i in pixels {
		pixels[i] = {30, 30, 30, 255}
	}
	im := rl.Image{raw_data(pixels), N, N, 1, rl.PixelFormat.UNCOMPRESSED_R8G8B8A8}
	//load the mesh
	mesh = buildTestTriangles2()
	defer delete(mesh)
	//build BVH of the mesh
	bvh := Create(mesh[:])
	Build(&bvh, BINS, false)
	// _balancedBVH(bvh)
	defer _bvh_Destroy(bvh)
	//create TLAS struct, this could be done manually, but it is easier to wrap it
	tlas = Create()
	defer _tlas_Destroy(tlas)
	//create the lower level blas entries
	append(&tlas.bvhList, bvh)
	append(&tlas.blas, BLAS{bvhIndex = 0}, BLAS{bvhIndex = 0})

	//Init raylib stuff
	rl.InitWindow(640, 640, "test")
	tex = rl.LoadTextureFromImage(im)
	//whe profiling, only run for a single frame
	when PROFILING {
		Frame(0)
	} else {
		frame: u64 = 0
		for !rl.WindowShouldClose() {
			Frame(frame)
			frame += 1
		}
	}

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
	time.stopwatch_reset(&frameWatch)
	time.stopwatch_start(&frameWatch)

	rl.BeginDrawing()
	rl.ClearBackground(rl.WHITE)
	for _, i in pixels {
		pixels[i] = {30, 30, 30, 255}
	}
	time.stopwatch_reset(&searchWatch)
	time.stopwatch_start(&searchWatch)
	_tick(f32(frame) / 10)
	time.stopwatch_stop(&searchWatch)
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
	when LOGGING {log.logf(log.Level(1), "Frame time %v", time.stopwatch_duration(frameWatch))}
	rl.EndDrawing()
}

swingCamera :: proc(angle: f32) {
	dist := la.distance(fl3{}, camPos)
	r := la.matrix4_rotate(angle, fl3{0, 1, 0})
	camPos = _transformPosition(baseCamPos, r)
	for corner, i in baseP {
		dist = la.distance(fl3{}, corner)
		p[i] = _transformPosition(corner, r)
	}
}

_tick :: proc(dt: f32) {
	when PROFILING {profileStart()}
	angle := dt / 4
	swingCamera(angle)
	m1 := la.matrix4_translate(fl3{-1.3, 0, 0})
	m2 := la.matrix4_translate(fl3{1.3, 0, 0}) * la.matrix4_rotate(angle, fl3{0, 1, 0})
	SetTransform(&tlas.blas[0], tlas.bvhList[tlas.blas[0].bvhIndex].node[0].aabb, m1)
	SetTransform(&tlas.blas[1], tlas.bvhList[tlas.blas[1].bvhIndex].node[0].aabb, m2)
	_tlas_Build(&tlas)
	tb, tt: u32
	sync.wait_group_add(&taskGroup, TILEW * TILEH)
	for tile in 0 ..< TILEW * TILEH {
		thread.pool_add_task(customPool, context.allocator, threadedTile, nil, tile)
	}
	sync.wait_group_wait(&taskGroup)
	when LOGGING {log.logf(log.Level(19), "Total AABB checks: %v, Total Tri checks: %v", tb, tt)}
	texUpdate = true
}

threadedTile :: proc(task: thread.Task) {
	when PROFILING {
		buffer_backing := make([]u8, spall.BUFFER_DEFAULT_SIZE)
		spall_buffer = spall.buffer_create(buffer_backing, u32(sync.current_thread_id()))
		defer {
			spall.buffer_destroy(&spall_ctx, &spall_buffer)
			delete(buffer_backing)
		}
		spall.SCOPED_EVENT(&spall_ctx, &spall_buffer, #procedure)
	}
	when ODIN_DEBUG {
		track: mem.Tracking_Allocator
		mem.tracking_allocator_init(&track, context.allocator)
		context.allocator = mem.tracking_allocator(&track)
		defer {
			if len(track.allocation_map) > 0 {
				log.warnf("tracking allocator output starts here: %v", len(track.allocation_map))
			}
			for err, entry in track.allocation_map {
				log.warnf("%v leaked %v bytes\n", entry.location, entry.size)
			}
			mem.tracking_allocator_destroy(&track)
		}
	}
	context.logger = tLogger
	tile_tick(task.user_index)
	sync.wait_group_done(&taskGroup)
}

tile_tick :: proc(tile: int) -> (u32, u32) {
	tb, tt: u32 = 0, 0
	x, y := tile % TILEW, tile / TILEW
	ray: Ray
	ray.O = camPos
	for v in 0 ..< SCALEH {
		for u in 0 ..< SCALEW {
			when PROFILING {profileStart(
					fmt.tprint("Pixel ", x * SCALEW + u, ",", y * SCALEH + v, sep = ""),
				)}
			pixelPos :=
				ray.O +
				p.x +
				(p.y - p.x) * (f32(x * SCALEW + u) / 640) +
				(p.z - p.x) * (f32(y * SCALEH + v) / 640)
			ray.D = la.normalize(pixelPos - ray.O)
			ray.rD = 1 / ray.D
			ray.t = MAX
			bIt, tIt, shapeID := Intersect(tlas, &ray)
			tb += bIt
			tt += tIt
			c: u8 = u8(240 - (ray.t - 3) * 50)
			pixels[(x * SCALEW + u) + (y * SCALEH + v) * 640] = rl.Color{c, c, c, 255}.rgba
		}
	}
	return tb, tt
}

buildTestTriangles2 :: proc(alloc := context.allocator) -> [dynamic]Shape {
	data, err := os.read_entire_file("assets/armadillo.tri", alloc)
	iterator := string(data)
	pointList := make([dynamic]f32, 9, 9)
	defer delete(pointList)
	input := make([dynamic]Shape, alloc)
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
