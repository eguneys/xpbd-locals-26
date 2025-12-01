import { DragHandler } from "./drag";
import { GameAction, InputController } from "./keyboard";
import type { SceneName } from "./main";
import { add_map, get_map } from "./maps_store";
import { buildPoly, findSegmentAtPoint, type Poly } from "./math/polygon";
import type { Vec2 } from "./math/vec2";
import { Color } from "./webgl/color";
import { g } from "./webgl/gl_init";

const FindSegmentThreshold = 24

type Camera = {
    x: number
    y: number
    zoom: number
}

function screenToWorld(screenX: number, screenY: number) {
    // (Screen - Center - CameraOffset) / Zoom
    const worldX = (screenX - area_center_x - camera.x) / camera.zoom;
    const worldY = (screenY - area_center_y - camera.y) / camera.zoom;
    return { x: worldX, y: worldY };
}

/**
 * Converts a world point to a screen point (used for drawing).
 */
export function worldToScreen(worldX: number, worldY: number) {

    // (World * Zoom) + Center + CameraOffset

    const screenX = worldX * camera.zoom + area_center_x + camera.x;
    const screenY = worldY * camera.zoom + area_center_y + camera.y;
    return { x: screenX, y: screenY };
}

let camera: Camera

let drag: DragHandler

type Cursor = {
    x: number
    y: number
    world_x: number
    world_y: number
    prev_x: number
    prev_y: number
    is_panning: boolean
}

let cursor: Cursor

let kb: InputController

let drawing_points: Vec2[]
let existing_polygons: Poly[]


let error_drawing_lines_timer: number
let is_hovering_on_close_point: boolean

export function _init() {
    set_next_scene = undefined

    kb = new InputController()

    camera = {
        x: 0,
        y: 0,
        zoom: 1,
    };

    drag = DragHandler(g.canvas)

    cursor = {
        x: 0,
        y: 0,
        world_x: 0,
        world_y: 0,
        prev_x: 0,
        prev_y: 0,
        is_panning: false
    }

    drawing_points = []
    existing_polygons = get_map()

    error_drawing_lines_timer = 0
    is_hovering_on_close_point = false
}

const MIN_ZOOM = 0.8
const MAX_ZOOM = 2

let area_x = 380 + 16
let area_y = 16
let area_width = 1920 - 380 - 32
let area_height = 1080 - 32

let area_center_x = area_x + area_width / 2
let area_center_y = area_y + area_height / 2


let zoom_double_tap_timer: number

export function _update(delta: number) {


    if (kb.wasReleased(GameAction.ATTACK)) {
        add_map(existing_polygons)
        set_next_scene = 'simulate'
    }

    if (kb.wasReleased(GameAction.ZOOM)) {
        if (zoom_double_tap_timer > 0) {
            zoom_double_tap_timer = 0
            camera.zoom = 1
        } else {
            zoom_double_tap_timer = 0.5
        }
    }

    zoom_double_tap_timer -= delta / 1000


    if (drag.wheel) {

        const world_before = screenToWorld(cursor.x, cursor.y)

        let zoom_factor = 1.1

        if (drag.wheel < 0) {
            camera.zoom *= zoom_factor
        } else {
            camera.zoom /= zoom_factor
        }


        camera.zoom = Math.min(Math.max(camera.zoom, MIN_ZOOM), MAX_ZOOM)

        camera.x = cursor.x - area_center_x - world_before.x * camera.zoom
        camera.y = cursor.y - area_center_y - world_before.y * camera.zoom
    }

    if (drag.is_hovering) {
        cursor.x = drag.is_hovering[0]
        cursor.y = drag.is_hovering[1]
        cursor.world_x = screenToWorld(cursor.x, cursor.y).x
        cursor.world_y = screenToWorld(cursor.x, cursor.y).y


        if (cursor.is_panning) {
            let dx = cursor.x - cursor.prev_x
            let dy = cursor.y - cursor.prev_y

            camera.x += dx
            camera.y += dy
        }

    }

    if (drag.is_just_down) {
        if (drag.down_button === 1 || kb.isDown(GameAction.JUMP)) {
            g.canvas.style.cursor = 'grabbing'
            cursor.is_panning = true
        }

        if (drag.down_button === 0 && !cursor.is_panning) {
            place_point()
        }
        if (drag.down_button === 2) {
            drawing_points = []
        }
    }

    if (drag.is_up) {
        g.canvas.style.cursor = 'default'
        cursor.is_panning = false
    }

    cursor.prev_x = cursor.x
    cursor.prev_y = cursor.y


    error_drawing_lines_timer -= delta / 1000


    let world_cursor = { x: cursor.world_x, y: cursor.world_y }
    is_hovering_on_close_point = findSegmentAtPoint(drawing_points, world_cursor, FindSegmentThreshold) === 0

    kb.update()
    drag.update(delta)
}

function place_point() {
    let p = {x: cursor.world_x, y:cursor.world_y}

    if (drawing_points.length === 0) {
        for (let i = 0; i < existing_polygons.length; i++) {
            let e_poly = existing_polygons[i]

            if (findSegmentAtPoint(e_poly.points, p, FindSegmentThreshold) !== -1) {
                existing_polygons.splice(i, 1)
                return
            }
        }
    }



    let c_edge = findSegmentAtPoint(drawing_points, p, FindSegmentThreshold)
    if (c_edge === 0) {
        try  {
            let poly = buildPoly(drawing_points)
            existing_polygons.push(poly)

            drawing_points = []
        } catch (e) {
            error_drawing_lines_timer = 0.5
        }
        return
    }

    if (c_edge !== -1) {
        error_drawing_lines_timer = 0.5
        return
    }

    if (drawing_points.length > 2) {
        try {
            buildPoly([...drawing_points, p])
        } catch {
            error_drawing_lines_timer = 0.5
            return
        }
    }

    drawing_points.push(p)
}

export function _render() {

    g.clear()

   
    g.begin_shapes()

    g.stroke_rect(8, 16, area_x - 24, area_height, Color.white, 8)
    g.stroke_rect(area_x, 16, area_width, area_height, Color.white, 8)

    g.end_shapes()

    grid()
    d_cursor()



}

function d_cursor() {
    g.begin_stencil()


    g.translate(area_x + area_width / 2, area_y + area_height / 2)
    g.begin_shapes()
    g.fill_rect(-area_width / 2, -area_height / 2, area_width, area_height, Color.white)
    g.end_shapes()


    g.begin_stencil_bg()

    g.translate(area_x + area_width / 2 + camera.x, area_y + area_height / 2 + camera.y)
    g.scale(camera.zoom, camera.zoom)

    let size = 16
    g.begin_shapes()
    g.draw_line(cursor.world_x - size / camera.zoom, cursor.world_y, cursor.world_x + size / camera.zoom, cursor.world_y, size / 4 / camera.zoom, Color.white)
    g.draw_line(cursor.world_x, cursor.world_y - size / camera.zoom, cursor.world_x, cursor.world_y + size / camera.zoom, size / 4 / camera.zoom, Color.white)

    g.end_shapes()
    g.end_stencil()

    g.translate(0, 0)
    g.scale(1, 1)
}

function grid() {

    let viewport_width = area_width / camera.zoom
    let viewport_height = area_height / camera.zoom
    let view_left = -viewport_width / 2 - camera.x / camera.zoom
    let view_top = -viewport_height / 2 - camera.y / camera.zoom

    let GRID_SIZE = 32
    let start_x = Math.floor(view_left / GRID_SIZE) * GRID_SIZE
    let start_y = Math.floor(view_top / GRID_SIZE) * GRID_SIZE
    let end_x = start_x + viewport_width + GRID_SIZE
    let end_y = start_y + viewport_height + GRID_SIZE

    let color = Color.grey
    color.a = 20

    g.begin_stencil()

    g.begin_shapes()
    g.fill_rect(area_x, area_y, area_width, area_height, Color.white)
    g.end_shapes()



    g.begin_stencil_bg()

    g.translate(area_x + area_width / 2 + camera.x, 2 + area_y + area_height / 2  + camera.y)
    g.scale(camera.zoom, camera.zoom)
    g.begin_shapes()

    for (let x = start_x; x < end_x; x += GRID_SIZE) {
        g.draw_line(x, start_y, x, end_y, 2, color)
    }

    for (let y = start_y; y < end_y; y += GRID_SIZE) {
        g.draw_line(start_x, y, end_x, y, 2, color)
    }

    g.draw_line(view_left, 0, end_x, 0, 4 / camera.zoom, Color.white)
    g.draw_line(0, view_top, 0, end_y, 4 / camera.zoom, Color.white)


    let drawing_lines_color = Color.grey

    if (error_drawing_lines_timer % 0.2 > 0.1) {
        drawing_lines_color = Color.red
    }

    if (drawing_points.length > 0) {
        let p = drawing_points[0]


        for (let i = 1; i < drawing_points.length; i++) {
            let p2 = drawing_points[i]

            g.draw_line(p.x, p.y, p2.x, p2.y, 8 / camera.zoom, drawing_lines_color)
            p = p2
        }

        g.draw_line(p.x, p.y, cursor.world_x, cursor.world_y, 4 / camera.zoom, Color.white)
    }

    if (is_hovering_on_close_point) {
        let p = drawing_points[0]
        g.fill_rect(p.x - 10, p.y - 10, 20, 20, Color.red)
    }

    for (let polygon of existing_polygons) {
        draw_polygon(polygon)
    }


    g.end_shapes()
    g.end_stencil()

    g.translate(0, 0)
    g.scale(1, 1)
}


export function draw_polygon(polygon: Poly) {

    let p = polygon.points[0]
    let p2

    for (let i = 1; i < polygon.points.length; i++) {

        p2 = polygon.points[i]

        g.draw_line(p.x, p.y, p2.x, p2.y, 8 / camera.zoom, Color.white)

        p = p2
    }

    p2 = polygon.points[0]
    g.draw_line(p.x, p.y, p2.x, p2.y, 8 / camera.zoom, Color.white)
}


export function _destroy() {
    kb.destroy()
}

let set_next_scene: SceneName | undefined = undefined
export function next_scene() {
    return set_next_scene
}