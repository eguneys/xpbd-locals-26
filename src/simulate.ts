import { GameAction, InputController } from "./keyboard";
import type { SceneName } from "./main";
import { get_map } from "./maps_store";
import type { Poly } from "./math/polygon";
import { Color } from "./webgl/color";
import { g } from "./webgl/gl_init";
import type { Particle } from "./xpbd";
import { Simulator2D_XPBD, demoSim } from './xpbd_2'

let kb: InputController
let sim: Simulator2D_XPBD

/*
const trackPointss = [
    [{ x: 0, y: 500 }, { x: 1200, y: 500 },],
    [{ x: 0, y: 300 }, { x: 500, y: 500 }, { x: 0, y: 350 }],
    [{ x: 10, y: 0 }, { x: 10, y: 200 }],
    [{x: 400, y: 200}, { x: 700, y: 200}]
];
*/



export function _init() {
    kb = new InputController()
    sim = demoSim(get_map(), {
        get jump_pressed() {
            return kb.wasPressed(GameAction.JUMP)
        },
        get jump_held() {
            return kb.isDown(GameAction.JUMP)
        },
        get x() {
            return kb.isDown(GameAction.RIGHT) ? 1 : (kb.isDown(GameAction.LEFT) ? -1 : 0)
        }
    })
}

export function _update(delta: number) {

    if (kb.wasReleased(GameAction.ATTACK)) {
        set_next_scene = 'editor'
    }

    sim.step(delta/1000)
    kb.update()
}


export function _render() {

    g.clear()

    g.translate(1920/ 2, 1080 / 2)

    /*
    g.begin_shapes()
    for (let poly of sim.polygons) {
        draw_polygon(poly)
    }

    g.end_shapes()
    */
    /*

    cx.strokeStyle = 'white'
    cx.lineWidth = 1

    for (let trackPoints of trackPointss) {
        let p = trackPoints[0]

        let off = -8
        cx.beginPath()
        cx.moveTo(p.x / 3 - off, p.y / 3 + off)
        for (let i = 1; i < trackPoints.length; i++) {
            let p = trackPoints[i]
            cx.lineTo(p.x / 3 - off, p.y / 3 + off)
        }
        cx.stroke()
    }
        */



    /*
    g.begin_shapes()
    g.shape_rect(0, 0, 320, 180, Color.black)
    g.end_shapes()
    */



    g.begin_shapes()

    for (let p of sim.particles) {
        draw_particle(p)
    }
    g.end_shapes()

    /*
    g.begin_render()

    let cx = 0, cy = 0
    for (let p of sim.particles) {
        cx += p.x.x
        cy += p.x.y
    }
    cx /= 6
    cy /= 6

    for (let i = 0; i < 6; i++) {
        let p1 = sim.particles[i].x
        let p2 = sim.particles[(i + 1) % 6].x
        let p3 = { x: cx, y: cy }

        let u: [number, number, number] = [0, 64, 32]
        let v: [number, number, number] = [210, 210, 210 + 64]

        g.draw_tri([p1.x, p2.x, p3.x], [p1.y, p2.y, p3.y], u, v)

    }
    g.end_render()

    */

}

export function draw_particle(p: Particle) {

    g.draw_line(p.x.x - 4, p.x.y - 4, p.x.x + 8, p.x.y + 8, 4, Color.grey)
}


export function draw_polygon(polygon: Poly) {

    let p = polygon.points[0]
    let p2

    for (let i = 1; i < polygon.points.length; i++) {

        p2 = polygon.points[i]

        g.draw_line(p.x, p.y, p2.x, p2.y, 8, Color.white)

        p = p2
    }

    p2 = polygon.points[0]
    g.draw_line(p.x, p.y, p2.x, p2.y, 8, Color.white)
}




export function _destroy() {
    kb.destroy()
}



let set_next_scene: SceneName | undefined = undefined
export function next_scene() {
    let res =  set_next_scene
    if (res !== undefined){
        set_next_scene = undefined
        return res
    }
}