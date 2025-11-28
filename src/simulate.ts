import { GameAction, InputController } from "./keyboard";
import type { SceneName } from "./main";
import { g } from "./webgl/gl_init";
import { demoTire, type Simulator2D } from "./xpbd";

let kb: InputController
let sim: Simulator2D

const trackPointss = [
    [{ x: 0, y: 500 }, { x: 1200, y: 500 },],
    [{ x: 0, y: 300 }, { x: 500, y: 500 }, { x: 0, y: 350 }],
    [{ x: 10, y: 0 }, { x: 10, y: 200 }],
    [{x: 400, y: 200}, { x: 700, y: 200}]
];



export function _init() {
    kb = new InputController()
    let dt = demoTire(trackPointss, {
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
    sim = dt.sim
}

export function _update(delta: number) {

    sim.step(delta/1000)
    kb.update()
}


export function _render() {

    g.clear()

    /*
    if (DEBUG) {
        for (let p of sim.particles) {
            g.
            cx.fillRect(p.x.x / 3, p.x.y / 3, 3, 3)
        }
    }


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

    g.begin_render()

    g.draw(0, 0, 32, 32, 0, 110, false)
    let ccx = 0, cy = 0
    for (let p of sim.particles) {
        ccx += p.x.x
        cy += p.x.y
    }
    ccx /= 6
    cy /= 6

    for (let i = 0; i < 6; i++) {
        let p1 = sim.particles[i].x
        let p2 = sim.particles[i === 5 ? 0 : i + 1].x
        let p3 = { x: ccx, y: cy }

        let u: [number, number, number] = [0, 64, 32]
        let v: [number, number, number] = [210, 210, 210 + 64]

        g.draw_tri([p1.x/3, p2.x/3, p3.x/3], [p1.y/3, p2.y/3, p3.y/3], u, v)

    }
    g.end_render()

    g.flush_to_screen()

}


let set_next_scene: SceneName | undefined = undefined
export function next_scene() {
    return set_next_scene
}