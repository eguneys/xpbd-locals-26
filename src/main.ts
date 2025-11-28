import './style.css'
import { Loop } from "./loop_input"
import { demoTire, Simulator2D, TireController2DPlus } from './xpbd'
import { GameAction, InputController } from './keyboard'
import { g } from './webgl/gl_init'
import Content from './content'


let kb: InputController
let sim: Simulator2D
let tire: TireController2DPlus

const platforms = [
    { y: 400, x1: 0, x2: 1200 },
    { y: 300, x1: 0, x2: 500 }
];

function _init() {
    kb = new InputController()
    let dt = demoTire(platforms, {
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
    tire = dt.tire
}

let t = 0
function _update(delta: number) {

    sim.step(delta/1000)
    t += delta
    kb.update()
}

let DEBUG = false
function _render() {

    cx.clearRect(0, 0, 320, 180)

    if (DEBUG) {
        cx.fillStyle = 'white'
        for (let p of sim.particles) {
            cx.fillRect(p.x.x / 3, p.x.y / 3, 3, 3)
        }
    }


    cx.strokeStyle = 'white'
    cx.lineWidth = 1
    for (let p of platforms) {
        cx.beginPath()
        cx.moveTo(p.x1/3, p.y/3)
        cx.lineTo(p.x2/3, p.y/3)
        cx.stroke()
    }


    g.clear()

    /*
    g.begin_shapes()
    g.shape_rect(0, 0, 320, 180, Color.black)
    g.end_shapes()
    */

    g.begin_render()

    g.draw(0, 0, 32, 32, 0, 110, false)
    let ccx = 0, cy = 0
    for (let p of tire.cluster.particles) {
        ccx += p.x.x
        cy += p.x.y
    }
    ccx /= 6
    cy /= 6

    for (let i = 0; i < 6; i++) {
        let p1 = tire.cluster.particles[i].x
        let p2 = tire.cluster.particles[i === 5 ? 0 : i + 1].x
        let p3 = { x: ccx, y: cy }

        let u: [number, number, number] = [0, 64, 32]
        let v: [number, number, number] = [210, 210, 210 + 64]

        g.draw_tri([p1.x/3, p2.x/3, p3.x/3], [p1.y/3, p2.y/3, p3.y/3], u, v)

    }
    g.end_render()

    g.flush_to_screen()
}

function _after_render() {

}

let cx: CanvasRenderingContext2D

function init_canvas() {
    let canvas = document.createElement('canvas')
    canvas.width = 320
    canvas.height = 180
    cx = canvas.getContext('2d')!
    cx.imageSmoothingEnabled = false
    return canvas
}

async function main(el: HTMLElement) {

    let canvas = init_canvas()
    let $ = document.createElement('div')
    $.classList.add('content')
    canvas.classList.add('pixelated')
    g.canvas.classList.add('pixelated')
    $.appendChild(g.canvas)
    $.appendChild(canvas)
    el.appendChild($)

    await Content.load()

    g.load_bg(Content.bg)
    g.load_sheet(Content.spritesheet)

    _init()

    Loop(_update, _render, _after_render)
}


main(document.getElementById('app')!)