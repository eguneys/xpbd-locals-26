import './style.css'
import { Loop } from "./loop_input"
import { add, canJump, demoTire, Simulator2D, TireController2D } from './xpbd'
import { GameAction, InputController } from './keyboard'
import { appr } from './util'


let kb: InputController
let sim: Simulator2D
let tire: TireController2D
function _init() {
    kb = new InputController()
    let dt = demoTire()
    sim = dt.sim
    tire = dt.tire
}

let t = 0
function _update(delta: number) {
    sim.step(delta/1000)
    tire.applyControl(delta/1000)

    t += delta
    if (kb.isDown(GameAction.RIGHT)) {
        tire.desiredVelocity.x = appr(tire.desiredVelocity.x, 100, delta)
        tire.angularVelocity = tire.desiredVelocity.x / tire.radius
    } else if (kb.isDown(GameAction.LEFT)) {
        tire.desiredVelocity.x = appr(tire.desiredVelocity.x, -100, delta)
        tire.angularVelocity = tire.desiredVelocity.x / tire.radius
    } else {
        tire.desiredVelocity.x = 0
        tire.angularVelocity = 0
    }


    if (kb.wasPressed(GameAction.UP)) {
        if (canJump(tire.cluster, 400)) {
            const jumpSpeed = -700;
            for (let p of tire.cluster.particles) p.v = add(p.v, { x: 0, y: jumpSpeed });
        }
    }

    kb.update()
}
function _render() {

    cx.fillStyle = 'black'
    cx.fillRect(0, 0, 320, 180)

    cx.fillStyle = 'white'
    for (let p of sim.particles) {
        cx.fillRect(p.x.x/3, p.x.y/3, 3, 3)
    }
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
    $.appendChild(canvas)
    el.appendChild($)

    _init()

    Loop(_update, _render, _after_render)
}


main(document.getElementById('app')!)