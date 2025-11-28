import './style.css'
import { Loop } from "./loop_input"
import { g } from './webgl/gl_init'
import Content from './content'
import * as editor from './editor'

type Scene = {
    _init(): void
    _update(delta: number): void
    _render(): void
    _after_render?: () => void
}

const default_scene = {
    _init() {},
    _update(_delta: number) {},
    _render() {}
}

let current_scene: Scene
let next_scene: Scene

function switch_to_scene(scene: Scene) {
    next_scene = scene
}

function _init() {

    current_scene = default_scene
    next_scene = current_scene

    switch_to_scene(editor)
}

function _update(delta: number) {

    if (next_scene !== current_scene) {
        current_scene = next_scene
        current_scene._init()
    }

    current_scene._update(delta)
}


function _render() {
    current_scene._render()
}



function _after_render() {
    current_scene._after_render?.()
}

async function main(el: HTMLElement) {

    let $ = document.createElement('div')
    $.classList.add('content')
    g.canvas.classList.add('pixelated')
    $.appendChild(g.canvas)
    el.appendChild($)

    await Content.load()

    g.load_bg(Content.bg)
    g.load_sheet(Content.spritesheet)

    _init()

    Loop(_update, _render, _after_render)
}


main(document.getElementById('app')!)