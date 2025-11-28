import './style.css'
import { Loop } from "./loop_input"
import { g } from './webgl/gl_init'
import Content from './content'
import * as editor from './editor'
import * as simulate from './simulate'


type Scene = {
    _init(): void
    _update(delta: number): void
    _render(): void
    _after_render?: () => void
    _destroy?: () => void
    next_scene(): SceneName | undefined
}

const default_scene = {
    _init() {},
    _update(_delta: number) {},
    _render() {},
    next_scene() { return undefined }
}

let current_scene: Scene
let next_scene: Scene

function switch_to_scene(scene: Scene) {
    next_scene._destroy?.()
    next_scene = scene
}

let Scenes: Record<string, Scene> = {
    'editor': editor,
    'simulate': simulate
} as const

export type SceneName = keyof typeof Scenes;


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

    let next = current_scene.next_scene()

    if (next !== undefined) {
        switch_to_scene(Scenes[next])
    }
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
    g.canvas.classList.add('interactive')
    $.appendChild(g.canvas)
    el.appendChild($)

    await Content.load()

    g.load_bg(Content.bg)
    g.load_sheet(Content.spritesheet)

    _init()

    Loop(_update, _render, _after_render)
}


main(document.getElementById('app')!)