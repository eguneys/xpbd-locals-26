import { g } from "./webgl/gl_init";

export function _init() {

}

export function _update() {

}


export function _render() {

    g.clear()

    
    g.begin_render()
    g.draw(0, 0, 32, 32, 0, 110, false)
    g.end_render()

}