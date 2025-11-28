import type { Poly } from "./math/polygon";

let map: Poly[] = []

export function add_map(poly: Poly[]) {
    map = poly

    localStorage.setItem('l.map', JSON.stringify(map))
}

export function get_map() {
    let res = localStorage.getItem('l.map')
    if (res !== null) {
        map = JSON.parse(res)
    }
    return map
}