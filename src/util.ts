
export type XY = [number, number]
export type XYWH = [number, number, number, number]


export function on_interval(t: number, interval: number, delta: number, offset = 0) {
    let last = Math.floor((t - offset - delta) / interval)
    let next = Math.floor((t - offset) / interval)
    return last < next
}

export function lerp_angle(a: number, b: number, t: number) {
    let d = b - a
    if (d > Math.PI) d -= 2 * Math.PI
    if (d < -Math.PI) d += 2 * Math.PI
    return a + d * t
}

export function lerp_angle2(a: number, b: number, t: number) {
    // Normalize angles to [0, 2π] range
    a = ((a % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    b = ((b % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    
    let d = b - a;
    
    // Take the shortest path
    if (d > Math.PI) d -= 2 * Math.PI;
    if (d < -Math.PI) d += 2 * Math.PI;
    
    const result = a + d * t;
    
    // Optional: normalize result to [0, 2π]
    return ((result % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
}



export function lerp(a: number, b: number, t: number) {
    return a + (b - a) * t
}

export function appr(a: number, b: number, t: number) {
    if (a < b) {
        return Math.min(a + t, b)
    } else if (a > b) {
        return Math.max(a - t, b)
    } else {
        return a
    }
}


export function ease(t: number): number {
    return t * t * (3 - 2 * t)
}

export function box_intersect(a: XYWH, b: XYWH) {
    let [ax, ay, aw, ah] = a
    let [bx, by, bw, bh] = b

    return ax < bx + bw && ax + aw > bx && ay < by + bh && ay + ah > by
}


