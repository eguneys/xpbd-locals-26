export type Vec2 = { x: number, y: number }

export function len2(a: Vec2) { return a.x * a.x + a.y * a.y; }
export function length(a: Vec2) { return Math.hypot(a.x, a.y); }
export function distance(a: Vec2, b: Vec2) { return length(sub(a, b)); }
export function normalize(a: Vec2) { const l = length(a); return { x: a.x / l, y: a.y / l };  }

export function vec2(x=0,y=0){ return {x, y}; }
export function add(a: Vec2,b: Vec2){ return {x: a.x+b.x, y: a.y+b.y}; }
export function sub(a: Vec2,b: Vec2){ return {x: a.x-b.x, y: a.y-b.y}; }
export function mulScalar(v: Vec2,s: number){ return {x: v.x*s, y: v.y*s}; }
export function dot(a: Vec2,b: Vec2){ return a.x*b.x + a.y*b.y; }

/** Rotate a 2D vector `v` by angle `theta` (in radians) */
export function rotateVec2(v: Vec2, theta: number): Vec2 {
    const cosT = Math.cos(theta);
    const sinT = Math.sin(theta);
    return {
        x: v.x * cosT - v.y * sinT,
        y: v.x * sinT + v.y * cosT
    };
}