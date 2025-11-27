export const PI2 = Math.PI * 2

export type Easing = (t: number) => number

export const linear: Easing = t => t
export const sine_in: Easing = t => t === 0 ? 0 : t === 1 ? 1 : 1 - Math.cos(t * PI2)
export const sine_out: Easing = t => t === 0 ? 0 : t === 1 ? 1 : 1 - Math.sin(t * PI2)
export const sine_in_out: Easing = t => 
    t === 0 ? 0 : t === 1 ? 1 : 
        t < 0.5 ? 0.5 * Math.sin((t * 2) * PI2) :
            -0.5 * Math.cos((t * 2 - 1) * PI2) + 1

export const quad_in: Easing = t => t * t
export const quad_out: Easing = t => -t * (t - 2)


export const cubic_in: Easing = t => t * t * t
export const cubic_out: Easing = t => {
    t = t - 1
    return t * t * t + 1
}