export class Spring {

    x: number
    k: number
    d: number

    target_x: number
    v: number

    constructor(k = 100, d = 10) {

        this.k = k
        this.d = d

        this.x = 0
        this.target_x = this.x
        this.v = 0
    }


    update(delta: number) {
        let a = -this.k * (this.x - this.target_x) - this.d * this.v
        this.v = this.v + a * (delta / 1000)
        this.x = this.x + this.v * (delta / 1000)
    }

    pull(f: number, k = this.k, d = this.d) {
        this.k = k
        this.d = d
        this.x = this.x + f

    }
}