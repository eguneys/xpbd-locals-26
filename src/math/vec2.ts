export const EPSILON = 1e-4

export type XY = [number, number]
export type XYWH = [number, number, number, number]

export class Vec2 {
    

    static rotate_point(x: number, y: number, theta: number, cx: number, cy: number) {
        let cos = Math.cos(theta)
        let sin = Math.sin(theta)

        let x2 = x - cx
        let y2 = y - cy

        let x3 = x2 * cos - y2 * sin
        let y3 = x2 * sin + y2 * cos

        x3 += cx
        y3 += cy

        return Vec2.make(x3, y3)
    }

    static from_angle = (n: number) =>
        new Vec2(Math.cos(n), Math.sin(n))

    static make = (x: number, y: number) => new Vec2(x, y)
    static xy = (xy: XY) => new Vec2(xy[0], xy[1])


    static get unit() { return new Vec2(1, 1) }
    static get zero() { return new Vec2(0, 0) }


    get xy(): XY {
        return [this.x, this.y]
    }

    get mul_inverse() {
        return new Vec2(1 / this.x, 1 / this.y)
    }

    get inverse() {
        return new Vec2(- this.x, - this.y)
    }

    get half() {
        return new Vec2(this.x / 2, this.y / 2)
    }

    get length_squared() {
        return this.x * this.x + this.y * this.y
    }

    get length() {
        return Math.sqrt(this.length_squared)
    }

    get normalize(): Vec2 | undefined {
        if (this.length !== 0) {
            return this.scale(1 / this.length)
        }
    }

    get perpendicular() {
        return new Vec2(-this.y, this.x)
    }

    get angle() {
        return Math.atan2(this.y, this.x)
    }

    readonly x: number
    readonly y: number

    constructor(x: number, y: number) {
        this.x = x
        this.y = y
    }

    clamp_epsilon(epsilon = EPSILON): Vec2 {
        return this.length < epsilon ? Vec2.zero : this
    }



    dot(v: Vec2) {
        return this.x * v.x + this.y * v.y
    }


    cross(v: Vec2) {
        return this.x * v.y - this.y * v.x
    }


    project_to(v: Vec2) {
        let lsq = v.length_squared
        let dp = this.dot(v)
        return Vec2.make(dp * v.x / lsq, dp * v.y / lsq)
    }


    distance(v: Vec2) {
        return this.sub(v).length
    }


    addy(dy: number) {
        return Vec2.make(this.x, this.y + dy)
    }

    add_angle(n: number) {
        return Vec2.from_angle(this.angle + n)
    }

    scale(n: number) {
        return Vec2.make(this.x * n, this.y * n)
    }


    add(v: Vec2) {
        return Vec2.make(this.x + v.x, this.y + v.y)
    }

    sub(v: Vec2) {
        return Vec2.make(this.x - v.x, this.y - v.y)
    }

    mul(v: Vec2) {
        return Vec2.make(this.x * v.x, this.y * v.y)
    }

    div(v: Vec2) {
        return Vec2.make(this.x / v.x, this.y / v.y)
    }


    truncate(max: number) {
        let length = this.length
        if (length > max) {
            return this.scale(max / length)
        }
        return this
    }
}


export class Matrix {

    static get identity() { return new Matrix(1, 0, 0, 1, 0, 0) }

    static get unit() { return Matrix.identity }

    static projection = (width: number, height: number) => {
        let b = 0,
        c = 0

        let a = 1 / width * 2,
        d = -1 / height * 2,
        tx = -1,
        ty = 1

        return new Matrix(a, b, c, d, tx, ty)
    }

    /* https://gamedev.stackexchange.com/questions/200784/how-to-move-an-enemy-to-sneak-up-on-the-player-from-behind-using-forces */
    static rotate_matrix = (heading: Vec2, side: Vec2, pos: Vec2) => {
        let a = heading.x,
            b = side.x,
            c = heading.y,
            d = side.y,
            tx = pos.x,
            ty = pos.y

            return new Matrix(a, b, c, d, tx, ty)
    }


    get matrix_forward() {
        return Vec2.make(this.a, this.c)
    }

    get matrix_side() {
        return Vec2.make(this.b, this.d)
    }

    get matrix_translate() {
        return Vec2.make(this.tx, this.ty)
    }


    get inverse() {
        let { a, b, c, d, tx, ty } = this

        let n = a * d - b * c

        let a1 = d / n,
        b1 = -b / n,
        c1 = -c / n,
        d1 = a / n,
        tx1 = (c * ty - d * tx) / n,
        ty1 = - (a * ty - b * tx) / n

        return new Matrix(a1, b1, c1, d1, tx1, ty1)
    }

    get array() {
        return [
            this.a, this.b, 0,
            this.c, this.d, 0,
            this.tx, this.ty, 1
        ]
    }

    readonly a: number
    readonly b: number
    readonly c: number
    readonly d: number
    readonly tx: number
    readonly ty: number

    constructor(
        a: number,
        b: number,
        c: number,
        d: number,
        tx: number,
        ty: number) {

            this.a = a
            this.b = b
            this.c = c
            this.d = d
            this.tx = tx
            this.ty = ty
        }


    rotate(r: number) {

        let cosa = Math.cos(r),
            sina = Math.sin(r)

        let a = this.a * cosa - this.b * sina,
            b = this.a * sina + this.b * cosa,
            c = this.c * cosa - this.d * sina,
            d = this.c * sina + this.d * cosa,
            tx = this.tx * cosa - this.ty * sina,
            ty = this.tx * sina + this.ty * cosa

            return new Matrix(a, b, c, d, tx, ty)
    }


    scale(x: number, y: number) {


        let a = this.a * x,
            b = this.b,
            c = this.c,
            d = this.d * y,
            tx = this.tx,
            ty = this.ty

            return new Matrix(a, b, c, d, tx, ty)

    }


    translate(x: number, y: number) {


        let a = this.a,
            b = this.b,
            c = this.c,
            d = this.d,
            tx = this.tx + x,
            ty = this.ty + y

            return new Matrix(a, b, c, d, tx, ty)
    }



    mul_vec2(v: Vec2) {


        let a = this.a,
            b = this.b,
            c = this.c,
            d = this.d,
            tx = this.tx,
            ty = this.ty

            let x = a * v.x + c * v.y + tx,
            y = b * v.x + d * v.y + ty

            return Vec2.make(x, y)
    }


    mul(m: Matrix) {


        let a = m.a * this.a + m.b + this.c,
            b = m.a * this.b + m.b + this.d,
            c = m.c * this.a + m.d + this.c,
            d = m.c * this.b + m.d + this.d,
            tx = m.tx * this.a + m.ty * this.c + this.tx,
            ty = m.tx * this.b + m.ty * this.d + this.ty

        return new Matrix(a, b, c, d, tx, ty)
    }
}


export class Circle {
    static make = (x: number, y: number, r: number) => new Circle(Vec2.make(x, y), r)


    static get unit() { return Circle.make(0, 0, 1) }

    get r() {
        return this.radius
    }

    o: Vec2
    radius: number

    constructor(o: Vec2, radius: number) {
        this.o = o
        this.radius = radius
    }


    scale(n: number) {
        return Circle.make(this.o.x, this.o.y, this.r * n)
    }

    translate(v: Vec2) {
        return new Circle(this.o.add(v), this.radius)
    }
}


export class Line {
    static make = (x: number, y: number, x2: number, y2: number) => new Line(Vec2.make(x, y), Vec2.make(x2, y2))


    get center() {
        return this.b.add(this.a).half
    }

    get parallel() {
        return this.b.sub(this.a).normalize
    }

    get normal() {
        return this.parallel?.perpendicular
    }

    get length() {
        return this.b.sub(this.a).length
    }

    get angle() {
        return this.b.sub(this.a).angle
    }

    get radius() {
        return this.length / 2
    }

    get xywh(): XYWH {
        return [this.a.x, this.a.y, this.b.x, this.b.y]
    }

    a: Vec2
    b: Vec2

    constructor(a: Vec2, b: Vec2) {
        this.a = a
        this.b = b
    }

    interpolate(dist: number) {
        let dd = this.b.sub(this.a)

        let len = dd.length
        let ratio = dist / len
        return this.a.add(dd.scale(ratio))
    }

    unsafe_segments(ns: number[], xs: number[]) {
        return ns.map((_, i) =>
        this.a.add(this.normal!.scale(xs[i]).add(this.parallel!.scale(this.length * _))))
    }

    extrude(thickness: number) {
        let perp = this.normal!.scale(thickness)

        let offset1 = Vec2.zero
        let offset2 = perp

        offset1 = perp.scale(0.5)
        offset2 = perp.scale(-0.5)

        return new Rectangle([
            this.a.add(offset1),
            this.a.add(offset2),
            this.b.add(offset2),
            this.b.add(offset1)
        ])
    }


    /* https://github.com/wangchen/Programming-Game-AI-by-Example-src/blob/master/Common/2D/geometry.h */
  intersects(cd: Line): [number, Vec2] | undefined {
    let { a, b } = this
    let { a: c, b: d } = cd
    let r_top = (a.y - c.y) * (d.x - c.x) - (a.x - c.x) * (d.y - c.y)
    let r_bot = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x)

    let s_top = (a.y - c.y) * (b.x - a.x) - (a.x - c.x) * (b.y - a.y)
    let s_bot = (b.x - a.x) * (d.y - c.y) - (b.y - a.y) * (d.x - c.x)

    if ((r_bot === 0) || (s_bot === 0)) {
      return undefined
    }
    let r = r_top / r_bot
    let s = s_top / s_bot

    if ((r > 0) && (r < 1) && (s > 0) && (s < 1)) {
      let dist = a.distance(b) * r
      let point = a.add(b.sub(a).scale(r))

      return [dist, point]
    }
  }
}


export class Rectangle {

    static make = (x: number, y: number, w: number, h: number) => 
        new Rectangle([
            Vec2.make(x, y),
            Vec2.make(x + w, y),
            Vec2.make(x + w, y + h),
            Vec2.make(x, y + h)])

    static get unit() { return Rectangle.make(0, 0, 1, 1) }

    get xywh(): XYWH {
        let { x, y, w, h } = this
        return [x, y, w, h]
    }


    get x1() { return this.vertices[0].x }
    get y1() { return this.vertices[0].y}
    get x2() { return this.vertices[2].x }
    get y2() { return this.vertices[2].y }

    get x() { return this.x1 }
    get y() { return this.y1 }
    get w() { return this.x2 - this.x1 }
    get h() { return this.y2 - this.y1 }


    get center() {
        return Vec2.make(this.x + this.w / 2, this.y + this.h / 2)
    }


    get vertex_data() {
        return new Float32Array(
            this.vertices.flatMap(_ => _.xy)
        )
    }

    get indices() {
        return new Uint16Array([0, 1, 2, 0, 2, 3])
    }

    larger(r: number) {
        return Rectangle.make(this.x - r / 2, this.y - r / 2, this.w + r, this.h + r)
    }

    smaller(r: number) {
        return Rectangle.make(this.x + r  / 2, this.y + r / 2, this.w - r, this.h - r)
    }


    vertices: Vec2[]

    constructor(vertices: Vec2[]) {
        this.vertices = vertices
    }


    transform(m: Matrix) {
        return new Rectangle(this.vertices.map(_ => m.mul_vec2(_)))
    }


    div(v: Vec2) {
        return new Rectangle(this.vertices.map(_ => _.div(v)))
    }
}