import { rnd_float, type RNG } from "./random";
import { type XYWH } from "../util";
import { Line, Matrix, Vec2 } from "./vec2_old";

export type RigidOptions = {
    mass: number,
    damping: number,
    max_speed: number,
    max_force: number
}

export type Rigid = {
    opts: RigidOptions,
    force: Vec2,
    xy: Vec2,
    dx: Vec2
}

export function make_rigid(xy: Vec2, opts: RigidOptions): Rigid {
    return {
        opts,
        force: Vec2.zero,
        xy,
        dx: Vec2.zero
    }
}

function rigid_update(body: Rigid, delta: number) {
    let { opts: { damping, mass, max_speed } } = body

    let { force } = body

    let a = force.scale(1/mass)

    body.dx = body.dx.add(a.scale(delta / 1000))
    body.dx = body.dx.scale(damping)
    body.dx = body.dx.truncate(max_speed)

    body.xy = body.xy.add(body.dx.scale(delta / 1000))
}

export type RigidBody = {
    update(delta: number): void
    add_force(v: Vec2): void
    add_steering_force(v: Vec2): void
    add_impulse(v: Vec2): void
    position: Vec2
    velocity: Vec2
    heading: Vec2
    side: Vec2
    matrix: Matrix
    max_speed: number
}

export function rigid_body(xy: Vec2, opts: RigidOptions): RigidBody {
    let body = make_rigid(xy, opts)

    function update(delta: number) {
        rigid_update(body, delta)

        body.force = Vec2.zero

        m_left = opts.max_force
    }

    let s_factor = 0.08
    let smoothed_force = Vec2.zero
    let m_left = opts.max_force
    function add_steering_force(f: Vec2) {
        smoothed_force = smoothed_force.scale(1 - s_factor).add(f.scale(s_factor))
        f = smoothed_force.truncate(m_left)
        m_left -= f.length

        body.force = body.force.add(f)
    }

    function add_force(f: Vec2) {
        body.force = body.force.add(f)
    }

    function add_impulse(impulse: Vec2) {
        body.dx = body.dx.add(impulse)
    }

    return {
        update,
        add_steering_force,
        add_force,
        add_impulse,
        get position() {
            return body.xy
        },
        get velocity() {
            return body.dx
        },
        get heading() {
            return this.velocity.clamp_epsilon().normalize ?? Vec2.unit
        },
        get side() {
            return this.heading.perpendicular
        },
        get matrix() {
            return Matrix.rotate_matrix(this.heading, this.side, this.position)
        },
        get max_speed() {
            return opts.max_speed
        },
        set max_speed(v: number) {
            opts.max_speed = v
        }
    }
}

export type Behavior = [(body: RigidBody) => Vec2, number]

export type SteerBehaviors = {
    body: RigidBody
    update(delta: number): void
    set_bs(bs: Behavior[]): void
    get_bs(): Behavior[]
    add_applied_force(f: Vec2): void
}

/* https://github.com/wangchen/Programming-Game-AI-by-Example-src/tree/master/Buckland_Chapter3-Steering%20Behaviors */
export function steer_behaviours(xy: Vec2, opts: RigidOptions, bs: Behavior[]): SteerBehaviors {

    let applied_force = Vec2.zero
    let body = rigid_body(xy, opts)

    function update(delta: number) {

        let t_weight = bs.map(_ => _[1]).reduce((a, b) => a + b, 0)
        for (let [steer, weight] of bs) {
            let desired_vel = steer(body)
            let steering =
                desired_vel.sub(body.velocity)
                    .scale(opts.max_force / opts.max_speed)
            body.add_steering_force(steering.scale(weight / t_weight))
        }

        body.add_force(applied_force)
        applied_force = Vec2.zero

        body.update(delta)
    }


    return {
        body,
        update,
        set_bs(new_bs: Behavior[]) {
            bs = new_bs
        },
        get_bs() {
            return bs
        },
        add_applied_force(f: Vec2) {
            applied_force = applied_force.add(f)
        }
    }
}


export const b_no_steer = (_body: RigidBody) => Vec2.zero
export const b_orbit_steer = (target: { position: Vec2 }, radius = 100) => 
    (body: RigidBody) => orbit_steer(body.position, target.position, radius, body.max_speed)

export const b_wander_steer = (jitter: number, r: number, distance: number, random: RNG) => {
    let v_wander = Vec2.zero
    return (body: RigidBody) => {
        if (random() < 0.1) {
            v_wander = Vec2.zero
        }
        return wander_steer(body.matrix, v_wander, jitter, r, distance, random)
    }
}

export const b_separation_steer = (group: { group: Vec2[]} ) => 
    (body: RigidBody) => separation_steer(body.position, group.group, body.max_speed)

export const b_avoid_circle_steer = (target: { position: Vec2 }, radius = 120, zero_angle = Math.PI / 2) => 
    (body: RigidBody) => avoid_circle_steer(body.position, target.position, body.max_speed, zero_angle, radius)

export const b_flee_steer = (target: { position: Vec2 }, zero_angle: number, slowing_distance = 100) => 
    (body: RigidBody) => flee_steer(body.position, target.position, body.max_speed, zero_angle, slowing_distance)
export const b_arrive_steer = (target: { position: Vec2 }, slowing_distance = 100) => 
    (body: RigidBody) => arrive_steer(body.position, target.position, body.max_speed, slowing_distance)
export const b_chase_steer = (target: { position: Vec2 }) => 
    (body: RigidBody) => chase_steer(body.position, target.position, body.max_speed)
export const b_pursuit_steer = (target: { position: Vec2, velocity: Vec2 }, c = 1) => 
    (body: RigidBody) => pursuit_steer(body.position, target.position, target.velocity, body.max_speed, c)


export const b_path_follow_steer = (path: Line[], c = 1, r = 10) => 
    (body: RigidBody) => path_follow_steer(body.position, body.velocity, body.max_speed, c, path, r)



export const b_wall_avoid_steer = (length: number, walls: { walls: Line[] }) =>
(body: RigidBody) => wall_avoid_steer(body.matrix, length, walls.walls)




function orbit_steer(position: Vec2, target: Vec2, radius: number, max_speed: number) {

    let target_offset = position.sub(target)
    let out = target_offset.scale(target_offset.length < radius ? 1 : -1).normalize ?? Vec2.zero

    let n = target_offset.perpendicular.normalize
    return (n ?? Vec2.zero).add(out).scale(max_speed / 2)
}

function separation_steer(position: Vec2, group: Vec2[], max_speed: number) {
    let res = Vec2.zero

    for (let neighbor of group) {
        let to_agent = position.sub(neighbor)

        if (to_agent.length === 0) {
            continue
        }

        res = res.add(to_agent.normalize!.scale(1 / (to_agent.length ?? 0.1)))

    }

    return res.scale(max_speed)
}

function avoid_circle_steer(position: Vec2, target: Vec2, max_speed: number, zero_angle: number, radius: number) {
    return flee_steer(position, target, max_speed, zero_angle, radius)
}

function flee_steer(position: Vec2, target: Vec2, max_speed: number, zero_angle: number, slowing_distance: number) {
    let target_offset = position.sub(target)
    let distance = target_offset.length
    if (distance > slowing_distance) {
        return Vec2.zero
    }

    if (target_offset.length === 0) {
        target_offset = target_offset.add_angle(zero_angle)
    }
    return target_offset.normalize?.scale(max_speed) ?? Vec2.zero
}

function arrive_steer(position: Vec2, target: Vec2, max_speed: number, slowing_distance: number) {
    let target_offset = target.sub(position)
    let distance = target_offset.length
    if (distance < 20) {
        return Vec2.zero
    }

    let ramped_speed = max_speed * (distance / slowing_distance)
    let clipped_speed = Math.min(ramped_speed, max_speed)
    let desired_velocity = target_offset.scale(clipped_speed / distance)
    return desired_velocity
}

function chase_steer(position: Vec2, target: Vec2, max_speed: number) {
    return (target.sub(position).normalize ?? Vec2.zero).scale(max_speed)
}


/* https://github.com/a327ex/SNKRX/blob/master/engine/game/steering.lua#L188 */
function wander_steer(position: Matrix, wander_target: Vec2, jitter: number, r: number, distance: number, random: RNG) {

    let x = wander_target.x + rnd_float(-1, 1, random) * jitter
    let y = wander_target.y + rnd_float(-1, 1, random) * jitter
    let t = Vec2.make(x, y).normalize ?? Vec2.zero
    let transform = t.scale(r).add(Vec2.make(distance, 0))
    return position.mul_vec2(transform).sub(position.matrix_translate)
}


function wall_avoid_steer(position: Matrix, length: number, walls: Line[]) {
    let orig = position.matrix_translate
    let heading = position.matrix_forward
    //let side = position.matrix_side

    let fs = []

    fs.push(heading.scale(length).add(orig))
    fs.push(heading.add_angle(-Math.PI * 0.25).scale(length / 2).add(orig))
    fs.push(heading.add_angle( Math.PI * 0.25).scale(length / 2).add(orig))


    let steering_force

    let closest_dist,
    closest_wall,
    closest_point


    for (let f of fs) {
        for (let line of walls) {

            let res = line.intersects(new Line(orig, f))

            if (res) {
                let [dist, point] = res

                if (!closest_dist || dist < closest_dist) {
                    closest_dist = dist
                    closest_wall = line
                    closest_point = point
                }
            }
        }

        if (closest_wall) {
            let overshoot = f.sub(closest_point!)
            steering_force = closest_wall.normal?.scale(overshoot.length)
        }
    }
    steering_force = steering_force?.scale(30)
    return steering_force ?? Vec2.zero
}


function pursuit_steer(position: Vec2, target: Vec2, target_velocity: Vec2, max_speed: number, c: number) {
    let D = position.distance(target)
    let T = D * c
    return chase_steer(position, target.add(target_velocity.scale(T)), max_speed)
}


function path_follow_steer(position: Vec2, velocity: Vec2, max_speed: number, c: number, path: Line[], r: number) {
    let T = c
    let fu = position.add(velocity.scale(T))

    let nearest = closestPointOnSegments(fu.x, fu.y, path.map(_ => _.xywh))

    if (nearest === null) {
        return Vec2.zero
    }

    let v = Vec2.make(nearest.x, nearest.y).project_to(fu)

    if (v.length < r) {
        return Vec2.zero
    }

    let l = path[nearest.i]

    v = v.add(Vec2.from_angle(l.angle))

    return chase_steer(position, v, max_speed)
}


function closestPointOnSegments(px: number, py: number, segments: XYWH[]) {
  let closest = null;
  let minDistSq = Infinity;

  let i = 0
  for (const [ax, ay, bx, by] of segments) {
    const abx = bx - ax;
    const aby = by - ay;
    const apx = px - ax;
    const apy = py - ay;

    const abLenSq = abx * abx + aby * aby;
    let t = (apx * abx + apy * aby) / abLenSq;

    // Clamp t to the segment [0, 1]
    t = Math.max(0, Math.min(1, t));

    // Closest point on the segment
    const cx = ax + t * abx;
    const cy = ay + t * aby;

    const dx = px - cx;
    const dy = py - cy;
    const distSq = dx * dx + dy * dy;

    if (distSq < minDistSq) {
      minDistSq = distSq;
      closest = { x: cx, y: cy, i };
    }
    i++
  }

  return closest;
}
