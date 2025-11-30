import { mat2, matMulVec, polarRotation2x2, type Mat2 } from "./math/mat2";
import type { Poly } from "./math/polygon";
import { add, distance, mulScalar, rotateVec2, sub, vec2, type Vec2 } from "./math/vec2";
import { Particle, type InputController } from "./xpbd";

const SUBSTEPS = 3
const CONSTRAINT_ITERATIONS = 5

const GRAVITY_Y = 3000


const VEL_IS_GROUNDED_Y_THRESHOLD = 60
const C_PENETRATION_CONTACT_THRESHOLD = 60


interface XPBDConstraint {
    project(dt: number): void
    perStep?: (dt: number) => void
    resetLambda?: () => void
}

export class Cluster {

    particles: Particle[]
    is_grounded: boolean = false
    angularVelocity: number = 0

    _has_floor_contact: boolean = false


    constructor(particles: Particle[]) {
        this.particles = particles
    }


    computeCOM() {
        // compute current CM
        let sumW = 0;
        let cm = vec2(0,0);
        for (let p of this.particles) {
            cm = mulScalar(add(cm, p.xPred), p.invMass)
            sumW += p.invMass;
        }
        cm = mulScalar(cm, 1 / sumW)
        return { xcom: cm, sumW }
    }

    computeVCOM(dt: number) {
        // compute current CM
        let sumW = 0;
        let vcom = vec2(0,0);
        for (let p of this.particles) {
            if (p.invMass === 0) continue
            let vc = mulScalar(sub(p.xPred, p.x), 1 / dt)
            vcom = mulScalar(add(vcom, vc), p.invMass) 
            sumW += p.invMass;
        }
        vcom = mulScalar(vcom, 1 / sumW)
        return { vcom, sumW }
    }




    update_is_grounded(dt: number) {
        let vCOMy = this.computeVCOM(dt).vcom.y

        const velThreshold = VEL_IS_GROUNDED_Y_THRESHOLD

        console.log(this._has_floor_contact, vCOMy)
        if (this._has_floor_contact && Math.abs(vCOMy) < velThreshold) {
            this.is_grounded = true
        } else {
            this.is_grounded = false
        }

        this._has_floor_contact = false
    }

}

export class FloorConstraint implements XPBDConstraint {
    cluster: Cluster
    floorY: number;          // y-coordinate of the floor
    compliance: number;      // optional softness

    constructor(cluster: Cluster, floorY: number, compliance: number) {
        this.cluster = cluster
        this.floorY = floorY;
        this.compliance = compliance;
    }

    project(dt: number) {
        const alpha = this.compliance / (dt * dt);

        let _contact = false
        for (let p of this.cluster.particles) {
            if (p.invMass === 0) continue;

            // if particle is below the floor
            const C = this.floorY - p.xPred.y;


            if (C < C_PENETRATION_CONTACT_THRESHOLD) {
                _contact = true
            }

            if (C >= 0) continue; // above floor, nothing to do


            // XPBD correction: move particle along normal (0,1)
            const deltaLambda = C / (p.invMass + alpha); 
            p.xPred.y += deltaLambda; 
        }

        this.cluster._has_floor_contact = _contact
    }
}



export class DistanceConstraint implements XPBDConstraint {
    i: number;                // index of first particle
    j: number;                // index of second particle
    restLength: number;
    compliance: number;       // XPBD compliance
    lambda: number = 0;       // accumulated Lagrange multiplier

    cluster: Cluster

    constructor(cluster: Cluster, i: number, j: number, restLength?: number, compliance = 0) {
        this.cluster = cluster
        this.i = i;
        this.j = j;
        this.restLength = restLength ?? distance(cluster.particles[i].xPred, cluster.particles[j].xPred);
        this.compliance = compliance;
    }

    project(dt: number) {
        let cl = this.cluster

        const pi = cl.particles[this.i];
        const pj = cl.particles[this.j];

        const w1 = pi.invMass;
        const w2 = pj.invMass;
        if (w1 + w2 === 0) return; // both static

        // 1. current vector between particles
        const dx = sub(pj.xPred, pi.xPred);
        const dist = Math.hypot(dx.x, dx.y);
        if (dist === 0) return; // avoid division by zero

        const n = { x: dx.x / dist, y: dx.y / dist }; // normalized direction

        // 2. constraint C = |pj - pi| - restLength
        const C = dist - this.restLength;

        // 3. XPBD scalar lambda update
        const alpha = this.compliance / (dt * dt);
        const deltaLambda = (-C - alpha * this.lambda) / (w1 + w2 + alpha);
        this.lambda += deltaLambda;

        // 4. apply corrections along n
        const correction = mulScalar(n, deltaLambda);
        if (w1 > 0) pi.xPred = sub(pi.xPred, mulScalar(correction, w1));
        if (w2 > 0) pj.xPred = add(pj.xPred, mulScalar(correction, w2));
    }

    resetLambda() {
        this.lambda = 0
    }
}


export class ShapeMatchingClusterConstraint implements XPBDConstraint {
    cluster: Cluster
    restShapePositions: Vec2[]; // rest positions (constant)
    cmRest: Vec2;               // precomputed center of rest shape
    N: number;
    compliance: number;         // cluster compliance
    stiffness: number;          // optional blending factor 0..1
    lambda: number

    R!: Mat2

    input_stop_rotate: boolean

    constructor(cluster: Cluster, compliance: number, stiffness: number) {
        this.cluster = cluster
        let xbar0 = this.cluster.computeCOM().xcom
        this.restShapePositions = cluster.particles.map(p => sub(p.x, xbar0));
        this.N = cluster.particles.length;
        this.compliance = compliance;
        this.stiffness = stiffness;
        this.lambda = 0

        // precompute rest center
        let sum = { x: 0, y: 0 };
        for (let r of this.restShapePositions) { sum.x += r.x; sum.y += r.y; }
        this.cmRest = { x: sum.x / this.N, y: sum.y / this.N };

        this.input_stop_rotate = false
    }

    computeBestRotation(): void {
        let cl = this.cluster
        let cm = this.cluster.computeCOM().xcom
        // compute A = sum m * (x~) * (q~)^T
        // A is 2x2
        let A11 = 0, A12 = 0, A21 = 0, A22 = 0;
        for (let i = 0; i < this.N; i++) {
            const w = cl.particles[i].invMass
            const x_tilde = sub(cl.particles[i].xPred, cm);
            const q_tilde = sub(this.restShapePositions[i], this.cmRest)
            A11 += w * x_tilde.x * q_tilde.x;
            A12 += w * x_tilde.x * q_tilde.y;
            A21 += w * x_tilde.y * q_tilde.x;
            A22 += w * x_tilde.y * q_tilde.y;
        }
        const A = mat2(A11, A12, A21, A22);
        // polar rotation
        const R = polarRotation2x2(A);
        this.R = R;
    }

    applyRotation(qtilde_i: Vec2): Vec2 {
        return matMulVec(this.R, qtilde_i)
    }

    project(dt: number) {

        let cl = this.cluster
        let cm = this.cluster.computeCOM().xcom
        // 2. Compute rotation matrix R that best aligns rest offsets to current offsets
        this.computeBestRotation();

        // 3. Compute per-particle goal positions using rotation + current CM
        const goals: Vec2[] = [];
        for (let i = 0; i < this.N; i++) {
            const q_i = sub(this.restShapePositions[i], this.cmRest)
            let rotated = this.applyRotation(q_i)
            goals.push(add(rotated, cm));
        }

        // 4. Compute residuals r_i = xPred_i - goal_i
        const residuals: Vec2[] = [];
        for (let i = 0; i < this.N; i++) {
            residuals.push(sub(cl.particles[i].xPred, goals[i]));
        }

        // 5. Compute XPBD-style scalar lambda for the cluster
        let C = 0;
        let sumWGrad = 0;
        for (let i = 0; i < this.N; i++) {
            const r = residuals[i];
            C += cl.particles[i].invMass * (r.x * r.x + r.y * r.y);
            sumWGrad += cl.particles[i].invMass
        }


        const alpha = this.compliance / (dt*dt);
        const dlambda = - (C + alpha * this.lambda) / (sumWGrad + alpha);
        this.lambda += dlambda

        // 6. Apply per-particle corrections
        for (let i = 0; i < this.N; i++) {
            const p = cl.particles[i];
            const delta = mulScalar(residuals[i], dlambda * p.invMass * this.stiffness);
            p.xPred = add(p.xPred, delta);
        }
    }

    resetLambda() {
        this.lambda = 0
    }

}



export class Simulator2D_XPBD {

    clusters: Cluster[] = []
    particles: Particle[] = []
    constraints: XPBDConstraint[] = []
    gravity: Vec2 = { x: 0, y: GRAVITY_Y }
    subSteps: number = SUBSTEPS

    constraintIterations: number = CONSTRAINT_ITERATIONS

    addCluster(c: Cluster) {
        this.clusters.push(c)
    }

    addParticle(x: number, y: number, mass: number = 1) {
        this.particles.push(new Particle(x, y, mass))
        return this.particles[this.particles.length - 1]
    }

    addConstraint(c: XPBDConstraint) {
        this.constraints.push(c)
    }

    step(dt: number) {

        console.log(this.clusters[0].is_grounded)
        for (let c of this.constraints) {
            c.resetLambda?.()
        }

        const sdt = dt / this.subSteps

        for (let s = 0; s < this.subSteps; s++) {

            for (let p of this.particles) {
                if (p.invMass === 0) continue

                const vel = sub(p.x, p.prev)
                p.xPred = add(add(p.x, vel), mulScalar(this.gravity, sdt * sdt))
            }

            for (let c of this.constraints) c.perStep?.(sdt)
            for (let iter = 0; iter < this.constraintIterations; iter++) {
                for (let c of this.constraints) c.project(sdt)
            }


            for (let c of this.clusters) {
                c.update_is_grounded(sdt)
            }

            for (let p of this.particles) {
                if (p.invMass === 0) continue;
                p.prev = { ...p.x };
                p.v = mulScalar(sub(p.xPred, p.x), 1 / sdt);
                p.x = { ...p.xPred };
            }
        }
    }
}


function createTire(sim: Simulator2D_XPBD, center: Vec2, radius: number, particleCount: number, particleMass = 1, input: InputController) {



    const distance_compliance = 1e-4
    let tireParticles = []
    const angleStep = (2 * Math.PI) / particleCount;

    // Step 1: Add particles in a circle
    for (let i = 0; i < particleCount; i++) {
        const angle = i * angleStep;
        const x = center.x + radius * Math.cos(angle);
        const y = center.y + radius * Math.sin(angle);
        tireParticles.push(sim.addParticle(x, y, particleMass))
    }

    let tireCluster = new Cluster(tireParticles)
    sim.addCluster(tireCluster)

    let shape_compliance = 0.05
    let shape_stiffness = 0.8
    sim.addConstraint(new ShapeMatchingClusterConstraint(tireCluster, shape_compliance, shape_stiffness))
    let tire_stiffness = 1
    let tire_compliance = 1
    sim.addConstraint(new TireConstraint(tireCluster, input, tire_compliance,  tire_stiffness))

    // Step 2: Connect neighboring particles with distance constraints
    for (let i = 0; i < particleCount; i++) {
        const j = (i + 1) % particleCount; // wrap around
        sim.addConstraint(new DistanceConstraint(tireCluster, i, j, undefined, distance_compliance))
    }

    // Step 3: Optional cross-links for extra stiffness (connect particles across the circle)
    const crossLinkStep = Math.floor(particleCount / 2);
    for (let i = 0; i < particleCount; i++) {
        const j = (i + crossLinkStep) % particleCount;
        sim.addConstraint(new DistanceConstraint(tireCluster, i, j, undefined, distance_compliance))
    }

    let jump_compliance = 0
    let jump_stiffness = 1
    let jump_opts = {
        jumpStrength: 1000,
        jumpBuffer: .5,
        coyoteTime: .5
    }

    let floor_compliance = 1e-8
    sim.addConstraint(new FloorConstraint(tireCluster, 500, floor_compliance))

    sim.addConstraint(new JumpConstraint(tireCluster, input, jump_compliance, jump_stiffness, jump_opts))
}

    let TIRE_RADIUS = 80
export function demoSim(polygons: Poly[], input: InputController) {

    let sim = new Simulator2D_XPBD()

    createTire(sim, { x: 0, y: -500 }, TIRE_RADIUS, 10, 1, input)


    return sim
}

export type JumpConstraintOptions = {
    jumpStrength: number
    jumpBuffer: number
    coyoteTime: number
}

export const Default_Jump_Opts = {
    jumpStrength: 500,
    JUMP_BUFFER: .5,
    COYOTE_TIME: .5
}

export class JumpConstraint implements XPBDConstraint {
    cluster: Cluster
    input: InputController
    compliance: number
    stiffness: number

    lambdaJump: number

    jumpStart: Vec2 | null

    jumpBufferTimer: number
    coyoteTimer: number
    jumpActive: boolean

    COYOTE_TIME: number
    JUMP_BUFFER: number

    JUMP_STRENGTH: number
    JUMP_COMPLIANCE: number


    /* Variable Jump */

    jumpHoldTimer: number

    MAX_JUMP_HOLD: number
    GRAVITY: number
    HOLD_GRAVITY_FACTOR: number


    constructor(cluster: Cluster, input: InputController, jumpCompliance: number, stiffness: number, opts: Partial<JumpConstraintOptions> = {}) {
        this.cluster = cluster
        this.input = input
        this.compliance = jumpCompliance;
        this.stiffness = stiffness;


        this.lambdaJump = 0;

        this.jumpActive = false;
        this.jumpStart = null;
        this.jumpBufferTimer = 0
        this.coyoteTimer = 0

        this.JUMP_COMPLIANCE = jumpCompliance;
        this.JUMP_STRENGTH = opts.jumpStrength ?? Default_Jump_Opts.jumpStrength;

        this.JUMP_BUFFER = opts.jumpBuffer ?? Default_Jump_Opts.JUMP_BUFFER;
        this.COYOTE_TIME = opts.coyoteTime ?? Default_Jump_Opts.COYOTE_TIME


        this.GRAVITY = GRAVITY_Y * 0.0003

        this.jumpHoldTimer = 0

        this.MAX_JUMP_HOLD = .3
        this.HOLD_GRAVITY_FACTOR = .3



    }

    handleInput(dt: number) {
        // --- Jump buffering ---
        if (this.input.jump_pressed) this.jumpBufferTimer = this.JUMP_BUFFER;
        else this.jumpBufferTimer -= dt;

        // --- Coyote timer ---
        if (this.cluster.is_grounded) this.coyoteTimer = this.COYOTE_TIME;
        else this.coyoteTimer -= dt;
    }

    perStep(dt: number) {
        this.handleInput(dt)
        if (this.jumpBufferTimer > 0 && this.coyoteTimer > 0) {
            this.doJump(dt)
        }

        if (!this.jumpActive) {
            return
        }

        if (!this.input.jump_held) {
            this.jumpActive = false;
            return
        }

        this.jumpHoldTimer += dt;

    }

    project(dt: number) {
        this.variableJump(dt)
    }

    variableJump(dt: number) {

        if (!this.jumpActive || !this.input.jump_held) {
            return
        }
        if (this.jumpHoldTimer >= this.MAX_JUMP_HOLD) return

        const scale = (1 - this.HOLD_GRAVITY_FACTOR) * this.GRAVITY * dt

        for (let p of this.cluster.particles) {
            p.xPred.y -= scale
        }
    }


    doJump(dt: number) {
        let cl = this.cluster
        const particles = cl.particles;

        let { xcom, sumW } = cl.computeCOM()

        // ----- target upward displacement -----
        const deltaY = this.JUMP_STRENGTH * dt;

        if (!this.jumpStart) {
            this.jumpStart = { ...xcom };
        }

        // Constraint (2D, nUp = -Y): (xcom.y - jumpStart.y) - deltaY = 0
        const C = (this.jumpStart.y - xcom.y) - deltaY;

        // Grad = ∂C/∂xi = invMass/wsum * nUp
        let denom = 0;
        for (let p of particles) {
            denom += p.invMass * (1 / sumW) * (1 / sumW);
        }

        const alpha = this.JUMP_COMPLIANCE / (dt * dt);
        const dlambda = -(C + alpha * this.lambdaJump) / (denom + alpha);
        this.lambdaJump += dlambda;

        // Apply projected corrections
        for (let p of particles) {
            const coeff = p.invMass * dlambda * (1 / sumW);
            p.xPred.y -= coeff; // projected along -Y axis
        }

        this.jumpActive = true;
        this.jumpStart = null;
        this.jumpBufferTimer = 0;
        this.coyoteTimer = 0;

        this.jumpHoldTimer = 0
    }

    resetLambda() {
        this.lambdaJump = 0
    }

}


export class TireConstraint implements XPBDConstraint {
    cluster: Cluster
    input: InputController
    angularVelocity: number
    theta: number
    compliance: number
    stiffness: number

    constructor(cluster: Cluster, input: InputController, compliance: number, stiffness: number) {
        this.cluster = cluster
        this.input = input
        this.angularVelocity = 0
        this.theta = 0

        this.compliance = compliance;
        this.stiffness = stiffness;
    }


    applyGroundTorque(dt: number) {
        const NORMAL_TORQUE = 1e-2
        const STRONG_TORQUE = 8;

        // flip direction -> stronger damping
        if (Math.sign(this.angularVelocity) !== Math.sign(this.input.x) && this.input.x !== 0) {
            this.angularVelocity = 0
            this.angularVelocity += this.input.x * STRONG_TORQUE * dt;
        } else {
            this.angularVelocity += this.input.x * NORMAL_TORQUE * dt;
        }

        // damping when no input
        if (this.input.x === 0) {
            const DAMPING = 0.95;
            this.angularVelocity *= Math.pow(DAMPING, dt * 60);
            this.angularVelocity = 0
        }

        // clamp angular velocity
        const MAX_SPIN = 0.016;
        this.angularVelocity = Math.max(-MAX_SPIN, Math.min(MAX_SPIN, this.angularVelocity));
    }

    applyTangentialRotation(dt: number) {

        let cl = this.cluster
        let { xcom } = this.cluster.computeCOM()

        const theta = this.angularVelocity * dt
        if (theta === 0) { return }


        for (let p of cl.particles) {
            const r = sub(p.xPred, xcom)

            const rRot = rotateVec2(r, theta)

            const goal = add(xcom, rRot)

            const alpha = 0.8;  // 0..1
            p.xPred = add(mulScalar(p.xPred, 1 - alpha), mulScalar(goal, alpha));

        }


        const radius = TIRE_RADIUS
        const linearVel = this.angularVelocity * radius;
        for (let p of this.cluster.particles) p.xPred.x += linearVel * dt;


        let frictionCompliance = 0
        const alpha = frictionCompliance / (dt * dt);

        // For each particle in contact with the floor
        const contactParticles = cl.particles.filter(p =>
            sub(p.xPred, xcom).y > 0
        );

        for (let p of contactParticles) {
            // Constraint: stop tangential motion relative to floor
            const C = p.xPred.x - p.prev.x; // displacement along tangent since last step
            const deltaLambda = -C / (p.invMass + alpha);

            // Optional: clamp based on Coulomb friction
            const normalForce = p.m
            const maxFriction = normalForce * dt;
            const correction = Math.max(-maxFriction, Math.min(maxFriction, deltaLambda));

            p.xPred.x += correction;
        }
        

    }



    applyFriction(_dt: number) {

        let cl = this.cluster

        /*
        for (let p of this.particles) {
            let vx = (p.xPred.x - p.x.x) / dt;     // tangential vel
            vx *= 0.999; // 0=no sliding, 1=no friction

            p.xPred.x = p.x.x + vx * dt;
        }
            */


        for (let p of cl.particles) {
            const dx = p.xPred.x - p.x.x;  // tangential displacement
            const dxAbs = Math.abs(dx);

            const staticLimit = 0.002;
            const dynamicCoeff = 0.9994;

            // STATIC friction region
            if (dxAbs < staticLimit) {
                p.xPred.x = p.x.x;
            }
            // DYNAMIC friction region
            else {
                p.xPred.x = p.x.x + dx * dynamicCoeff;
            }
        }



    }


    perStep(dt: number) {
        this.applyGroundTorque(dt)
    }

    project(dt: number) {
        this.applyTangentialRotation(dt)
        this.applyFriction(dt)
    }
}
