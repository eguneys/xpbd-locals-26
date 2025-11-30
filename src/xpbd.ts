// xpbd-shape-matching-2d.js
// Simple 2D XPBD-ish meshless shape matching demo
// Author: ChatGPT (adapted for user's request)

import { pointInPolygon, type Poly } from "./math/polygon"
import { add, dot, length, mulScalar, normalize, sub, vec2, type Vec2 } from "./math/vec2"


const SUBSTEPS = 30
const GRAVITY_Y = 8e5
const NB_ITERATIONS = 8
const SHAPE_COMPLIANCE = 0.0001
const SHAPE_STIFFNESS = 3.56


// ---- linear algebra helpers for 2D ----

type Mat2 = { a: number, b: number, c: number, d: number }
// 2x2 matrix stored as {a,b,c,d} = [[a,b],[c,d]]
function mat2(a: number,b: number,c: number,d: number){ return {a,b,c,d}; }
function matMulVec(m: Mat2,v: Vec2){ return { x: m.a*v.x + m.b*v.y, y: m.c*v.x + m.d*v.y }; }
function matMul(m1: Mat2,m2: Mat2){
  return {
    a: m1.a*m2.a + m1.b*m2.c,
    b: m1.a*m2.b + m1.b*m2.d,
    c: m1.c*m2.a + m1.d*m2.c,
    d: m1.c*m2.b + m1.d*m2.d
  };
}
function matTranspose(m: Mat2){ return mat2(m.a, m.c, m.b, m.d); }

// ---- 2x2 symmetric eigen-decomposition (for polar decomposition) ----
// returns { vals: [l1,l2], vecs: [v1, v2] } where each v is {x,y}
function eig2x2_sym(s11: number, s12: number, s22: number){
  // analytic eigenvalues
  const tr = s11 + s22;
  const det = s11*s22 - s12*s12;
  const disc = Math.max(0, tr*tr/4 - det); // (t/2)^2 - det
  const sqrtDisc = Math.sqrt(disc);
  const l1 = tr/2 + sqrtDisc;
  const l2 = tr/2 - sqrtDisc;

  // eigenvector for l1: (s12, l1 - s11) unless near-zero
  function eigenvec(lambda: number){
    let vx = s12;
    let vy = lambda - s11;
    if (Math.abs(vx) < 1e-9 && Math.abs(vy) < 1e-9){
      // fallback: use [1,0]
      vx = 1; vy = 0;
    }
    const norm = Math.hypot(vx, vy);
    return { x: vx / norm, y: vy / norm };
  }

  const v1 = eigenvec(l1);
  // ensure v2 is orthonormal
  const v2 = { x: -v1.y, y: v1.x };
  return { vals: [l1, l2], vecs: [v1, v2] };
}

// compute inverse square root of symmetric positive-definite 2x2 matrix S
// S = [[s11, s12],[s12, s22]] -> return S^{-1/2} as mat2
function invSqrtSym2x2(s11: number, s12: number, s22: number){
  const e = eig2x2_sym(s11, s12, s22);
  const l1 = Math.max(e.vals[0], 1e-12);
  const l2 = Math.max(e.vals[1], 1e-12);
  const v1 = e.vecs[0], v2 = e.vecs[1];

  const invSqrt1 = 1/Math.sqrt(l1);
  const invSqrt2 = 1/Math.sqrt(l2);

  // reconstruct: V diag(invSqrt) V^T
  const a = v1.x*invSqrt1*v1.x + v2.x*invSqrt2*v2.x;
  const b = v1.x*invSqrt1*v1.y + v2.x*invSqrt2*v2.y;
  const c = v1.y*invSqrt1*v1.x + v2.y*invSqrt2*v2.x;
  const d = v1.y*invSqrt1*v1.y + v2.y*invSqrt2*v2.y;
  return mat2(a,b,c,d);
}

// polar decomposition: given A (2x2), compute R (rotation) as R = A * (A^T A)^{-1/2}
function polarRotation2x2(A: Mat2){
  // compute S = A^T * A (symmetric)
  const At = matTranspose(A);
  const S = matMul(At, A); // 2x2 symmetric
  // S elements:
  const s11 = S.a, s12 = S.b, s22 = S.d;
  // compute (S)^{-1/2}
  const invSqrt = invSqrtSym2x2(s11, s12, s22);
  // R = A * invSqrt
  return matMul(A, invSqrt);
}

const TIRE_RADIUS = 60
const R_tire = TIRE_RADIUS
const N = 6
let ParticleRadius = 2 * Math.PI * R_tire / N / 2;


type CollisionContact = {
  penetration: number;
  normal: Vec2;
  contactPoint: Vec2;
  contactIsVertex: boolean;
  restitution: number;
  friction: number;
}


// ---- Particle and Cluster classes ----
class Particle {

    x: Vec2
    v: Vec2
    m: number
    invMass: number
    xPred: Vec2
    prev: Vec2

    radius = ParticleRadius

    collisionContact: CollisionContact | null

  constructor(x: number, y: number, mass=1){
    this.x = vec2(x,y);      // position
    this.prev = vec2(x, y) // previous position
    this.v = vec2(0,0);      // velocity
    this.m = mass;           // mass
    this.invMass = mass > 0 ? 1 / mass : 0;
    this.xPred = vec2(x,y);  // predicted position for the timestep
    this.collisionContact = null
  }
}

class ShapeCluster2D {

    particles: Particle[]
    N: number
    q: Vec2[]
    compliance: number
    stiffness: number
    masses: number[]
    qbar: Vec2
    qtilde: Vec2[]
    invM: Mat2


    xbar!: Vec2
    R!: Mat2
    T!: Mat2
    goals!: Vec2[]

    is_grounded: boolean


  // particles: array of Particle, restPositions: array matching length with local rest q_i
  // compliance: XPBD compliance parameter (higher => softer). Use 0 for hard.
  // stiffness: alternative convenience parameter [0..1] that blends correction.
  constructor(particles: Particle[], restPositions: Vec2[], compliance = 0, stiffness = 1.0){
    this.particles = particles;
    this.N = particles.length;
    this.q = restPositions.map(p => ({x: p.x, y: p.y})); // rest local coords
    this.compliance = compliance;
    this.stiffness = stiffness; // used to optionally scale final correction
    // precompute rest center and M = sum m * q~ * q~^T and its inverse
    this.masses = particles.map(p => p.m);
    //const Msum = particles.reduce((s,_,i) => s + particles[i].m, 0) || 1;
    // compute rest center
    let qbar = vec2(0,0);
    let massSum = 0;
    for(let i=0;i<this.N;i++){
      qbar = add(qbar, mulScalar(this.q[i], particles[i].m));
      massSum += particles[i].m;
    }
    qbar = mulScalar(qbar, 1/massSum);
    this.qbar = qbar;
    // store q~ (relative to qbar)
    this.qtilde = this.q.map(qi => sub(qi, qbar));
    // compute M = sum m * q~ * q~^T  (2x2)
    let M11=0,M12=0,M22=0;
    for(let i=0;i<this.N;i++){
      const m = particles[i].m;
      const r = this.qtilde[i];
      M11 += m * r.x * r.x;
      M12 += m * r.x * r.y;
      M22 += m * r.y * r.y;
    }
    // compute inverse of 2x2 symmetric matrix
    const det = M11*M22 - M12*M12;
    if (Math.abs(det) < 1e-12) {
      // degenerate => add small regularization
      M11 += 1e-6; M22 += 1e-6;
    }
    const invDet = 1 / (M11*M22 - M12*M12);
    this.invM = mat2(M22*invDet, -M12*invDet, -M12*invDet, M11*invDet);

    this.is_grounded = false
  }


  // compute center of mass of predicted positions
  computeCOMPred(){
    let com = vec2(0,0);
    let msum = 0;
    for(let i=0;i<this.N;i++){
      const p = this.particles[i];
      com = add(com, mulScalar(p.xPred, p.m));
      msum += p.m;
    }
    return mulScalar(com, 1/msum);
  }



  // compute best-fit rotation R and optionally affine T
  computeBestRotation(){
    // center-of-mass of predicted positions
    const xbar = this.computeCOMPred();
    this.xbar = xbar;
    // compute A = sum m * (x~) * (q~)^T
    // A is 2x2
    let A11=0,A12=0,A21=0,A22=0;
    for(let i=0;i<this.N;i++){
      const m = this.particles[i].m;
      const x_tilde = sub(this.particles[i].xPred, xbar);
      const q_tilde = this.qtilde[i];
      A11 += m * x_tilde.x * q_tilde.x;
      A12 += m * x_tilde.x * q_tilde.y;
      A21 += m * x_tilde.y * q_tilde.x;
      A22 += m * x_tilde.y * q_tilde.y;
    }
    const A = mat2(A11, A12, A21, A22);
    // polar rotation
    const R = polarRotation2x2(A);
    this.R = R;
    // optionally compute affine T = A * invM (not used for rigid)
    const T = matMul(A, this.invM);
    this.T = T;
  }

  // compute goal positions g_i = R * qtilde_i + xbar
  computeGoals(){
    this.goals = [];
    for(let i=0;i<this.N;i++){
      const gLocal = matMulVec(this.R, this.qtilde[i]);
      const g = add(gLocal, this.xbar);
      this.goals.push(g);
    }
  }

  // apply XPBD-like projection with compliance
  // dt: timestep, iterations: number of solver iterations (if you call cluster multiple times)
  project(dt: number){
    // compute rotation and goals from current xPred
    this.computeBestRotation();
    this.computeGoals();

    // We'll treat the cluster constraint as minimizing per-particle vector residuals r_i = xPred_i - g_i
    // XPBD-style scalar lambda for cluster isn't exact; instead we'll compute per-particle corrections
    // with compliance blended in a stable way:
    //
    // For each particle: Δx_i = - w_i / (Σ w_j + alpha/dt^2) * (r_i)
    //
    // This is a heuristic but matches XPBD behaviour when r is linearized and mass-w is used.
    //
    // compute sum of inverse masses weighted
    let sumW = 0;
    const invMasses = [];
    for(let i=0;i<this.N;i++){
      const w = this.particles[i].invMass;
      invMasses.push(w);
      sumW += w;
    }

    // alpha = compliance
    const alpha = this.compliance;
    const denom = sumW + alpha / (dt*dt);

    // apply vector corrections
    for(let i=0;i<this.N;i++){
      const p = this.particles[i];
      const r = sub(p.xPred, this.goals[i]); // residual
      // correction proportional to invMass
      const wi = p.invMass;
      const scale = wi / denom;
      const delta = mulScalar(r, -scale * this.stiffness); // stiffness blends toward full correction
      p.xPred = add(p.xPred, delta);
    }
  }


  computeClusterEnclosingCircle() {
    // center is COM
    const com = this.computeCOMPred();

    let maxDist = 0;
    for (let p of this.particles) {
        const dx = p.xPred.x - com.x;
        const dy = p.xPred.y - com.y;
        const d = Math.hypot(dx, dy) + (p.radius ?? 0);
        if (d > maxDist) maxDist = d;
    }

    return { center: com, radius: maxDist };
}

/*
  updateClusterGrounded(walls: Wall[]) {
    const hull = this.computeClusterEnclosingCircle();
    this.is_grounded = walls.some(wall => {
      // approximate: hull touches or slightly below floor
      const closestPoint = closestPointOnSegment(wall.A, wall.B, hull.center);
      return hull.center.y - hull.radius <= closestPoint.y + 1e-4;
    });
  }
    */


  /*
  updateClusterGrounded(walls: Wall[]) {
    const hull = this.computeClusterEnclosingCircle();
    const skin = 1e-2;

    this.is_grounded = walls.some(wall => {
      const A = wall.A;
      const B = wall.B;

      // wall direction & normal
      const dir = sub(B, A);
      const len = length(dir);
      if (len < 1e-8) return false;

      const n = normalize(vec2(-dir.y, dir.x)); // outward normal

      // signed distance from circle center to infinite line
      const d = dot(sub(hull.center, A), n);

      // Only consider if circle is within contact distance
      if (d < 0 || d > hull.radius + skin) {
        return false;
      }

      // Now check if the projected point lies on segment
      const t = dot(sub(hull.center, A), dir) / (len * len);

      // clamp t into slight tolerance
      if (t < -0.01 || t > 1.01) {
        return false;
      }

      // All conditions satisfied
      return true;
    });
  }
    */

}

// ---- Simulator: integrates, collision placeholder, shape clusters ----
class Simulator2D {

    particles: Particle[]
    clusters: ShapeCluster2D[]
    gravity: Vec2
    iterations: number
    polygons: Poly[]

    tireController!: TireController2DPlus


  constructor(){
    this.polygons = []
    this.particles = [];
    this.clusters = []; // array of ShapeCluster2D
    this.gravity = vec2(0, GRAVITY_Y); // y-down (pixels/sec^2) - tune to your world
    this.iterations = NB_ITERATIONS; // solver iterations per step
  }

  addPolygons(p: Poly[]){ this.polygons.push(...p); }
  addParticle(p: Particle){ this.particles.push(p); return p; }
  addCluster(cluster: ShapeCluster2D){ this.clusters.push(cluster); return cluster; }
  addTireController(tire_controller: TireController2DPlus) {
    this.tireController = tire_controller
  }

  step(dt: number) {
    const sdt = dt / SUBSTEPS

    for (let step = 0; step < SUBSTEPS; step++) {
      this.substep(sdt)
    }
  }

  substep(dt: number){

    let ctrl = this.tireController
    let cl = ctrl.cluster
    if (!cl.xbar) {
        cl.xbar = cl.computeCOMPred()
    }

    for (let p of this.particles) {
      const v = sub(p.x, p.prev)
      const accel = add(this.gravity, { x: 0, y: 0})
      p.xPred = add(p.x, v)
      p.xPred = add(p.xPred, mulScalar(accel, dt * dt))
    }

    ctrl.handleInput(dt)

    if (ctrl.jumpBufferTimer > 0 && ctrl.coyoteTimer > 0) {
        ctrl.doJump(dt)
    }




    ctrl.applyAirRotation(dt)
    ctrl.applyAirControl(dt)
    ctrl.applyAirDrag(dt)
    //ctrl.moveCOM(dt)
    ctrl.applyGroundTorque(dt)
    ctrl.applyTorque(dt)
    ctrl.variableJump(dt)


    // 2) initial collision pass (optional)
    for (let cl of this.clusters) {
      resolvePolygonCollisions(cl, this.polygons, dt)
    }

    // 3) shape-matching solver iterations (XPBD-like)
    for(let iter=0; iter<this.iterations; iter++){


      for(let cl of this.clusters){
          cl.project(dt);
      }

      for (let cl of this.clusters) {
        resolvePolygonCollisions(cl, this.polygons, dt)
      }


    }

    // 4) finalize positions and update velocities
    for(let p of this.particles){
      // v = (x_new - x_old) / dt
      p.v = mulScalar(sub(p.xPred, p.prev), 1/dt);
      p.prev = { ...p.x }
      p.x = { ... p.xPred };
    }
    console.log(this.particles[0].v)
  }
}

//type Platform = { x1: number, x2: number, y: number }

/*
// platforms: array of { y, x1, x2 }
// gravityY: simulator gravity y-acceleration (e.g. 1200)
// mu: friction coefficient (e.g. 0.8)
function resolveFlatCollisions(cl: ShapeCluster2D, platforms: Platform[], dt: number, gravityY: number, mu=0.8) {

    cl.is_grounded = false

  const eps = 1e-6;

  for (let p of cl.particles) {
    for (let plat of platforms) {
      // quick horizontal check
      if (p.xPred.x < plat.x1 || p.xPred.x > plat.x2) continue;

      // crossing test: was above (prev) and now below (pred)?
      // remember: +y is down. "above" means smaller y value.
      const wasAbove = (p.prev.y <= plat.y + eps);
      const nowBelow = (p.xPred.y > plat.y - eps);
      if (!(wasAbove && nowBelow)) continue;

      // compute penetration (how far past the platform surface we are)
      const penetration = p.xPred.y - plat.y; // positive when below
        if (penetration > 30) continue;
        if (penetration < -10) continue;

        cl.is_grounded = true

        if (penetration > 0) {
            // push particle up by the penetration amount (so its final y = plat.y)
            p.xPred.y -= penetration; // i.e. p.xPred.y = plat.y;

            // remove downward velocity (v.y positive = down)
            if (p.v.y > 0) {
                p.v.y = 0;
            }
        }

      // approximate normal force = mass * gravity (per-frame impulse limit we'll use)
      const normalForce = p.m * Math.abs(gravityY);

      // friction impulse along x (ground tangent). We clamp by mu * normalForce * dt.
      const vTang = p.v.x; // horizontal tangent with flat platform
      const desiredImpulse = -vTang * p.m; // impulse that would cancel tangential velocity
      const maxF = mu * normalForce * dt;
      const clampedImpulse = Math.max(-maxF, Math.min(maxF, desiredImpulse));

      // apply impulse (impulse / mass -> dv)
      p.v.x += clampedImpulse / p.m;

      // we've resolved this platform collision for this particle; break to avoid multiple resolves
      break;
    }
  }
}
  */


// ---- tiny example usage ----
function demo(polygons: Poly[]){
  const sim = new Simulator2D();

  sim.addPolygons(polygons)
  // build a small rectangular patch of particles (4 particles) as a cluster
  /*
  const particles = [];
  particles.push(new Particle(200, 360, 1)); // top-left
  particles.push(new Particle(240, 360, 1)); // top-right
  particles.push(new Particle(200, 400, 1)); // bottom-left
  particles.push(new Particle(240, 400, 1)); // bottom-right
  for(let p of particles) sim.addParticle(p);
  */

  const tireParticles = [];
  const N = 6; // circle particles
  const center = { x: -200, y: -400 }, radius = TIRE_RADIUS;
  for (let i = 0; i < N; i++) {
      const angle = i / N * 2 * Math.PI;
      const x = center.x + radius * Math.cos(angle);
      const y = center.y + radius * Math.sin(angle);
      const p = new Particle(x, y, 1);
      sim.addParticle(p);
      tireParticles.push(p);
  }



  // rest positions relative to initial center-of-mass (they should be local q_i)
  // here we use the initial positions as rest q's (in world coords) and cluster will compute qbar -> qtilde
  const restQ = tireParticles.map(p => ({x: p.x.x, y: p.x.y}));

  // create cluster: low compliance -> stiff object
  const cluster = new ShapeCluster2D(tireParticles, restQ, /*compliance=*/SHAPE_COMPLIANCE, /*stiffness=*/SHAPE_STIFFNESS);
  sim.addCluster(cluster);

  return sim;
}

// If run in a main loop, call e.g.:
// const sim = demo();
// setInterval(() => { sim.step(1/60); render(sim); }, 1000/60);

export { Particle, ShapeCluster2D, Simulator2D, demo };


// --- Tire controller ---

export type InputController = { 
    jump_pressed: boolean
    jump_held: boolean
    x: number
}

export class TireController2DPlus {

    cluster: ShapeCluster2D
    input: InputController

    angularVelocity: number
    jumpBufferTimer: number
    coyoteTimer: number
    jumpActive: boolean

    jumpHoldTimer: number

    JUMP_IMPULSE: number
    MAX_JUMP_HOLD: number
    HOLD_GRAVITY_FACTOR: number

    COYOTE_TIME: number
    JUMP_BUFFER: number

    MAX_ANGULAR: number

    AIR_CONTROL_ACCEL: number
    MAX_AIR_ANGULAR: number

    AIR_DRAG: number
    AIR_DRAG_LINEAR: number

    GROUND_FRICTION: number
    GRAVITY: number

    HORIZONTAL_SPEED: number

    restRadius: number

    constructor(cluster: ShapeCluster2D, input: InputController, radius: number) {
        this.cluster = cluster;
        this.input = input;

        this.restRadius = radius

        // Controller state
        this.angularVelocity = 0;

        // Jump variables
        this.jumpBufferTimer = 0;
        this.coyoteTimer = 0;
        this.jumpActive = false;
        this.jumpHoldTimer = 0;

        this.MAX_ANGULAR = 60;

        // Constants (tune these!)
        this.JUMP_IMPULSE = 3000;
        this.MAX_JUMP_HOLD = .2;         // seconds
        this.HOLD_GRAVITY_FACTOR = .5;   // reduces gravity while holding jump
        this.COYOTE_TIME = .5;           // seconds
        this.JUMP_BUFFER = .5;           // seconds

        this.AIR_CONTROL_ACCEL = 8;
        this.MAX_AIR_ANGULAR = 3;
        this.AIR_DRAG = 0.98;
        this.AIR_DRAG_LINEAR = 0.1

        this.GROUND_FRICTION = 1;
        this.GRAVITY = GRAVITY_Y;              // positive down

        this.HORIZONTAL_SPEED = 1000


    }

    handleInput(dt: number) {
        // --- Jump buffering ---
        if (this.input.jump_pressed) this.jumpBufferTimer = this.JUMP_BUFFER;
        else this.jumpBufferTimer -= dt;

        // --- Coyote timer ---
        if (this.cluster.is_grounded) this.coyoteTimer = this.COYOTE_TIME;
        else this.coyoteTimer -= dt;
    }

  applyAirControl(dt: number) {
    const cl = this.cluster;
    if (cl.is_grounded) return;

    const inputX = this.input.x;
    const airAccel = this.AIR_CONTROL_ACCEL * dt;

    cl.xbar.x += inputX * airAccel;
    for (let p of cl.particles) {
      p.xPred.x += inputX * airAccel;
    }
  }

    
  applyAirDrag(_dt: number) {
    const cl = this.cluster;
    if (cl.is_grounded) return;

    this.angularVelocity *= this.AIR_DRAG; // 0.98–0.995 typical

    for (let p of cl.particles) {
      const dx = p.xPred.x - p.x.x;
      const dy = p.xPred.y - p.x.y;
      p.xPred.x = p.x.x + dx * this.AIR_DRAG_LINEAR;
      p.xPred.y = p.x.y + dy * this.AIR_DRAG_LINEAR;
    }
  }


    doJump(dt: number) {
    const cl = this.cluster;

    // compute COM vertical "velocity" from predicted positions
    let vCOM = 0;
    let massSum = 0;
    for (let p of cl.particles) {
        const v = (p.xPred.y - p.x.y) / dt;  // derived velocity
        vCOM += v / p.invMass;
        massSum += 1 / p.invMass;
    }
    vCOM /= massSum;

    // desired jump impulse
    const netImpulse = this.JUMP_IMPULSE + Math.max(0, vCOM);

    // convert impulse to position-space change
    for (let p of cl.particles) {
        p.xPred.y -= netImpulse * dt * p.invMass; // XPBD-friendly
    }

    this.jumpActive = true;
    this.jumpHoldTimer = 0;
    this.jumpBufferTimer = 0;
    this.coyoteTimer = 0;
}


  variableJump(dt: number) {
    if (!this.jumpActive || !this.input.jump_held) {
      this.jumpActive = false;
      return;
    }

    this.jumpHoldTimer += dt;

    if (this.jumpHoldTimer < this.MAX_JUMP_HOLD) {

      // This acts as "cancel gravity" AND "add upward velocity"
      const impulse = this.JUMP_IMPULSE * (1 - this.HOLD_GRAVITY_FACTOR);

      for (const p of this.cluster.particles) {
        p.xPred.y -= impulse * dt;
      }
    }
  }

  applyAirRotation(dt: number) {
    const cl = this.cluster;
    if (cl.is_grounded) return;

    const scale = 0.3; // reduce torque effect in air

    for (let p of cl.particles) {
      const r = sub(p.xPred, cl.xbar);
      const rLen = Math.hypot(r.x, r.y);
      if (rLen > 0) {
        const tangent = { x: -r.y / rLen, y: r.x / rLen };
        const deltaPos = mulScalar(tangent, this.angularVelocity * rLen * dt * scale);
        p.xPred = add(p.xPred, deltaPos);
      }
    }
  }


    applyTorque(dt: number) {
      let cl = this.cluster

      let dx = 0, dy = 0;
      let n = cl.particles.length;

      // 1) rotate particles around COM
      for (let p of cl.particles) {
        const r = sub(p.xPred, cl.xbar);
        const rLen = Math.hypot(r.x, r.y);
        if (rLen > 0) {
          const tangential = { x: -r.y / rLen, y: r.x / rLen };
          const delta = mulScalar(tangential, this.angularVelocity * rLen * dt);

          p.xPred = add(p.xPred, delta);

          dx += delta.x;
          dy += delta.y;
        }
      }

      // 2) subtract mean delta to avoid COM drifting
      dx /= n;
      dy /= n;

      for (let p of cl.particles) {
        p.xPred.x -= dx;
        p.xPred.y -= dy;
      }



      const contactParticles = cl.particles.filter(
        //p => (p.xPred.y - cl.xbar.y) < 0
        p => p.collisionContact !== null
      );

      let grounded = contactParticles.length > 0
      this.cluster.is_grounded = grounded

      if (grounded) {
        const radius = 20;
        const linearVel = this.angularVelocity * radius;
        const deltaX = linearVel * dt;

        cl.xbar.x += deltaX;
        for (let p of cl.particles) {
          p.xPred.x += deltaX;
        }
      }



      if (grounded) {

        const mu = this.GROUND_FRICTION;

        let totalFriction = 0;

        for (let p of contactParticles) {

          // tangential slip velocity based on xPred
          const vTang = (p.xPred.x - p.prev.x) / dt;

          // friction "impulse" in position-space
          const frictionDelta = -vTang * mu * dt;

          p.xPred.x += frictionDelta;

          totalFriction += frictionDelta;
        }

        // prevent friction from dragging COM
        const correction = totalFriction / cl.particles.length;
        for (let p of cl.particles) {
          p.xPred.x -= correction;
        }
      }

    }

    applyGroundTorque(dt: number) {
      // TODO 
        //if (!this.cluster.is_grounded) return;

        let GROUND_TORQUE = 80; // tuning
        if (Math.sign(this.angularVelocity) !== Math.sign(this.input.x)) {
            GROUND_TORQUE = 120
            this.angularVelocity *= 0.001
        }
        this.angularVelocity += this.input.x * GROUND_TORQUE * dt;
        this.angularVelocity = Math.min(this.MAX_ANGULAR, Math.max(-this.MAX_ANGULAR, this.angularVelocity))

        if (this.input.x === 0) {
            this.angularVelocity *= 0.001
        }
    }


    moveCOM(dt: number) {
        const deltaCOM = mulScalar({x: this.input.x * this.HORIZONTAL_SPEED, y: 0}, dt);
        this.cluster.xbar = add(this.cluster.xbar, deltaCOM);
        for (let p of this.cluster.particles) p.xPred = add(p.xPred, deltaCOM);
    }


  applyRadialSprings(dt: number, stiffness = 50, damping = 0.9) {
    let cluster = this.cluster
    const com = cluster.computeCOMPred(); // current COM prediction
    for (let p of cluster.particles) {
      let r = sub(p.xPred, com);
      let rLen = Math.hypot(r.x, r.y);
      let restLen = this.restRadius;

      if (rLen === 0) continue;

      let dir = { x: r.x / rLen, y: r.y / rLen };
      let delta = rLen - restLen;

      // Hooke's law + simple damping
      let force = -stiffness * delta;

      // apply to velocity
      p.v.x += force * dir.x * dt;
      p.v.y += force * dir.y * dt;

      // optional damping along radial direction
      let radialVel = p.v.x * dir.x + p.v.y * dir.y;
      p.v.x -= radialVel * (1 - damping);
      p.v.y -= radialVel * (1 - damping);
    }
  }

}

function resolvePolygonCollisions(cluster: ShapeCluster2D, polygons: Poly[], dt: number) {
    for (let p of cluster.particles) {
      p.collisionContact = null
        for (let polygon of polygons) {
            let res = resolveParticlePolygon(p, polygon, { dt });

            if (res !== null) {
              p.collisionContact = res
            }
        }
    }
}


/**
 * resolveParticlePolygon - robust version
 *
 * Resolves collision between a particle and a polygon.
 * Works for convex and concave polygons, handles edge/vertex collisions,
 * automatically computes normals, and avoids false positives from far-away edges.
 */
function resolveParticlePolygon(
  particle: Particle,
  poly: Poly,
  options?: {
    dt?: number;
    restitution?: number;
    friction?: number;
    slop?: number;
    allowVertexCollisions?: boolean;
  }
): CollisionContact | null {
  const dt = options?.dt ?? 1.0;
  const restitution = options?.restitution ?? 0.0;
  const friction = options?.friction ?? 0.2;
  const slop = options?.slop ?? 1e-3;
  const allowVertex = options?.allowVertexCollisions ?? true;

  const verts = poly.points;
  const p = particle.xPred; // predicted position
  const r = particle.radius;

  let bestPenetration = -Infinity;
  let bestNormal: Vec2 | null = null;
  let bestContactPoint: Vec2 | null = null;
  let contactIsVertex = false;

  // Helper: closest point on segment AB to point P
  function closestPointOnSegment(A: Vec2, B: Vec2, P: Vec2) {
    const AB = sub(B, A);
    const t = Math.max(0, Math.min(1, dot(sub(P, A), AB) / dot(AB, AB)));
    return { point: add(A, mulScalar(AB, t)), t };
  }

  // --- 1) Edge collisions ---
  for (let i = 0; i < verts.length; i++) {
    const a = verts[i];
    const b = verts[(i + 1) % verts.length];

    const cp = closestPointOnSegment(a, b, p);
    const dVec = sub(p, cp.point);
    const dist2 = dot(dVec, dVec);

    if (dist2 < (r + slop) * (r + slop)) {
      const dist = Math.sqrt(dist2);
      const penetration = r - dist;
      if (penetration > (bestPenetration ?? -Infinity)) {
        bestPenetration = penetration;
        bestContactPoint = cp.point;
        bestNormal = dist > 1e-12 ? normalize(dVec) : { x: 0, y: 1 };
        contactIsVertex = false;
      }
    }
  }

  // --- 2) Vertex collisions ---
  if (allowVertex) {
    for (let i = 0; i < verts.length; i++) {
      const v = verts[i];
      const dVec = sub(p, v);
      const dist2 = dot(dVec, dVec);
      const rad2 = r * r;

      if (dist2 < rad2) {
        const dist = Math.sqrt(dist2);
        const penetration = r - dist;
        if (penetration > (bestPenetration ?? -Infinity)) {
          bestPenetration = penetration;
          bestNormal = dist > 1e-12 ? normalize(dVec) : { x: 0, y: 1 };
          bestContactPoint = v;
          contactIsVertex = true;
        }
      }
    }
  }

  // --- 3) Particle fully inside polygon ---
  if (!bestNormal && pointInPolygon(p, poly)) {
    let minDist = Infinity;
    //let closestEdge = 0;
    for (let i = 0; i < verts.length; i++) {
      const a = verts[i];
      const b = verts[(i + 1) % verts.length];
      const cp = closestPointOnSegment(a, b, p);
      const d = length(sub(p, cp.point));
      if (d < minDist) {
        minDist = d;
        bestContactPoint = cp.point;
        //closestEdge = i;
      }
    }
    if (bestContactPoint) {
      const dVec = sub(p, bestContactPoint);
      bestNormal = normalize(dVec);
      bestPenetration = r - minDist;
      contactIsVertex = false;
    }
  }

  // --- 4) No collision ---
  if (!bestNormal || bestPenetration <= slop) return null;

  // --- 5) Positional correction ---
  particle.xPred = add(particle.xPred, mulScalar(bestNormal, bestPenetration));

  // --- 6) XPBD-style velocity update (optional, commented out) ---
  const newVel = mulScalar(sub(particle.xPred, particle.prev), 1 / dt);
  const vn = mulScalar(bestNormal, dot(newVel, bestNormal));
  const vt = sub(newVel, vn);
  const vn_after = mulScalar(vn, -restitution);
  const vt_after = mulScalar(vt, Math.max(0, 1 - friction));
  const finalVel = add(vn_after, vt_after);
  particle.prev = sub(particle.xPred, mulScalar(finalVel, dt));

  console.log(dt)
  return {
    penetration: bestPenetration,
    normal: bestNormal,
    contactPoint: bestContactPoint!,
    contactIsVertex,
    restitution,
    friction,
  };
}



/*
function resolveParticleWallCollision(p: Particle, wall: Wall) {
  const A = wall.A;
  const B = wall.B;

  // 1️⃣ Closest point on segment
  const ABx = B.x - A.x;
  const ABy = B.y - A.y;
  const t = ((p.xPred.x - A.x) * ABx + (p.xPred.y - A.y) * ABy) / (ABx * ABx + ABy * ABy);
  const clampedT = Math.max(0, Math.min(1, t));
  const Cx = A.x + ABx * clampedT;
  const Cy = A.y + ABy * clampedT;

  // 2️⃣ Distance vector
  const rx = p.xPred.x - Cx;
  const ry = p.xPred.y - Cy;
  const dist = Math.hypot(rx, ry);

  if (dist >= p.radius) return undefined; // no collision

  const penetration = p.radius - dist;

  // 3️⃣ Compute outward normal using wall orientation
  let nx = -ABy / Math.hypot(ABx, ABy);
  let ny = ABx / Math.hypot(ABx, ABy);

  // check which side particle is on
  const dot = (p.xPred.x - A.x) * nx + (p.xPred.y - A.y) * ny;
  if (dot < 0) {
    nx = -nx;
    ny = -ny;
  }

  // 4️⃣ Push particle out along normal
  p.xPred.x += penetration * nx;
  p.xPred.y += penetration * ny;

  // 5️⃣ Velocity response (elastic + friction)
  const vn = p.v.x * nx + p.v.y * ny;
  p.v.x -= (1 + (wall.restitution ?? 0.8)) * vn * nx;
  p.v.y -= (1 + (wall.restitution ?? 0.8)) * vn * ny;

  // friction along tangent
  const tx = -ny;
  const ty = nx;
  const vt = p.v.x * tx + p.v.y * ty;
  const friction = wall.friction ?? 0.5;
  p.v.x -= vt * friction * tx;
  p.v.y -= vt * friction * ty;
  return { x: nx, y: ny }
}
  */




/*
// Resolve all particles in a cluster against multiple sloped walls
function resolveSlopedWallCollisions(cluster: ShapeCluster2D, walls: Wall[]) {
    for (let p of cluster.particles) {
        for (let wall of walls) {
            resolveParticleWallCollision(p, wall);
        }
    }
}
    */


let createSim = demo
// --- Demo setup ---
function demoTire(polygons: Poly[], input: InputController){
  const sim = createSim(polygons); // from previous demo: small rectangular cluster

  // Treat the cluster as a tire
  let tire = new TireController2DPlus(sim.clusters[0], input, TIRE_RADIUS)
  sim.addTireController(tire)

  return { sim, tire };
}

export { demoTire }