// xpbd-shape-matching-2d.js
// Simple 2D XPBD-ish meshless shape matching demo
// Author: ChatGPT (adapted for user's request)

// ---- linear algebra helpers for 2D ----

type Vec2 = { x: number, y: number }

function vec2(x=0,y=0){ return {x, y}; }
export function add(a: Vec2,b: Vec2){ return {x: a.x+b.x, y: a.y+b.y}; }
function sub(a: Vec2,b: Vec2){ return {x: a.x-b.x, y: a.y-b.y}; }
function mulScalar(v: Vec2,s: number){ return {x: v.x*s, y: v.y*s}; }
export function dot(a: Vec2,b: Vec2){ return a.x*b.x + a.y*b.y; }

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

const TIRE_RADIUS = 40
const R_tire = TIRE_RADIUS
const N = 6
let ParticleRadius = 2 * Math.PI * R_tire / N / 2;

// ---- Particle and Cluster classes ----
class Particle {

    x: Vec2
    v: Vec2
    m: number
    invMass: number
    xPred: Vec2
    prev: Vec2

    radius = ParticleRadius

  constructor(x: number, y: number, mass=1){
    this.x = vec2(x,y);      // position
    this.prev = vec2(x, y) // previous position
    this.v = vec2(0,0);      // velocity
    this.m = mass;           // mass
    this.invMass = mass > 0 ? 1 / mass : 0;
    this.xPred = vec2(x,y);  // predicted position for the timestep
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

  updateClusterGrounded(walls: Wall[]) {
    const hull = this.computeClusterEnclosingCircle();
    this.is_grounded = walls.some(wall => {
      // approximate: hull touches or slightly below floor
      const closestPoint = closestPointOnSegment(wall.A, wall.B, hull.center);
      return hull.center.y - hull.radius <= closestPoint.y + 1e-4;
    });
  }
}

// ---- Simulator: integrates, collision placeholder, shape clusters ----
class Simulator2D {

    particles: Particle[]
    clusters: ShapeCluster2D[]
    gravity: Vec2
    iterations: number
    //platforms: Platform[]
    walls: Wall[]

    tireController!: TireController2DPlus


  constructor(){
    this.walls = []
    this.particles = [];
    this.clusters = []; // array of ShapeCluster2D
    this.gravity = vec2(0, 2000); // y-down (pixels/sec^2) - tune to your world
    this.iterations = 3; // solver iterations per step
  }

  addWalls(w: Wall[]){ this.walls.push(...w); }
  addParticle(p: Particle){ this.particles.push(p); return p; }
  addCluster(cluster: ShapeCluster2D){ this.clusters.push(cluster); return cluster; }
  addTireController(tire_controller: TireController2DPlus) {
    this.tireController = tire_controller
  }

  // simple ground collision: push up to y >= groundY and simple velocity update
  resolveCollisions(predicted: Particle[]){
    const groundY = 500; // example ground line
    for(let p of predicted){
      if (p.xPred.y > groundY){
        p.xPred.y = groundY;
        if (p.v.y > 0) p.v.y *= -0.2; // bounce/damping
      }
    }
  }

  step(dt: number){

    for (let p of this.particles) {
        p.prev.x = p.x.x
        p.prev.y = p.x.y
    }

    let ctrl = this.tireController

    // 1) external forces -> update velocities and predict positions
    for(let p of this.particles){
      // semi-implicit: v += dt * a
      p.v = add(p.v, mulScalar(this.gravity, dt));
    }

    ctrl.handleInput(dt)

    if (ctrl.jumpBufferTimer > 0 && ctrl.coyoteTimer > 0) {
        ctrl.doJump()
    }
    ctrl.applyAirControl(dt)
    ctrl.applyAirDrag()

    let cl = ctrl.cluster
    if (!cl.xbar) {
        cl.xbar = cl.computeCOMPred()
    }
      //ctrl.moveCOM(dt)
      ctrl.applyGroundTorque(dt)
      ctrl.applyTorque(dt)
      ctrl.variableJump(dt)

      //ctrl.applyRadialSprings(dt)

      for (let p of ctrl.cluster.particles) {
          p.xPred = add(p.x, mulScalar(p.v, dt));
      }

    // 2) initial collision pass (optional)
    //this.resolveCollisions(this.particles);
    for (let cl of this.clusters) {
      //resolveFlatCollisions(cl, this.platforms, dt, this.gravity.y, 0.8)
      resolveSlopedWallCollisions(cl, this.walls)
      resolveClusterHullCollision(cl, this.walls)
    }

    // 3) shape-matching solver iterations (XPBD-like)
    for(let iter=0; iter<this.iterations; iter++){


      for(let cl of this.clusters){
          cl.project(dt);
      }

      // do a final collision pass (ensures non-penetration after projections)
      //this.resolveCollisions(this.particles);

        for (let cl of this.clusters) {
            //resolveFlatCollisions(cl, this.platforms, dt, this.gravity.y, 0.8)
          resolveSlopedWallCollisions(cl, this.walls)
          resolveClusterHullCollision(cl, this.walls)
        }


    }

    for (let cl of this.clusters) {
      cl.updateClusterGrounded(this.walls)
    }



    // 4) finalize positions and update velocities
    for(let p of this.particles){
      // v = (x_new - x_old) / dt
      const newV = mulScalar(sub(p.xPred, p.x), 1/dt);
      p.v = newV;
      p.x = p.xPred;
    }
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
function demo(trackPointss: Vec2[][]){
  const sim = new Simulator2D();

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
  const center = { x: 400, y: 100 }, radius = TIRE_RADIUS;
  for (let i = 0; i < N; i++) {
      const angle = i / N * 2 * Math.PI;
      const x = center.x + radius * Math.cos(angle);
      const y = center.y + radius * Math.sin(angle);
      const p = new Particle(x, y, 1);
      sim.addParticle(p);
      tireParticles.push(p);
  }

  for (let trackPoints of trackPointss) {
    const trackSegments = createTrackSegments(trackPoints, 0.8, 0.5)
    sim.addWalls(trackSegments)
  }


  // rest positions relative to initial center-of-mass (they should be local q_i)
  // here we use the initial positions as rest q's (in world coords) and cluster will compute qbar -> qtilde
  const restQ = tireParticles.map(p => ({x: p.x.x, y: p.x.y}));

  // create cluster: low compliance -> stiff object
  const cluster = new ShapeCluster2D(tireParticles, restQ, /*compliance=*/1e-3, /*stiffness=*/2.99);
  sim.addCluster(cluster);

  return sim;
}

// If run in a main loop, call e.g.:
// const sim = demo();
// setInterval(() => { sim.step(1/60); render(sim); }, 1000/60);

export { Particle, ShapeCluster2D, Simulator2D, demo };



// --- Tire controller ---
class TireController2D {

    cluster: ShapeCluster2D
    radius: number
    desiredVelocity: Vec2
    angularVelocity: number
    frictionCoeff: number
    groundY: number

    static AIR_CONTROL = 0.1
    static MAX_AIR_ANGULAR = 10

    static AIR_DRAG = 0.51
    static AIR_DRAG_LINEAR = 0.927

  constructor(cluster: ShapeCluster2D, radius=20){
    this.cluster = cluster;
    this.radius = radius;
    this.desiredVelocity = vec2(0,0); // translational velocity
    this.angularVelocity = 0;         // rad/sec
    this.frictionCoeff = 0.8
    this.groundY = 100
  }

  applyAirDrag() {
    if (this.cluster.is_grounded) return

    this.angularVelocity *= TireController2D.AIR_DRAG

    for (let p of this.cluster.particles) {
        p.v.x *= TireController2D.AIR_DRAG_LINEAR
    }
  }

    applyAirControl(dt: number, input: Vec2) {
        if (this.cluster.is_grounded) return; // only apply in air

        // Input.x = -1 (left) or 1 (right)
        const targetAngular = input.x * TireController2D.MAX_AIR_ANGULAR;
        const deltaAngular = targetAngular - this.angularVelocity;

        this.angularVelocity += deltaAngular * TireController2D.AIR_CONTROL * dt;
    }


  // Call this before the simulation step
  applyControl(dt: number){
    const cl = this.cluster;

    // --- 1. Apply rolling torque BEFORE COM shift ---
    for(let p of cl.particles){
      const r = sub(p.xPred, cl.xbar);
      const tangential = {x:-r.y, y:r.x};
      p.v = add(p.v, mulScalar(tangential, this.angularVelocity));
    }

    const angularImpulse = 0.0; // rad/sec
    for (let p of cl.particles) {
        const r = sub(p.xPred, cl.xbar);
        const tangential = { x: -r.y, y: r.x };
        p.v = add(p.v, mulScalar(tangential, angularImpulse));
    }


    // 3. friction at contacts
    //const contactParticles = cl.particles.filter(p=>p.xPred.y>=this.groundY-1e-3);
      const contactParticles = cl.particles.filter(p =>
          p.xPred.y >= this.groundY - 1e-3 && sub(p.xPred, cl.xbar).y > 0
      );
    for(let p of contactParticles){
      const vTang=p.v.x;
      const normalForce = p.m * 8200
      const maxFriction = this.frictionCoeff * normalForce * dt
      const frictionImpulse=Math.max(-maxFriction, Math.min(maxFriction, -vTang * p.m));
      p.v.x+=frictionImpulse / p.m;
    }
  }
}


type InputController = { 
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

    JUMP_STRENGTH: number
    MAX_JUMP_HOLD: number
    HOLD_GRAVITY_FACTOR: number

    COYOTE_TIME: number
    JUMP_BUFFER: number

    AIR_CONTROL: number
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

        // Constants (tune these!)
        this.JUMP_STRENGTH = 500;
        this.MAX_JUMP_HOLD = .3;         // seconds
        this.HOLD_GRAVITY_FACTOR = 0.3;   // reduces gravity while holding jump
        this.COYOTE_TIME = .5;           // seconds
        this.JUMP_BUFFER = .5;           // seconds

        this.AIR_CONTROL = 0.5;
        this.MAX_AIR_ANGULAR = 3;
        this.AIR_DRAG = 0.98;
        this.AIR_DRAG_LINEAR = 0.995;

        this.GROUND_FRICTION = 1.9;
        this.GRAVITY = 3000;              // positive down

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
        if (this.cluster.is_grounded) return;

        const targetAngular = this.input.x * this.MAX_AIR_ANGULAR;
        const deltaAngular = targetAngular - this.angularVelocity;

        this.angularVelocity += deltaAngular * this.AIR_CONTROL * dt;
    }

    applyAirDrag() {
        if (this.cluster.is_grounded) return;

        this.angularVelocity *= this.AIR_DRAG;
        for (let p of this.cluster.particles) p.v.x *= this.AIR_DRAG_LINEAR;
    }

    doJump() {
        // compute cluster COM vertical velocity
        let vCOM = 0;
        let massSum = 0;
        for (let p of this.cluster.particles) {
            vCOM += p.v.y / p.invMass;
            massSum += 1 / p.invMass;
        }
        vCOM /= massSum;

        const netImpulse = this.JUMP_STRENGTH + Math.max(0, vCOM);
        for (let p of this.cluster.particles) {
            p.v.y -= netImpulse * p.invMass;
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
            for (let p of this.cluster.particles) {
                p.v.y -= this.GRAVITY * (1 - this.HOLD_GRAVITY_FACTOR) * dt;
            }
        }
    }

    applyTorque(dt: number) {

        let cl = this.cluster

        for (let p of cl.particles){
            const r = { x: p.xPred.x - cl.xbar.x, y: p.xPred.y - cl.xbar.y };
            const rLen = Math.hypot(r.x, r.y);
            if (rLen > 0) {
                const tangential = { x: -r.y / rLen, y: r.x / rLen }; // correct rotation direction
                p.v = add(p.v, mulScalar(tangential, this.angularVelocity * rLen));
            }
        }


        if (this.cluster.is_grounded) {
            const radius = 20;
            const linearVel = this.angularVelocity * radius;
            cl.xbar.x += linearVel * dt;
            for (let p of cl.particles) p.xPred.x += linearVel * dt;
        }


        // 3. friction at contacts
        //const contactParticles = cl.particles.filter(p=>p.xPred.y>=this.groundY-1e-3);
        const contactParticles = cl.particles.filter(p =>
            sub(p.xPred, cl.xbar).y > 0
        );
        for (let p of contactParticles) {
            const vTang = p.v.x;
            const normalForce = p.m * 8200
            const maxFriction = 0.8 * normalForce * dt
            const frictionImpulse = Math.max(-maxFriction, Math.min(maxFriction, -vTang * p.m));
            p.v.x += frictionImpulse / p.m;
        }
    }

    applyGroundTorque(dt: number) {
        if (!this.cluster.is_grounded) return;

        let GROUND_TORQUE = 6; // tuning
        if (Math.sign(this.angularVelocity) !== Math.sign(this.input.x)) {
            GROUND_TORQUE = 30
            this.angularVelocity *= 0.001
        }
        this.angularVelocity += this.input.x * GROUND_TORQUE * dt;

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

// Helper: closest point on a line segment
function closestPointOnSegment(A: Vec2, B: Vec2, P: Vec2) {
    const ABx = B.x - A.x;
    const ABy = B.y - A.y;
    const t = ((P.x - A.x) * ABx + (P.y - A.y) * ABy) / (ABx*ABx + ABy*ABy);
    const clampedT = Math.max(0, Math.min(1, t));
    return { x: A.x + ABx * clampedT, y: A.y + ABy * clampedT };
}

function resolveClusterHullCollision(cluster: ShapeCluster2D, walls: Wall[]) {
    const hull = cluster.computeClusterEnclosingCircle();

    for (let wall of walls) {
        const A = wall.A;
        const B = wall.B;

        // closest point on segment
        const ABx = B.x - A.x;
        const ABy = B.y - A.y;
        let t = ((hull.center.x - A.x) * ABx + (hull.center.y - A.y) * ABy) / (ABx*ABx + ABy*ABy);
        t = Math.min(Math.max(t, 0.001), 0.999);
        const Cx = A.x + ABx * t;
        const Cy = A.y + ABy * t;

        // penetration
        const rx = hull.center.x - Cx;
        const ry = hull.center.y - Cy;
        const dist = Math.hypot(rx, ry);

        if (dist >= hull.radius) continue;

        const penetration = hull.radius - dist;

        // normal
        let nx = rx / dist;
        let ny = ry / dist;

        // optional: flip normal based on wall orientation
        const len = Math.hypot(ABx, ABy);
        let wx = -ABy / len;
        let wy = ABx / len;
        if ((hull.center.x - A.x)*wx + (hull.center.y - A.y)*wy < 0) {
            wx = -wx;
            wy = -wy;
        }

        nx = wx;
        ny = wy;

        // push cluster COM
        let softness = 0.1
        const com = cluster.computeCOMPred();
        com.x += penetration * nx * softness;
        com.y += penetration * ny * softness;



        // distribute delta to particles (simple: proportional)
        for (let p of cluster.particles) {
            p.xPred.x += penetration * nx * softness;
            p.xPred.y += penetration * ny * softness;
        }
    }
}



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




// Resolve all particles in a cluster against multiple sloped walls
function resolveSlopedWallCollisions(cluster: ShapeCluster2D, walls: Wall[]) {
    for (let p of cluster.particles) {
        for (let wall of walls) {
            resolveParticleWallCollision(p, wall);
        }
    }
}


// --- 1. Convert corner points to wall segments ---
function createTrackSegments(points: Vec2[], restitution = 0.8, friction = 0.8) {
    const segments = [];
    for (let i = 0; i < points.length - 1; i++) {
        segments.push({
            A: points[i],
            B: points[i + 1],
            restitution,
            friction
        });
    }
    return segments;
}

type Wall = {
    A: {x: number, y: number}  // start point
    B: {x: number, y: number}  // end point
    restitution: number        // optional bounce factor
    friction: number           // optional friction along the slope
}


let createSim = demo
// --- Demo setup ---
function demoTire(trackPoints: Vec2[][], input: InputController){
  const sim = createSim(trackPoints); // from previous demo: small rectangular cluster

  // Treat the cluster as a tire
  let tire = new TireController2DPlus(sim.clusters[0], input, TIRE_RADIUS)
  sim.addTireController(tire)

  return { sim, tire };
}

export { demoTire, TireController2D }