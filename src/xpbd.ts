// xpbd-shape-matching-2d.js
// Simple 2D XPBD-ish meshless shape matching demo
// Author: ChatGPT (adapted for user's request)

// ---- linear algebra helpers for 2D ----

type Vec2 = { x: number, y: number }

function vec2(x=0,y=0){ return {x, y}; }
export function add(a: Vec2,b: Vec2){ return {x: a.x+b.x, y: a.y+b.y}; }
function sub(a: Vec2,b: Vec2){ return {x: a.x-b.x, y: a.y-b.y}; }
function mulScalar(v: Vec2,s: number){ return {x: v.x*s, y: v.y*s}; }
function dot(a: Vec2,b: Vec2){ return a.x*b.x + a.y*b.y; }

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

// ---- Particle and Cluster classes ----
class Particle {

    x: Vec2
    v: Vec2
    m: number
    invMass: number
    xPred: Vec2

  constructor(x: number, y: number, mass=1){
    this.x = vec2(x,y);      // position
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
    const Msum = particles.reduce((s,_,i) => s + particles[i].m, 0) || 1;
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
}

// ---- Simulator: integrates, collision placeholder, shape clusters ----
class Simulator2D {

    particles: Particle[]
    clusters: ShapeCluster2D[]
    gravity: Vec2
    iterations: number


  constructor(){
    this.particles = [];
    this.clusters = []; // array of ShapeCluster2D
    this.gravity = vec2(0, 2000); // y-down (pixels/sec^2) - tune to your world
    this.iterations = 3; // solver iterations per step
  }

  addParticle(p: Particle){ this.particles.push(p); return p; }
  addCluster(cluster: ShapeCluster2D){ this.clusters.push(cluster); return cluster; }

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
    // 1) external forces -> update velocities and predict positions
    for(let p of this.particles){
      // semi-implicit: v += dt * a
      p.v = add(p.v, mulScalar(this.gravity, dt));
      p.xPred = add(p.x, mulScalar(p.v, dt));
    }

    // 2) initial collision pass (optional)
    this.resolveCollisions(this.particles);

    // 3) shape-matching solver iterations (XPBD-like)
    for(let iter=0; iter<this.iterations; iter++){
      for(let cl of this.clusters){
        cl.project(dt);
      }
      // do a final collision pass (ensures non-penetration after projections)
      this.resolveCollisions(this.particles);
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

// ---- tiny example usage ----
function demo(){
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
  const N = 20; // circle particles
  const center = { x: 400, y: 350 }, radius = 40;
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
  const cluster = new ShapeCluster2D(tireParticles, restQ, /*compliance=*/1e-4, /*stiffness=*/5.99);
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


  constructor(cluster: ShapeCluster2D, radius=20){
    this.cluster = cluster;
    this.radius = radius;
    this.desiredVelocity = vec2(0,0); // translational velocity
    this.angularVelocity = 0;         // rad/sec
    this.frictionCoeff = 0.8
    this.groundY = 100
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

    // 1. Move COM
    const deltaCOM = mulScalar(this.desiredVelocity, dt);
    cl.xbar = add(cl.xbar, deltaCOM);

    // 2. Apply COM shift to all particles
    for(let p of cl.particles){
      p.xPred = add(p.xPred, deltaCOM);
    }

    // 3. Apply rolling / torque: tangential velocity to each particle
    for(let p of cl.particles){
      const r = sub(p.xPred, cl.xbar);         // vector from COM
      const tangential = {x: -r.y, y: r.x};    // perpendicular
      p.v = add(p.v, mulScalar(tangential, this.angularVelocity));
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

let createSim = demo
// --- Demo setup ---
function demoTire(){
  const sim = createSim(); // from previous demo: small rectangular cluster

  // Treat the cluster as a tire
  const tireCluster = sim.clusters[0];
  const tire = new TireController2D(tireCluster, /*radius=*/20);

  return { sim, tire };
}

export { demoTire, TireController2D, canJump }

function canJump(cluster: ShapeCluster2D, groundY: number) {
    return cluster.particles.some(p => p.xPred.y >= groundY - 1e-3);
}