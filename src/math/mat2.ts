import type { Vec2 } from "./vec2";

export type Mat2 = { a: number, b: number, c: number, d: number }
// 2x2 matrix stored as {a,b,c,d} = [[a,b],[c,d]]
export function mat2(a: number,b: number,c: number,d: number){ return {a,b,c,d}; }
export function matMulVec(m: Mat2,v: Vec2){ return { x: m.a*v.x + m.b*v.y, y: m.c*v.x + m.d*v.y }; }
export function matMul(m1: Mat2,m2: Mat2){
  return {
    a: m1.a*m2.a + m1.b*m2.c,
    b: m1.a*m2.b + m1.b*m2.d,
    c: m1.c*m2.a + m1.d*m2.c,
    d: m1.c*m2.b + m1.d*m2.d
  };
}
export function matTranspose(m: Mat2){ return mat2(m.a, m.c, m.b, m.d); }


// returns a 2x2 rotation matrix for angle theta (radians)
export function rotationMatrix(theta: number): Mat2 {
    const c = Math.cos(theta);
    const s = Math.sin(theta);
    return mat2(c, -s, s, c);
}

/** Linear interpolation between two 2x2 matrices A and B by t (0..1) */
export function matLerp(A: Mat2, B: Mat2, t: number): Mat2 {
    return mat2(
        A.a * (1 - t) + B.a * t,
        A.b * (1 - t) + B.b * t,
        A.c * (1 - t) + B.c * t,
        A.d * (1 - t) + B.d * t
    );
}

// ---- 2x2 symmetric eigen-decomposition (for polar decomposition) ----
// returns { vals: [l1,l2], vecs: [v1, v2] } where each v is {x,y}
export function eig2x2_sym(s11: number, s12: number, s22: number){
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
export function invSqrtSym2x2(s11: number, s12: number, s22: number){
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
export function polarRotation2x2(A: Mat2){
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
