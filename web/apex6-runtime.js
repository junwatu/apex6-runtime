/*
 * APEX-6 runtime — web (three.js / plain Object3D) control + cable routing
 * =======================================================================
 *
 * Drives the exported APEX-6 rig by setting joint rotations, exactly like the
 * Unity kit. The model is a named pivot hierarchy with no armature:
 *
 *     CTRL_J1_Waist -> CTRL_J2_Shoulder -> ... -> CTRL_J6_ToolRoll
 *                                              -> CTRL_Gripper -> CTRL_FingerL/R
 *                                              -> CTRL_TCP
 *
 * Nothing in here is hard-coded to one importer. Unity bakes the Z-up -> Y-up
 * conversion in different places depending on the file, so this runtime *solves*
 * the conversion from the known rest positions carried in `apex6_joint_map.json`,
 * then expresses each joint axis in its parent frame. That is the same method
 * that was verified on the Unity side (map-vs-FK 4.2e-7 m, live import 0.003 mm).
 *
 * Zero dependencies: the vector/quaternion/matrix maths is implemented here, so
 * the same code runs in a browser with three.js and head-less in Node for
 * verification. It only ever touches plain `position` / `quaternion` / `scale`
 * fields and a `children` array, all of which three.js Object3D provides.
 *
 *   import { Apex6Robot } from './apex6-runtime.js';
 *
 *   const robot = new Apex6Robot({ root: gltf.scene, map: await fetch(mapUrl).then(r => r.json()) });
 *   robot.prepare();
 *   robot.setJoints([0, -20, 70, 0, 55, 0]);
 *   robot.update(1 / 60);          // rate-limited toward the command
 *   robot.setGripper(0.040);       // pad-face gap, metres
 *   console.log(robot.tcpWorld());
 *
 * See README.md for the full API.
 */

const DEG = Math.PI / 180;

/* ------------------------------------------------------------------ *
 *  Tiny maths (plain objects, no dependencies)                        *
 * ------------------------------------------------------------------ */

export const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
const vCopy = (a) => ({ x: a.x, y: a.y, z: a.z });
const vAdd = (a, b) => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const vSub = (a, b) => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const vMul = (a, s) => ({ x: a.x * s, y: a.y * s, z: a.z * s });
const vDot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
const vCross = (a, b) => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x,
});
const vLen = (a) => Math.sqrt(vDot(a, a));
function vNorm(a) {
  const L = vLen(a);
  return L > 1e-12 ? vMul(a, 1 / L) : v3();
}

const qIdentity = () => ({ x: 0, y: 0, z: 0, w: 1 });
function qAxisAngle(axis, ang) {
  const a = vNorm(axis);
  const h = 0.5 * ang;
  const s = Math.sin(h);
  return { x: a.x * s, y: a.y * s, z: a.z * s, w: Math.cos(h) };
}
function qMul(a, b) {
  return {
    x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
    z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  };
}
function qConj(q) {
  return { x: -q.x, y: -q.y, z: -q.z, w: q.w };
}

/* row-major 4x4, as a flat 16-array */
const m4Identity = () =>
  new Float64Array([1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]);

function m4FromTRS(p, q, s) {
  const { x, y, z, w } = q;
  const x2 = x + x, y2 = y + y, z2 = z + z;
  const xx = x * x2, xy = x * y2, xz = x * z2;
  const yy = y * y2, yz = y * z2, zz = z * z2;
  const wx = w * x2, wy = w * y2, wz = w * z2;
  const sx = s.x, sy = s.y, sz = s.z;
  return new Float64Array([
    (1 - (yy + zz)) * sx, (xy - wz) * sy, (xz + wy) * sz, p.x,
    (xy + wz) * sx, (1 - (xx + zz)) * sy, (yz - wx) * sz, p.y,
    (xz - wy) * sx, (yz + wx) * sy, (1 - (xx + yy)) * sz, p.z,
    0, 0, 0, 1,
  ]);
}

function m4Mul(a, b) {
  const o = new Float64Array(16);
  for (let r = 0; r < 4; r++) {
    for (let c = 0; c < 4; c++) {
      o[r * 4 + c] =
        a[r * 4] * b[c] + a[r * 4 + 1] * b[4 + c] +
        a[r * 4 + 2] * b[8 + c] + a[r * 4 + 3] * b[12 + c];
    }
  }
  return o;
}

const m4Point = (m, p) => v3(
  m[0] * p.x + m[1] * p.y + m[2] * p.z + m[3],
  m[4] * p.x + m[5] * p.y + m[6] * p.z + m[7],
  m[8] * p.x + m[9] * p.y + m[10] * p.z + m[11]);

const m4Dir = (m, d) => v3(
  m[0] * d.x + m[1] * d.y + m[2] * d.z,
  m[4] * d.x + m[5] * d.y + m[6] * d.z,
  m[8] * d.x + m[9] * d.y + m[10] * d.z);

/** inverse of an affine 4x4 (rotation/reflection + translation); ignores nothing. */
function m4InverseRigid(m) {
  const M3 = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
  const inv = mat3Inverse(M3) || mat3Transpose(M3);
  const r = new Float64Array(16);
  r[0] = inv[0]; r[1] = inv[1]; r[2] = inv[2];
  r[4] = inv[3]; r[5] = inv[4]; r[6] = inv[5];
  r[8] = inv[6]; r[9] = inv[7]; r[10] = inv[8];
  const t = v3(m[3], m[7], m[11]);
  const it = mat3MulVec(inv, t);
  r[3] = -it.x; r[7] = -it.y; r[11] = -it.z; r[15] = 1;
  return r;
}

/* 3x3 helpers for the conversion solve */
function mat3Zero() { return [0, 0, 0, 0, 0, 0, 0, 0, 0]; }
function mat3MulVec(m, v) {
  return v3(
    m[0] * v.x + m[1] * v.y + m[2] * v.z,
    m[3] * v.x + m[4] * v.y + m[5] * v.z,
    m[6] * v.x + m[7] * v.y + m[8] * v.z);
}
function mat3Mul(a, b) {
  const o = mat3Zero();
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 3; j++)
      for (let k = 0; k < 3; k++) o[i * 3 + j] += a[i * 3 + k] * b[k * 3 + j];
  return o;
}
function mat3Det(m) {
  return m[0] * (m[4] * m[8] - m[5] * m[7])
       - m[1] * (m[3] * m[8] - m[5] * m[6])
       + m[2] * (m[3] * m[7] - m[4] * m[6]);
}
function mat3Inverse(m) {
  const det = mat3Det(m);
  if (Math.abs(det) < 1e-12) return null;
  const id = 1 / det;
  return [
    (m[4] * m[8] - m[5] * m[7]) * id,
    (m[2] * m[7] - m[1] * m[8]) * id,
    (m[1] * m[5] - m[2] * m[4]) * id,
    (m[5] * m[6] - m[3] * m[8]) * id,
    (m[0] * m[8] - m[2] * m[6]) * id,
    (m[2] * m[3] - m[0] * m[5]) * id,
    (m[3] * m[7] - m[4] * m[6]) * id,
    (m[1] * m[6] - m[0] * m[7]) * id,
    (m[0] * m[4] - m[1] * m[3]) * id,
  ];
}
/** the upper-left 3x3 of a row-major 4x4 */
const mat3FromM4 = (m) => [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]];
/** inverse of an orthonormal rotation (transpose) */
const mat3Transpose = (m) => [m[0], m[3], m[6], m[1], m[4], m[7], m[2], m[5], m[8]];
const MAT3_IDENTITY = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];

/** Rodrigues rotation matrix (row-major) about a unit axis by `ang` radians */
function mat3Rot(ang, axis) {
  const a = vNorm(axis);
  const c = Math.cos(ang), s = Math.sin(ang), t = 1 - c;
  const { x, y, z } = a;
  return [
    c + x * x * t, x * y * t - z * s, x * z * t + y * s,
    y * x * t + z * s, c + y * y * t, y * z * t - x * s,
    z * x * t - y * s, z * y * t + x * s, c + z * z * t,
  ];
}

/** axis + angle of a (near-)rotation matrix; angle carries the sense */
function mat3ToAxisAngle(m) {
  const tr = m[0] + m[4] + m[8];
  const ang = Math.acos(clamp((tr - 1) / 2, -1, 1));
  if (ang < 1e-8) return { axis: v3(1, 0, 0), ang: 0 };
  let ax = v3(m[7] - m[5], m[2] - m[6], m[3] - m[1]);
  if (vLen(ax) < 1e-9) {
    // ~180 deg: recover axis from the diagonal
    ax = v3(Math.sqrt(Math.max(0, (m[0] + 1) / 2)),
            Math.sqrt(Math.max(0, (m[4] + 1) / 2)),
            Math.sqrt(Math.max(0, (m[8] + 1) / 2)));
  }
  return { axis: vNorm(ax), ang };
}

const clamp = (x, lo, hi) => (x < lo ? lo : x > hi ? hi : x);
const D2R = DEG, R2D = 180 / Math.PI;

/* ------------------------------------------------------------------ *
 *  Scene-graph helpers (work on Object3D or any node with            *
 *  position / quaternion / scale / parent / children)                *
 * ------------------------------------------------------------------ */

function findByName(root, name) {
  if (!root) return null;
  if (root.name === name) return root;
  const kids = root.children || [];
  for (let i = 0; i < kids.length; i++) {
    const hit = findByName(kids[i], name);
    if (hit) return hit;
  }
  return null;
}

/** world matrix, accumulated from local TRS up to (and excluding) `stopAt`. */
function worldMatrix(node) {
  const chain = [];
  for (let n = node; n; n = n.parent) chain.push(n);
  let m = m4Identity();
  for (let i = chain.length - 1; i >= 0; i--) {
    const n = chain[i];
    m = m4Mul(m, m4FromTRS(
      n.position || v3(), n.quaternion || qIdentity(), n.scale || v3(1, 1, 1)));
  }
  return m;
}

function setQuat(node, q) {
  const t = node.quaternion;
  if (typeof t.set === 'function') t.set(q.x, q.y, q.z, q.w);
  else { t.x = q.x; t.y = q.y; t.z = q.z; t.w = q.w; }
}

function setPos(node, p) {
  const t = node.position;
  if (typeof t.set === 'function') t.set(p.x, p.y, p.z);
  else { t.x = p.x; t.y = p.y; t.z = p.z; }
}

/* ------------------------------------------------------------------ *
 *  Map helpers                                                       *
 * ------------------------------------------------------------------ */

export class Apex6Map {
  constructor(json) { this.data = json; }
  get joints() { return this.data.joints; }
  get jointCount() { return this.data.joints.length; }
  get cables() { return this.data.cables || []; }
  get gripper() { return this.data.gripper; }
  get restPoints() { return this.data.rest_points; }
  jointNames() { return this.data.joints.map((j) => j.name); }
  jointByShort(short) {
    return this.data.joints.find((j) => j.short === short);
  }
  limitMin(i) { return this.data.joints[i].limit_min_deg; }
  limitMax(i) { return this.data.joints[i].limit_max_deg; }
  clamp(i, deg) { return clamp(deg, this.limitMin(i), this.limitMax(i)); }
  homeDeg() { return this.data.joints.map((j) => j.home_deg); }
  maxSpeed(i) { return this.data.joints[i].max_speed_deg_s; }
  fingerHalf(opening) {
    return Math.max(0, opening) * 0.5 + this.data.gripper.pad_face_offset;
  }
  get openingMin() { return this.data.gripper.opening_min; }
  get openingMax() { return this.data.gripper.opening_max; }
}

/* ------------------------------------------------------------------ *
 *  Apex6Robot                                                        *
 * ------------------------------------------------------------------ */

export class Apex6Robot {
  /**
   * @param {object} opts
   * @param {object} opts.root  the loaded glTF scene / model root
   * @param {object} opts.map   the parsed `apex6_joint_map.json`
   */
  constructor(opts) {
    this.root = opts.root;
    this.map = opts.map instanceof Apex6Map ? opts.map : new Apex6Map(opts.map);

    this.joints = new Array(this.map.jointCount).fill(null);
    this.gripperRoot = null;
    this.fingerL = null;
    this.fingerR = null;
    this.tcp = null;

    this.ready = false;

    // per-joint calibration
    this._axisLocal = [];
    this._axisSign = [];
    this._restRot = [];
    // solved conversion + diagnostics
    this._conv = [1, 0, 0, 0, 0, 1, 0, -1, 0];
    this.conversionDet = 1;
    this.conversionResidual = 0;

    // commanded / applied state
    this._command = new Array(this.map.jointCount).fill(0);
    this._applied = new Array(this.map.jointCount).fill(0);
    this.opening = this.map.openingMax;
    this._openingApplied = this.map.openingMax;

    // gripper calibration
    this._fingerBase = [v3(), v3()];
    this._fingerAxis = v3(0, 0, 1);
    this._fingerSign = [-1, 1];
    this._fingerHalfAtImport = this.map.fingerHalf(this.map.openingMax);
  }

  /* ---- setup ------------------------------------------------------ */

  findByRole() {
    this.joints = this.map.jointNames().map((n) => findByName(this.root, n));
    this.gripperRoot = findByName(this.root, this.map.gripper.root);
    this.fingerL = findByName(this.root, this.map.gripper.fingers.L);
    this.fingerR = findByName(this.root, this.map.gripper.fingers.R);
    this.tcp = findByName(this.root, this.map.data.tcp.name);
    this.ready = this.joints.length >= 6 && this.joints[0] && this.joints[5];
    return this.ready;
  }

  /**
   * Find the rig and solve the importer's Blender->engine conversion from the
   * rest positions. **Call this with the arm at its zero pose** (the exported
   * file's own pose, which is zero).
   */
  prepare() {
    this.findByRole();
    this._calibrate();
    this._cacheFingers();
    for (let i = 0; i < this._command.length; i++) { this._command[i] = 0; this._applied[i] = 0; }
    this.opening = this.map.openingMax;
    this._openingApplied = this.map.openingMax;
    this.applyNow();
    return this.ready;
  }

  _calibrate() {
    const rp = this.map.restPoints;
    const A = mat3Zero(), B = mat3Zero();
    let used = 0;

    for (const p of rp) {
      const node = findByName(this.root, p.name);
      if (!node) continue;
      const world = m4Point(worldMatrix(node), v3(0, 0, 0));
      const b = [p.blender[0], p.blender[1], p.blender[2]];
      const u = [world.x, world.y, world.z];
      for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) {
        A[i * 3 + j] += b[i] * b[j];
        B[i * 3 + j] += u[i] * b[j];
      }
      used++;
    }

    const ainv = used >= 4 ? mat3Inverse(A) : null;
    const C = ainv ? mat3Mul(B, ainv) : [1, 0, 0, 0, 0, 1, 0, -1, 0];
    this._conv = C;
    this.conversionDet = mat3Det(C);

    // residual of the rest fit -> an honest quality number
    this.conversionResidual = 0;
    for (const p of rp) {
      const node = findByName(this.root, p.name);
      if (!node) continue;
      const pred = this.convertBlender(v3(p.blender[0], p.blender[1], p.blender[2]));
      const act = m4Point(worldMatrix(node), v3(0, 0, 0));
      const err = vLen(vSub(pred, act));
      if (err > this.conversionResidual) this.conversionResidual = err;
    }

    const mirrored = this.conversionDet < 0;
    this._mirrored = mirrored;
    this._axisLocal = [];
    this._axisSign = [];
    this._restRot = [];
    for (let i = 0; i < this.map.jointCount; i++) {
      const j = this.joints[i];
      const axB = this.map.data.joints[i].blender_axis_vec;
      // the joint's axis in the imported world frame. A det<0 world map
      // reverses the sense of every joint (Rot(a,t) == Rot(-a,-t)), so the
      // axis is negated before it is carried into the parent frame.
      let world = vNorm(mat3MulVec(C, v3(axB[0], axB[1], axB[2])));
      if (mirrored) world = vMul(world, -1);

      // local rotation needed for a world rotation about `world`:
      //   R_local = P^-1 * R_world * P          (P = parent world linear map)
      // exact for any parent, including a mirrored/reflected import; the
      // extracted angle sign is the joint's sense in that frame.
      const P = j && j.parent ? mat3FromM4(worldMatrix(j.parent)) : MAT3_IDENTITY();
      const Pinv = mat3Inverse(P) || mat3Transpose(P);
      const local = mat3Mul(Pinv, mat3Mul(mat3Rot(1, world), P));
      const { axis, ang } = mat3ToAxisAngle(local);

      this._axisLocal[i] = axis;
      this._axisSign[i] = ang >= 0 ? 1 : -1;
      this._restRot[i] = j && j.quaternion
        ? { x: j.quaternion.x, y: j.quaternion.y, z: j.quaternion.z, w: j.quaternion.w }
        : qIdentity();
    }
  }

  _cacheFingers() {
    this._fingerBase = [v3(), v3()];
    if (this.fingerL) this._fingerBase[0] = vCopy(this.fingerL.position || v3());
    if (this.fingerR) this._fingerBase[1] = vCopy(this.fingerR.position || v3());

    const sep = vSub(this._fingerBase[1], this._fingerBase[0]);
    if (vLen(sep) > 1e-7) {
      this._fingerAxis = vNorm(sep);
      this._fingerSign = [-1, 1];
    } else {
      const a = this.map.gripper.unity_travel_axis || [0, 0, 1];
      this._fingerAxis = vNorm(v3(a[0], a[1], a[2]));
      this._fingerSign = [-1, 1];
    }
  }

  /** Map a Blender (Z-up) point into the imported rig's frame. */
  convertBlender(p) { return mat3MulVec(this._conv, p); }

  /* ---- control ---------------------------------------------------- */

  setJoint(i, deg) {
    if (i < 0 || i >= this._command.length) return;
    this._command[i] = this.map.clamp(i, deg);
  }
  setJoints(deg) {
    if (!deg) return;
    for (let i = 0; i < this._command.length && i < deg.length; i++)
      if (!Number.isNaN(deg[i])) this.setJoint(i, deg[i]);
  }
  getJoint(i) { return i >= 0 && i < this._command.length ? this._command[i] : 0; }
  getJoints() { return this._command.slice(); }
  getAppliedJoints() { return this._applied.slice(); }

  goZero() { this.setJoints(new Array(this.map.jointCount).fill(0)); }
  goHome() { this.setJoints(this.map.homeDeg()); }

  setGripper(openingMeters) {
    this.opening = clamp(openingMeters, this.map.openingMin, this.map.openingMax);
  }
  openGripper() { this.setGripper(this.map.openingMax); }
  closeGripper() { this.setGripper(this.map.openingMin); }
  getGripper() { return this.opening; }

  atTarget(tol = 0.01) {
    for (let i = 0; i < this._applied.length; i++)
      if (Math.abs(this._applied[i] - this._command[i]) > tol) return false;
    return Math.abs(this._openingApplied - this.opening) < 1e-4;
  }

  /**
   * Rate-limited step toward the command (call from your animation loop).
   * @param {number} dt seconds
   * @param {number} [speedScale] 1 = rated speed, 0 = instant
   */
  update(dt, speedScale = 1) {
    if (!Number.isFinite(dt) || dt <= 0) return false;
    let moved = false;
    for (let i = 0; i < this._applied.length; i++) {
      const target = speedScale <= 0
        ? this._command[i]
        : this._moveTowards(this._applied[i], this._command[i],
            this.map.maxSpeed(i) * speedScale * dt);
      if (Math.abs(target - this._applied[i]) > 1e-9) { this._applied[i] = target; moved = true; }
    }
    if (speedScale > 0 && Math.abs(this._openingApplied - this.opening) > 1e-9) {
      this._openingApplied = this._moveTowards(this._openingApplied, this.opening, 0.15 * dt);
      moved = true;
    }
    if (moved) this.applyNow();
    return moved;
  }

  /** Push the applied state onto the transforms immediately. */
  applyNow() {
    for (let i = 0; i < this._applied.length; i++) {
      const j = this.joints[i];
      if (!j) continue;
      const q = qMul(qAxisAngle(this._axisLocal[i], this._applied[i] * DEG * this._axisSign[i]), this._restRot[i]);
      setQuat(j, q);
    }
    this._applyFingers(this._openingApplied);
  }

  /** Set the command and apply it within this same call (no smoothing). */
  applyCommandNow() {
    for (let i = 0; i < this._command.length; i++) this._applied[i] = this._command[i];
    this._openingApplied = this.opening;
    this.applyNow();
  }

  _applyFingers(opening) {
    const half = this.map.fingerHalf(opening);
    this._applyFinger(0, this.fingerL, half);
    this._applyFinger(1, this.fingerR, half);
  }

  _applyFinger(slot, finger, half) {
    if (!finger) return;
    const base = this._fingerBase[slot];
    const d = this._fingerSign[slot] * (half - this._fingerHalfAtImport);
    setPos(finger, vAdd(base, vMul(this._fingerAxis, d)));
  }

  /* ---- queries ---------------------------------------------------- */

  tcpWorld(out) {
    const p = this.tcp ? m4Point(worldMatrix(this.tcp), v3(0, 0, 0)) : v3();
    if (out && typeof out.set === 'function') out.set(p.x, p.y, p.z);
    return p;
  }

  get conversionNote() {
    const C = this._conv;
    const f = (x) => x.toFixed(4);
    return `C=[(${f(C[0])},${f(C[1])},${f(C[2])})(${f(C[3])},${f(C[4])},${f(C[5])})(${f(C[6])},${f(C[7])},${f(C[8])})] det=${this.conversionDet.toFixed(4)}` +
      (this.conversionDet < 0 ? ' (mirrored -> joint senses inverted)' : '') +
      ` residual=${(this.conversionResidual * 1000).toFixed(4)} mm`;
  }

  /**
   * Replay the map's self-test poses and compare the live TCP with the
   * generated expectation. Returns { maxError, worst, passes } in metres.
   */
  selfTest() {
    const st = this.map.data.self_test;
    const saved = this._applied.slice();
    let maxError = 0, worst = -1;
    for (let k = 0; k < st.poses_deg.length; k++) {
      this.setJoints(st.poses_deg[k]);
      this.applyCommandNow();
      const got = this.tcpWorld();
      const exp = this.convertBlender(v3(st.tcp_blender[k][0], st.tcp_blender[k][1], st.tcp_blender[k][2]));
      const err = vLen(vSub(got, exp));
      if (err > maxError) { maxError = err; worst = k; }
    }
    // restore
    for (let i = 0; i < saved.length; i++) this._applied[i] = saved[i];
    this.applyNow();
    return { maxError, worst, tolerance: st.tolerance_m, passes: maxError <= st.tolerance_m };
  }

  _moveTowards(cur, tgt, maxDelta) {
    const d = tgt - cur;
    if (Math.abs(d) <= maxDelta) return tgt;
    return cur + Math.sign(d) * maxDelta;
  }
}

/* ------------------------------------------------------------------ *
 *  Apex6Cables — rebuild the flexible hoses every frame              *
 * ------------------------------------------------------------------ *
 *
 * The `Cable_*` tubes are baked as static meshes at the export pose, so on
 * their own they stay put while the arm moves. This rebuilds each one from the
 * same spine Blender uses:
 *
 *     p(t) = pa + (pb - pa)*t + away * slack*sin(pi*t)
 *     slack = slackBase * min(1, 0.55 + 0.6*|pb - pa|)
 *
 * Attachment points come from the joint-parented `*_StrainRelief` / `*_Clamp`
 * objects; radius, slack and bow come from the map (generated from the same
 * CABLE_ROUTES Blender routes from). The bow is mapped through the solved
 * conversion, so it is correct for any importer.
 */
export class Apex6Cables {
  constructor(robot, opts = {}) {
    this.robot = robot;
    this.root = opts.root || robot.root;
    this.spineSamples = opts.spineSamples || 24;
    this.ringSegments = opts.ringSegments || 14;
    this.routing = opts.routing !== false;
    this.cables = [];
    this._ready = false;
  }

  prepare() {
    this.cables = [];
    const robot = this.robot;
    for (const route of robot.map.cables) {
      const meshNode = findByName(this.root, route.name);
      const relief = findByName(this.root, route.name + '_StrainRelief');
      const clampNode = findByName(this.root, route.name + '_Clamp');
      const entry = { route, ok: false };
      if (!meshNode || !relief || !clampNode || !meshNode.geometry) { this.cables.push(entry); continue; }

      entry.node = meshNode;
      entry.linkA = relief.parent || this.root;
      entry.linkB = clampNode.parent || this.root;
      entry.paLocal = m4Point(
        m4InverseRigid(worldMatrix(entry.linkA)),
        m4Point(worldMatrix(relief), v3(0, 0, 0)));
      entry.pbLocal = m4Point(
        m4InverseRigid(worldMatrix(entry.linkB)),
        m4Point(worldMatrix(clampNode), v3(0, 0, 0)));

      const bow = v3(route.bow[0], route.bow[1], route.bow[2]);
      const awayWorld = robot.convertBlender(bow);
      const invA0 = mat3FromM4(worldMatrix(entry.linkA));
      const invA = mat3Inverse(invA0) || mat3Transpose(invA0);
      entry.awayLocalA = vNorm(mat3MulVec(invA, awayWorld));
      entry.radius = route.radius;
      entry.slackBase = route.slack;

      // keep the host's geometry object (three.js BufferGeometry) so we can
      // rebuild into it without breaking the material/mesh that uses it
      entry.target = meshNode.geometry || null;
      entry.geometry = this._makeGeometry();
      entry.ok = true;
      this.cables.push(entry);
    }
    this._ready = true;
    return this.cables.filter((c) => c.ok).length;
  }

  _makeGeometry() {
    const rings = Math.max(3, this.spineSamples) + 1;
    const seg = Math.max(4, this.ringSegments);
    const vertCount = rings * seg + 2;
    const index = [];
    for (let i = 0; i < rings - 1; i++) {
      for (let s = 0; s < seg; s++) {
        const s1 = (s + 1) % seg;
        const a = i * seg + s, b = i * seg + s1;
        const c = (i + 1) * seg + s1, d = (i + 1) * seg + s;
        index.push(a, b, c, a, c, d);
      }
    }
    const capA = rings * seg, capB = capA + 1;
    for (let s = 0; s < seg; s++) {
      const s1 = (s + 1) % seg;
      index.push(capA, s1, s);
      index.push(capB, (rings - 1) * seg + s, (rings - 1) * seg + s1);
    }
    const pos = new Float32Array(vertCount * 3);
    const nrm = new Float32Array(vertCount * 3);
    return { pos, nrm, index, vertCount, rings, seg };
  }

  /** Rebuild every cable from the current joint transforms. */
  update() {
    if (!this._ready || !this.routing) return;
    for (const c of this.cables) {
      if (!c.ok) continue;

      const pa = m4Point(worldMatrix(c.linkA), c.paLocal);
      const pb = m4Point(worldMatrix(c.linkB), c.pbLocal);
      const d = vSub(pb, pa);
      const len = vLen(d);
      if (len < 1e-4) continue;
      const u = vMul(d, 1 / len);

      let away = m4Dir(worldMatrix(c.linkA), c.awayLocalA);
      away = vSub(away, vMul(u, vDot(away, u)));
      if (vLen(away) < 1e-8) away = vCross(u, v3(1, 0, 0));
      if (vLen(away) < 1e-8) away = vCross(u, v3(0, 1, 0));
      away = vNorm(away);

      const slack = c.slackBase * Math.min(1, 0.55 + 0.6 * len);

      // write in the tube object's LOCAL frame (it carries the import's
      // conversion rotation, so world-space vertices would be rotated twice)
      const inv = m4InverseRigid(worldMatrix(c.node));
      const paL = m4Point(inv, pa);
      const pbL = m4Point(inv, pb);
      const dL = vSub(pbL, paL);
      const uL = vMul(dL, 1 / Math.max(len, 1e-9));
      const awayL = vNorm(m4Dir(inv, away));

      this._buildTube(c, paL, dL, uL, awayL, slack);
      this._flush(c);
    }
  }

  /**
   * Copy the rebuilt tube into the host geometry.
   *
   * - three.js `BufferGeometry` (has `attributes.position` + `setAttribute`):
   *   build fresh BufferAttributes using the *same constructors* the loader
   *   already used, so no `three` import is needed here.
   * - a plain geometry with matching buffer sizes: write in place.
   * - otherwise (e.g. a head-less fake): publish plain typed arrays.
   */
  _flush(c) {
    const g = c.geometry;
    const target = c.target;
    if (!target) return;

    const attrs = target.attributes;
    if (attrs && attrs.position && typeof target.setAttribute === 'function') {
      const PosCtor = attrs.position.constructor || Float32Array;
      const NrmCtor = (attrs.normal || attrs.position).constructor || Float32Array;
      target.setAttribute('position', new PosCtor(g.pos, 3));
      target.setAttribute('normal', new NrmCtor(g.nrm, 3));
      const IdxCtor = (target.index && target.index.constructor) || Uint32Array;
      const idx = new IdxCtor(g.index);
      if (typeof target.setIndex === 'function') target.setIndex(idx);
      else target.index = idx;
      if (typeof target.computeBoundingSphere === 'function') target.computeBoundingSphere();
      if (typeof target.computeBoundingBox === 'function') target.computeBoundingBox();
      return;
    }

    if (attrs && attrs.position && attrs.position.array &&
        attrs.position.array.length === g.pos.length) {
      attrs.position.array.set(g.pos);
      attrs.position.needsUpdate = true;
      if (attrs.normal && attrs.normal.array.length === g.nrm.length) {
        attrs.normal.array.set(g.nrm);
        attrs.normal.needsUpdate = true;
      }
      return;
    }

    target.attributes = {
      position: { array: g.pos, itemSize: 3, count: g.vertCount, needsUpdate: true },
      normal: { array: g.nrm, itemSize: 3, count: g.vertCount, needsUpdate: true },
    };
    target.index = g.index;
  }

  _buildTube(c, pa, d, u, away, slack) {
    const g = c.geometry;
    const seg = g.seg, rings = g.rings, r = c.radius;
    const pos = g.pos, nrm = g.nrm;
    let o = 0;

    let prevN1 = v3();
    let tangentLast = u;

    for (let i = 0; i < rings; i++) {
      const t = i / (rings - 1);
      const centre = vAdd(vAdd(pa, vMul(d, t)), vMul(away, slack * Math.sin(Math.PI * t)));
      let tangent = vAdd(d, vMul(away, slack * Math.PI * Math.cos(Math.PI * t)));
      if (vLen(tangent) < 1e-8) tangent = u;
      tangent = vNorm(tangent);
      tangentLast = tangent;

      let n1;
      if (i === 0) n1 = vSub(away, vMul(tangent, vDot(away, tangent)));
      else n1 = vSub(prevN1, vMul(tangent, vDot(prevN1, tangent)));
      if (vLen(n1) < 1e-8) n1 = vCross(tangent, v3(1, 0, 0));
      if (vLen(n1) < 1e-8) n1 = vCross(tangent, v3(0, 1, 0));
      n1 = vNorm(n1);
      const n2 = vCross(tangent, n1);
      prevN1 = n1;

      for (let s = 0; s < seg; s++) {
        const a = 2 * Math.PI * s / seg;
        const dir = vAdd(vMul(n1, Math.cos(a)), vMul(n2, Math.sin(a)));
        const p = vAdd(centre, vMul(dir, r));
        pos[o] = p.x; pos[o + 1] = p.y; pos[o + 2] = p.z;
        nrm[o] = dir.x; nrm[o + 1] = dir.y; nrm[o + 2] = dir.z;
        o += 3;
      }
    }
    // caps
    const capA = pa, capB = vAdd(pa, d);
    pos[o] = capA.x; pos[o + 1] = capA.y; pos[o + 2] = capA.z;
    const nA = vMul(u, -1);
    nrm[o] = nA.x; nrm[o + 1] = nA.y; nrm[o + 2] = nA.z; o += 3;

    pos[o] = capB.x; pos[o + 1] = capB.y; pos[o + 2] = capB.z;
    nrm[o] = tangentLast.x; nrm[o + 1] = tangentLast.y; nrm[o + 2] = tangentLast.z;
  }
}

export default { Apex6Map, Apex6Robot, Apex6Cables };
