// Math library, TODO move to another file:
function vec(x, y, z) {
  return { x: x, y: y, z: z };
}
function getAngleABabAC(a, b, AC) {
  const bb = -2 * a * Math.cos(AC);
  const sdelta = Math.sqrt(bb * bb - 4 * (a * a - b * b));
  const c = (-bb - sdelta) / 2;
  return Math.acos((a * a + b * b - c * c) / a / b / 2);
}

function getAngleACabAB(a, b, ab) {
  const c2 = a * a + b * b - 2 * a * b * Math.cos(ab);
  const c = Math.sqrt(c2);
  const ac = Math.acos((a * a + c2 - b * b) / (2 * a * c));
  return ac;
}

function getNorm(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}
function getUnit(v) {
  const n = getNorm(v);
  return {
    x: v.x / n,
    y: v.y / n,
    z: v.z / n,
  };
}
function cross(v1, v2) {
  return {
    x: v1.y * v2.z - v1.z * v2.y,
    y: v1.z * v2.x - v1.x * v2.z,
    z: v1.x * v2.y - v1.y * v2.x,
  };
}
function dot(v1, v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

function substract(v1, v2) {
  return { x: v1.x - v2.x, y: v1.y - v2.y, z: v1.z - v2.z };
}

function add(v1, v2) {
  return { x: v1.x + v2.x, y: v1.y + v2.y, z: v1.z + v2.z };
}

function mult(k, v) {
  return { x: k * v.x, y: k * v.y, z: k * v.z };
}

function getOnPlane(v, v1, v2) {
  // return on plane component of vector v to the plane def by v1 and v2
  const u = getUnit(cross(v1, v2));
  return substract(v, mult(dot(v, u), u));
}

function getNormalToVec(v, v1) {
  // return component of v perpendicular to v1
  const u = getUnit(v1);
  return substract(v, mult(dot(v, u), u));
}

function rotate(v, u, th) {
  // rotate v vector in u unit vector axis of rotation in th radians
  const c = Math.cos(th);
  const s = Math.sin(th);
  const cc = 1 - c;
  return {
    x:
      (c + u.x * u.x * cc) * v.x +
      (u.x * u.y * cc - u.z * s) * v.y +
      (u.x * u.z * cc + u.y * s) * v.z,
    y:
      (u.y * u.x * cc + u.z * s) * v.x +
      (c + u.y * u.y * cc) * v.y +
      (u.y * u.z * cc - u.x * s) * v.z,
    z:
      (u.z * u.x * cc - u.y * s) * v.x +
      (u.z * u.y * cc + u.x * s) * v.y +
      (c + u.z * u.z * cc) * v.z,
  };
}

// Frame rotation matrix in x axis
// Fa ---> x, theta ---> Fb
// returns Aa2b
function rot1mat(theta){
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [1,0,0],
    [0,c,s],
    [0,-s,c],
  ]
}

// Frame rotation matrix in y axis
// Fa ---> y, theta ---> Fb
// returns Aa2b
function rot2mat(theta){
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c,0,-s],
    [0,1,0],
    [s,0,c],
  ]
}

// Frame rotation matrix in z axis
// Fa ---> z, theta ---> Fb
// returns Aa2b
function rot3mat(theta){
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c,s,0],
    [-s,c,0],
    [0,0,1],
  ]
}
// quaternion and kinematics:
function quat(qx, qy, qz, qs) {
  return {
    x: qx,
    y: qy,
    z: qz,
    s: qs,
  };
}

function qtn2A(q) {
  return [
    [
      q.x * q.x - q.y * q.y - q.z * q.z + q.s * q.s,
      2 * (q.x * q.y + q.z * q.s),
      2 * (q.x * q.z - q.y * q.s),
    ],
    [
      2 * (q.x * q.y - q.z * q.s),
      -q.x * q.x + q.y * q.y - q.z * q.z + q.s * q.s,
      2 * (q.y * q.z + q.x * q.s),
    ],
    [
      2 * (q.x * q.z + q.y * q.s),
      2 * (q.y * q.z - q.x * q.s),
      -q.x * q.x - q.y * q.y + q.z * q.z + q.s * q.s,
    ],
  ];
}

function MxM(M1, M2) {
  return [
    [
      M1[0][0] * M2[0][0] + M1[0][1] * M2[1][0] + M1[0][2] * M2[2][0],
      M1[0][0] * M2[0][1] + M1[0][1] * M2[1][1] + M1[0][2] * M2[2][1],
      M1[0][0] * M2[0][2] + M1[0][1] * M2[1][2] + M1[0][2] * M2[2][2],
    ],
    [
      M1[1][0] * M2[0][0] + M1[1][1] * M2[1][0] + M1[1][2] * M2[2][0],
      M1[1][0] * M2[0][1] + M1[1][1] * M2[1][1] + M1[1][2] * M2[2][1],
      M1[1][0] * M2[0][2] + M1[1][1] * M2[1][2] + M1[1][2] * M2[2][2],
    ],
    [
      M1[2][0] * M2[0][0] + M1[2][1] * M2[1][0] + M1[2][2] * M2[2][0],
      M1[2][0] * M2[0][1] + M1[2][1] * M2[1][1] + M1[2][2] * M2[2][1],
      M1[2][0] * M2[0][2] + M1[2][1] * M2[1][2] + M1[2][2] * M2[2][2],
    ]
  ];
}

// matrix vector multiplication
function Mxv(M, v) {
  return {
    x: M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z,
    y: M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z,
    z: M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z,
  };
}

// matrix scalar multiplication
function Mxs(M, s) {
  return [
    [M[0][0] * s, M[0][1] * s, M[0][2] * s],
    [M[1][0] * s, M[1][1] * s, M[1][2] * s],
    [M[2][0] * s, M[2][1] * s, M[2][2] * s]
  ];
}

// matrix transpose
function MT(M) {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]],
  ];
}

// construct matrix by unit vectors:
function Mbyv(u1, u2, u3) {
  return [
    [u1.x, u2.x, u3.x],
    [u1.y, u2.y, u3.y],
    [u1.z, u2.z, u3.z],
  ];
}

function eye() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

module.exports = {
  vec,
  getAngleABabAC,
  getAngleACabAB,
  getNorm,
  getUnit,
  cross,
  dot,
  substract,
  add,
  mult,
  getOnPlane,
  getNormalToVec,
  rotate,
  rot1mat,
  rot2mat,
  rot3mat,
  quat,
  qtn2A,
  MxM,
  Mxv,
  Mxs,
  MT,
  Mbyv,
  eye
}