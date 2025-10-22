/**
 * @typedef {Object} Vector
 * @property {number} x - x component
 * @property {number} y - y component
 * @property {number} z - y component
 */

/**
 * @typedef {Object} Quaternion
 * @property {number} x - x component
 * @property {number} y - y component
 * @property {number} z - y component
 * @property {number} s - scalar component
 */

/**
 * @typedef {number[]} Row3
 * Represents a row in a Matrix3x3.

/**
 * @typedef {Row3[]} Matrix3x3
 * Represents a 3x3 matrix where each element is a row of numbers.
 */


/**
 * Generates 3 dimensional vector 
 *
 * @param {number} x - x coordinate
 * @param {number} y - y coordinate
 * @param {number} z - z coordinate
 * @returns {Vector} new vec object with fields x, y and z
 */
function vec(x, y, z) {
  return { x: x, y: y, z: z };
}

/**
 * Calculates specific angle in a triangle 
 *
 * @param {number} a - length of the edge a
 * @param {number} b - length of the edge b
 * @param {number} AC - angle between the edges a and c in radians
 * @returns {number} angle between edges a and b
 */
function getAngleABabAC(a, b, AC) {
  const bb = -2 * a * Math.cos(AC);
  const sdelta = Math.sqrt(bb * bb - 4 * (a * a - b * b));
  let c = (-bb - sdelta) / 2;
  if (c < 0) // invalid solution
    c =  (-bb + sdelta) / 2;
  return Math.acos((a * a + b * b - c * c) / a / b / 2);
}

/**
 * Calculates length in a triangle 
 *
 * @param {number} a - length of the edge a
 * @param {number} b - length of the edge b
 * @param {number} AC - angle between the edges a and c in radians
 * @returns {number} length of the edge c
 */
function getLencabAC(a, b, AC) {
  const bb = -2 * a * Math.cos(AC);
  const sdelta = Math.sqrt(bb * bb - 4 * (a * a - b * b));
  let c = (-bb - sdelta) / 2;
  return c
}

/**
 * Calculates specific angle in a triangle 
 * getAngleACabAB(a, b, ab) 
 * 
 * @param {number} a - length of the edge a
 * @param {number} b - length of the edge b
 * @param {number} AB - angle between the edges a and b in radians
 * @returns {number} angle between edges a and c
 */
function getAngleACabAB(a, b, ab) {
  const c2 = a * a + b * b - 2 * a * b * Math.cos(ab);
  const c = Math.sqrt(c2);
  const ac = Math.acos((a * a + c2 - b * b) / (2 * a * c));
  return ac;
}

/**
 * Calculates norm of a vector
 *
 * @param {Vector} v - vector v
 * @returns {Vector} v - norm of new object
 */
function getNorm(v) {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

/**
 * Calculates unit vector of a vector
 * getUnit(v)
 *
 * @param {Vector} v - vector v
 * @returns {Vector} unit vector of v as new object
 */
function getUnit(v) {
  const n = getNorm(v);
  return {
    x: v.x / n,
    y: v.y / n,
    z: v.z / n,
  };
}

/**
 * Calculates cross product of vectors v1 and v2
 * cross(v1,v2)
 *
 * @param {Vector} v1 - vector v1
 * @param {Vector} v2 - vector v2
 * @returns {Vector} (v1 x v2) as new object
 */
function cross(v1, v2) {
  return {
    x: v1.y * v2.z - v1.z * v2.y,
    y: v1.z * v2.x - v1.x * v2.z,
    z: v1.x * v2.y - v1.y * v2.x,
  };
}


/**
 * Calculates dot product of vectors v1 and v2
 * dot(v1,v2)
 *
 * @param {Vector} v1 - vector v1
 * @param {Vector} v2 - vector v2
 * @returns {number} (v1 dot v2) as new object
 */
function dot(v1, v2) {
  return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

/**
 * Calculates difference of vectors v1 and v2
 * substract(v1,v2)
 *
 * @param {Vector} v1 - vector v1
 * @param {Vector} v2 - vector v2
 * @returns {Vector} (v1 - v2) as new object
 */
function substract(v1, v2) {
  return { x: v1.x - v2.x, y: v1.y - v2.y, z: v1.z - v2.z };
}

/**
 * Calculates sum of vectors v1 and v2
 * add(v1,v2)
 *
 * @param {Vector} v1 - vector v1
 * @param {Vector} v2 - vector v2
 * @returns {Vector} (v1 + v2) as new object
 */
function add(v1, v2) {
  return { x: v1.x + v2.x, y: v1.y + v2.y, z: v1.z + v2.z };
}

/**
 * Calculates vector multiplied by a scalar k
 * mult(k, v)
 *
 * @param {number} k - scalar
 * @param {Vector} v - vector
 * @returns {Vector} (kv) as new object
 */
function mult(k, v) {
  return { x: k * v.x, y: k * v.y, z: k * v.z };
}

/**
 * Calculates on plane component of vector v to the plane defined by v1 and v2
 * getOnPlane(v, v1, v2)
 *
 * @param {Vector} v - the vector to decompose
 * @param {Vector} v1 - first vector on the plane
 * @param {Vector} v2 - second vector on the plane
 * @returns {Vector} on plane component of the vector
 */
function getOnPlane(v, v1, v2) {
  // return 
  const u = getUnit(cross(v1, v2));
  return substract(v, mult(dot(v, u), u));
}

/**
 * Calculates normal component of a vector v to a vector v1,
 * e.g., on plane component of a vector v to a plane defined by a plane normal vector v1
 *
 * @param {Vector} v - the vector to decompose
 * @param {Vector} v1 - plane normal vector
 * @returns {Vector} normal component of the vector
 */
function getNormalToVec(v, v1) {
  // return component of v perpendicular to v1
  const u = getUnit(v1);
  return substract(v, mult(dot(v, u), u));
}

/**
 * rotate v vector in u unit vector axis of rotation in th radians
 *
 * @param {Vector} v - the vector to be rotated
 * @param {Vector} u - axis of rotation (has to be unit vector)
 * @param {number} th - angle to rotate in radians
 * @returns {Vector} rotated vector as new object
 */
function rotate(v, u, th) {
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


/**
 * Frame rotation matrix in x axis
 * Fa ---> x, theta ---> Fb
 * returns Aa2b
 * rotate(v, u, th)
 *
 * @param {number} th - angle to rotate in radians
 * @returns {Matrix3x3} rotation matrix Aa2b
 */
function rot1mat(theta){
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [1,0,0],
    [0,c,s],
    [0,-s,c],
  ]
}

/**
 * Frame rotation matrix in y axis
 * Fa ---> y, theta ---> Fb
 * returns Aa2b
 * rotate(v, u, th)
 *
 * @param {number} th - angle to rotate in radians
 * @returns {Matrix3x3} rotation matrix Aa2b
 */
function rot2mat(theta){
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c,0,-s],
    [0,1,0],
    [s,0,c],
  ]
}

/**
 * Frame rotation matrix in z axis
 * Fa ---> z, theta ---> Fb
 * returns Aa2b
 * rotate(v, u, th)
 *
 * @param {number} th - angle to rotate in radians
 * @returns {Matrix3x3} rotation matrix Aa2b
 */
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

/**
 * Define a quaternion object with fields (x,y,z,s)
 * quat(qx, qy, qz, qs) 
 * @param {number} qx - x coordinate
 * @param {number} qy - y coordinate
 * @param {number} qz - z coordinate
 * @param {number} qs - scalar part
 * @returns {Quaternion} the quaternion object
 */
function quat(qx, qy, qz, qs) {
  return {
    x: qx,
    y: qy,
    z: qz,
    s: qs,
  };
}

/**
 * Define a rotation matrix from quaternion
 * qtn2A(q)
 * @param {Quaternion} q - quaternion
 * @returns {Matrix3x3} the rotation matrix
 */
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


// Matrix operations

/**
 * 3x3 Matrix multiplication M1xM2
 * MxM(M1, M2)
 * @param {Array} M1 - 3x3 M1 matrix
 * @param {Matrix3x3} M2 - M2 matrix
 * @returns {Matrix3x3} M1 x M2
 */
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

/**
 * 3x3 Matrix 3x1 vector  multiplication Mxv
 * Mxv(M, v)
 * @param {Matrix3x3} M - M matrix
 * @param {Vector} v - v vector
 * @returns {Vector} M x v
 */
function Mxv(M, v) {
  return {
    x: M[0][0] * v.x + M[0][1] * v.y + M[0][2] * v.z,
    y: M[1][0] * v.x + M[1][1] * v.y + M[1][2] * v.z,
    z: M[2][0] * v.x + M[2][1] * v.y + M[2][2] * v.z,
  };
}

/**
 * 3x3 matrix scalar multiplication
 * Mxs(M, s)
 * @param {Matrix3x3} M - M matrix
 * @param {number} s - s scalar
 * @returns {Matrix3x3} M x s
 */
function Mxs(M, s) {
  return [
    [M[0][0] * s, M[0][1] * s, M[0][2] * s],
    [M[1][0] * s, M[1][1] * s, M[1][2] * s],
    [M[2][0] * s, M[2][1] * s, M[2][2] * s]
  ];
}

/**
 * 3x3 matrix transpose
 * MT(M)
 * @param {Matrix3x3} M - M matrix
 * @returns {Matrix3x3} transpose of M
 */
function MT(M) {
  return [
    [M[0][0], M[1][0], M[2][0]],
    [M[0][1], M[1][1], M[2][1]],
    [M[0][2], M[1][2], M[2][2]],
  ];
}

/**
 * Construct an 3x3 Matrix by three 3x1 vector [u1,u2,u3]
 * e.g., vectors are concatenated horizontally
 * Mbyv(u1, u2, u3) 
 * @param {Vector} u1 - first column of matrix
 * @param {Vector} u2 - second column of matrix
 * @param {Vector} u3 - third column of matrix
 * @returns {Matrix3x3} the 3x3 matrix
 */
function Mbyv(u1, u2, u3) {
  return [
    [u1.x, u2.x, u3.x],
    [u1.y, u2.y, u3.y],
    [u1.z, u2.z, u3.z],
  ];
}


/**
 * Construct an 3x3 Identity Matrix
 * eye()
 * @returns {Matrix3x3} the 3x3 identity matrix
 */
function eye() {
  return [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
}

/**
 * Returns angle between two vectors in radians in 0-pi
 * Inputs must not have to be unit vector
 * @param {Vector} v1 - first vector
 * @param {Vector} v2 - second vector
 * @returns {number} angle in radians
 */
function getAngle(v1, v2) {
  return Math.acos(dot(getUnit(v1), getUnit(v2)));
}

module.exports = {
  vec,
  getAngleABabAC,
  getLencabAC,
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
  eye,
  getAngle
}