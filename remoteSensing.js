const v = require("./vector")


function getGeodeticNormal(ecef){  
  const R_EQ = 6378.137; // equatorial radius km
  const R_POLAR = R_EQ * (1 - 1 / 298.257223563); // polar radius km R_EQ*(1-flattening)
  let n = v.vec(
    (R_POLAR / R_EQ) * ecef.x,
    (R_POLAR / R_EQ) * ecef.y,
    (R_EQ / R_POLAR) * ecef.z
  );
  n = v.getUnit(n);
  return n;
};


module.exports = {

getEarthIntersect: function(uce, re) {
  const a = 6378.137; // equatorial radius km
  const b = a * (1 - 1 / 298.257223563); // polar radius km R_EQ*(1-flattening)

  const d2 = uce.x * uce.x + uce.y * uce.y + ((a * a) / b / b) * uce.z * uce.z;
  const d1 =
    2 * (uce.x * re.x + uce.y * re.y + ((a * a) / b / b) * uce.z * re.z);
  const d0 =
    re.x * re.x + re.y * re.y + ((a * a) / b / b) * re.z * re.z - a * a;
  const disc = d1 * d1 - 4 * d0 * d2;
  // console.log(uce,re,d2,d1,d0,disc)
  if (disc <= 0) {
    console.log("vector is not pointing to the ellipsoid body!");
    return null;
  } else {
    const k = (-d1 - Math.sqrt(disc)) / 2 / d2;
    if (k < 0) {
      console.log("negative vector is pointing to the ellipsoid body!");
      return null;
    }
    return v.add(re, v.mult(k, uce));
  }
},

getOrbit2Ecef: function(re, ve) {
  // note: tirs2itrf conversion may be used for more accurate orbit 2 ecef conversation
  const weie = v.vec(
    0,
    0,
    (2 * Math.PI) / (23 * 60 * 60 + 56 * 60 + 4.098903691)
  );
  const D_I_rSatEcef = v.add(ve, v.cross(weie, re));
  const u3o_e = v.mult(-1, v.getUnit(re));
  const u2o_e = v.getUnit(v.cross(u3o_e, D_I_rSatEcef));
  const u1o_e = v.getUnit(v.cross(u2o_e, u3o_e));
  return v.Mbyv(u1o_e, u2o_e, u3o_e);
},

// return lat lng object in degrees
// algorithm is exact when input is on the ellipsoid
// if satellite ecef is used iterational model should be implemented
ecef2ll: function(ecef) {
  const n = getGeodeticNormal(ecef);
  return {
    lat: (Math.asin(n.z) / Math.PI) * 180,
    lng: (Math.atan2(n.y, n.x) / Math.PI) * 180,
  };
},

getGeodeticNormal,

// haversine formula for distance in great circle
getHaversineDistance: function(lat1,lon1,lat2,lon2){
    const slat = Math.sin((lat2 - lat1)/2);
    const slon = Math.sin((lon2 - lon1)/2);
    return 2*6378.137*Math.asin( Math.sqrt(
        slat* slat + Math.cos(lat1)*Math.cos(lat2)*slon*slon));
},

// ecef2lla  - convert earth-centered earth-fixed (ECEF)
//             in cartesian coordinates (km)
//             to geodetic lat (deg), lon (deg) and altitude (m)
//
// Notes: (1) This function assumes the WGS84 model.
//        (2) Latitude is customary geodetic (not geocentric).
//        (3) Inputs may be scalars, vectors, or matrices of the same
//            size and shape. Outputs will have that same size and shape.
//        (4) Tested but no warranty; use at your own risk.
//        (5) Michael Kleder, April 2006
ecef2lla: function(rEcef){

  // conver to meters
const x = rEcef.x*1000;
const y = rEcef.y*1000;
const z = rEcef.z*1000;

// WGS84 ellipsoid constants:
const a = 6378137;
const e = 8.1819190842622e-2;

// calculations:
const b   = Math.sqrt(a*a*(1-e*e));
const ep  = Math.sqrt((a*a-b*b)/(b*b));
const p   = Math.sqrt(x*x+y*y);
const th  = Math.atan2(a*z,b*p);
let lon = Math.atan2(y,x);
let lat = Math.atan2((z+ep*ep*b*Math.sin(th)**3),(p-e*e*a*Math.cos(th)**3));
const N   = a/Math.sqrt(1-e*e*Math.sin(lat)**2);
let alt = p/Math.cos(lat)-N;

// return lon in range [0,2*pi)
lon = (lon)%(2*Math.PI);

// correct for numerical instability in altitude near exact poles:
// (after this correction, error is about 2 millimeters, which is about
// the same as the numerical precision of the overall function)

if (Math.abs(x)<1 & Math.abs(y)<1)
  alt = (Math.abs(z)-b);


// return lat lon alt
return {
  lat: lat*180/Math.PI, // in degrees
  lng: lon*180/Math.PI, // in degrees
  alt: alt // in meters
}
},

/**
 * 
 * @param {Object} lla 
 * @param {number} lla.lat - Geodetic latitude in degrees
 * @param {number} lla.lng - Longitude in degrees
 * @param {number} lla.alt - Altitude above reference ellipsoid in m
 * @returns {Vector} Cartesian coordinates
 */
// lla2ecef -convert Lat Lng Alt to ECEF x, y, z coordinates
lla2ecef: function(lla){
  
  const rad = 180.0/Math.PI;
  const latgd = lla.lat / rad;
  const lon = lla.lng / rad;
  const alt = lla.alt / 1000.0; // m -> km


  const a = 6378.1363;
  const e = 0.08181919084262149;
  
  // intermediate calculation
  // prime vertical radius of curvature
  const N = a / Math.sqrt(1 - e*e*Math.sin(latgd)**2)

  return v.vec(
    (N + alt) * Math.cos(latgd) * Math.cos(lon),
    (N + alt) * Math.cos(latgd) * Math.sin(lon),
    (N * (1-e*e) + alt) * Math.sin(latgd));
}
}