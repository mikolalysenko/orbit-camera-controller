'use strict'

module.exports = createOrbitController

var filterVector  = require('filtered-vector')
var lookAt        = require('gl-mat4/lookAt')
var mat4FromQuat  = require('gl-mat4/fromQuat')
var invert44      = require('gl-mat4/invert')
var quatFromFrame = require('./lib/quatFromFrame')

function len3(x,y,z) {
  return Math.sqrt(Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2))
}

function len4(w,x,y,z) {
  return Math.sqrt(Math.pow(w,2) + Math.pow(x,2) + Math.pow(y,2) + Math.pow(z,2))
}

function normalize4(out, a) {
  var ax = a[0]
  var ay = a[1]
  var az = a[2]
  var aw = a[3]
  var al = len4(ax, ay, az, aw)
  if(al > 1e-6) {
    out[0] = ax/al
    out[1] = ay/al
    out[2] = az/al
    out[3] = aw/al
  } else {
    out[0] = out[1] = out[2] = 0.0
    out[3] = 1.0
  }
}


function OrbitCameraController(initQuat, initCenter, initRadius) {
  this.radius    = filterVector([initRadius])
  this.center    = filterVector(initCenter)
  this.rotation  = filterVector(initQuat)

  this.computedRadius   = this.radius.curve(0)
  this.computedCenter   = this.center.curve(0)
  this.computedRotation = this.rotation.curve(0)
  this.computedUp       = [0.1,0,0]
  this.computedEye      = [0.1,0,0]
  this.computedMatrix   = [0.1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

  this._recalcMatrix(0)
  this._lastTick = 0
  this._isDirty  = true
}

var proto = OrbitCameraController.prototype

proto._recalcMatrix = function(t) {
  this.radius.curve(t)
  this.center.curve(t)
  this.rotation.curve(t)

  var quat = this.computedRotation
  normalize4(quat, quat)

  var mat = this.computedMatrix
  mat4FromQuat(mat, quat)

  var center = this.computedCenter
  var eye    = this.computedEye
  var up     = this.computedUp
  var radius = Math.exp(this.computedRadius[0])
  eye[0] = center[0] - radius * mat[2]
  eye[1] = center[1] - radius * mat[6]
  eye[2] = center[2] - radius * mat[10]
  up[0] = mat[1]
  up[1] = mat[5]
  up[2] = mat[9]
  for(var i=0; i<3; ++i) {
    var rr = 0.0
    for(var j=0; j<3; ++j) {
      rr += mat[i+4*j] * eye[j]
    }
    mat[12+i] = rr
  }
}

proto.dirty = function() {
  return this._isDirty
}

proto.get = function(result) {
  this._isDirty = false
  var m = this.computedMatrix
  if(!result) {
    return m
  }
  for(var i=0; i<16; ++i) {
    result[i] = m[i]
  }
  return result
}

proto.tick = function(t) {
  this._isDirty = true
  this._lastTick = t
  this._recalcMatrix(t)
}

proto.idle = function(t) {
  this.center.idle(t)
  this.radius.idle(t)
  this.rotation.idle(t)
}

proto.flush = function(t) {
  this.center.flush(t)
  this.radius.flush(t)
  this.rotation.flush(t)
}

proto.zoom = function(t, dr) {
  if(dr > 0.0) {
    this.radius.move(t, Math.log(dr))
  }
}

proto.pan = function(t, dx, dy) {
  this._recalcMatrix(t)

  var mat = this.computedMatrix
  var center = this.computedCenter
  var cx = center[0] - (dx * mat[0] + dy * mat[1])
  var cy = center[1] - (dx * mat[4] + dy * mat[5])
  var cz = center[2] - (dx * mat[8] + dy * mat[9])

  this.center.push(t, cx, cy, cz)

  this._recalcMatrix(this._lastTick)
}

proto.rotate = function(t, dx, dy) {
  this._recalcMatrix(t)

  var mat = this.computedMatrix

  var rx = mat[0]
  var ry = mat[4]
  var rz = mat[8]

  var ux = mat[1]
  var uy = mat[5]
  var uz = mat[9]

  var fx = mat[2]
  var fy = mat[6]
  var fz = mat[10]

  var qx = dx * rx + dy * ux
  var qy = dx * ry + dy * uy
  var qz = dx * rz + dy * uz
  
  var bx = (fy * qz - fz * qy)
  var by = (fz * qx - fx * qz)
  var bz = (fx * qy - fy * qx)
  var bw = Math.sqrt(Math.max(0.0, 1.0 - Math.pow(bx,2) - Math.pow(by,2) - Math.pow(bz,2)))
  var bl = len4(bx, by, bz, bw)
  if(bl > 1e-6) {
    bx /= bl
    by /= bl
    bz /= bl
    bw /= bl
  } else {
    bx = by = bz = 0.0
    bw = 1.0
  }

  var rotation = this.computedRotation
  var ax = rotation[0]
  var ay = rotation[1]
  var az = rotation[2]
  var aw = rotation[3]

  var cx = ax*bw + aw*bx + ay*bz - az*by
  var cy = ay*bw + aw*by + az*bx - ax*bz
  var cz = az*bw + aw*bz + ax*by - ay*bx
  var cw = aw*bw - ax*bx - ay*by - az*bz
  var cl = len4(cx, cy, cz, cw)
  if(cl > 1e-6) {
    cx /= cl
    cy /= cl
    cz /= cl
    cw /= cl
  } else {
    cx = cy = cz = 0.0
    cw = 1.0
  }

  this.rotation.push(t, cx, cy, cz, cw)

  this._recalcMatrix(this._lastTick)
}

proto.lookAt = function(t, eye, center, up) {
  this._recalcMatrix(t)

  center = center || this.computedCenter
  eye    = eye    || this.computedEye
  up     = up     || this.computedUp

  var mat = this.computedMatrix
  lookAt(mat, center, eye, up)

  var fx = eye[0] - center[0]
  var fy = eye[1] - center[1]
  var fz = eye[2] - center[2]
  var fl = len3(fx, fy, fz)
  fx /= fl
  fy /= fl
  fz /= fl

  var ux = up[0]
  var uy = up[1]
  var uz = up[2]
  var fu = ux * fx + uy * fy + uz * fz
  ux -= fu * fx
  uy -= fu * fy
  uz -= fu * fz
  var ul = len3(ux, uy, uz)
  ux /= ul
  uy /= ul
  uz /= ul

  var rx = uy * fz - uz * fy
  var ry = uz * fx - ux * fz
  var rz = ux * fy - fy * ux
  var rl = len3(rx, ry, rz)
  rx /= rl
  ry /= rl
  rz /= rl

  var rotation = this.computedRotation
  quatFromFrame(rotation,
    rx, ry, rz,
    ux, uy, uz,
    fx, fy, fz)
  normalize4(rotation, rotation)
  this.rotation.set(t, rotation[0], rotation[1], rotation[2], rotation[3])

  var radius = 0.0
  for(var i=0; i<3; ++i) {
    radius += Math.pow(center[i] - eye[i], 2)
  }
  radius = Math.sqrt(radius)
  this.radius.set(t, Math.log(radius))

  this.center.set(t, center[0], center[1], center[2])
  console.log(center.join())
  
  this._recalcMatrix(this._lastTick)
}

proto.setMatrix = function(t, matrix) {
}

function createOrbitController(options) {
  options = options || {}
  var center   = options.center   || [0,0,0]
  var rotation = options.rotation || [0,0,0,1]
  var radius   = options.radius   || 50.0

  center = [].slice.call(center, 0, 3)
  rotation = [].slice.call(rotation, 0, 4)

  if('eye' in options) {
    //Compute rotation from eye
  }

  normalize4(rotation, rotation)

  return new OrbitCameraController(
    rotation,
    center,
    Math.log(radius))
}