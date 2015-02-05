'use strict'

module.exports = createOrbitController

var filterVector = require('filtered-vector')

function OrbitCameraController() {
}

var proto = OrbitCameraController.prototype

proto._recalcMatrix = function(t) {
}

proto.dirty = function() {
}

proto.get = function(matrix) {
}

proto.tick = function(t) {
}

proto.idle = function(t) {
}

proto.flush = function(t) {
}

proto.zoom = function(t, dr) {
}

proto.pan = function(t, dx, dy) {
}

proto.rotate = function(t, dx, dy) {
}

proto.lookAt = function(t, center, eye, up) {
}

proto.setMatrix = function(t, matrix) {
}

function createOrbitController(options) {
  options = options || {}
}