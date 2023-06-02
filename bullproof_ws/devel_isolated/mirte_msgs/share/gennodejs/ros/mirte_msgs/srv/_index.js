
"use strict";

let Turn = require('./Turn.js')
let SetOLEDImage = require('./SetOLEDImage.js')
let SetMotorSpeed = require('./SetMotorSpeed.js')
let GetKeypad = require('./GetKeypad.js')
let SetPinMode = require('./SetPinMode.js')
let GetPinValue = require('./GetPinValue.js')
let SetLEDValue = require('./SetLEDValue.js')
let GetEncoder = require('./GetEncoder.js')
let GetIntensity = require('./GetIntensity.js')
let get_barcode = require('./get_barcode.js')
let GetDistance = require('./GetDistance.js')
let SetServoAngle = require('./SetServoAngle.js')
let get_virtual_color = require('./get_virtual_color.js')
let GetIntensityDigital = require('./GetIntensityDigital.js')
let Move = require('./Move.js')
let SetPinValue = require('./SetPinValue.js')

module.exports = {
  Turn: Turn,
  SetOLEDImage: SetOLEDImage,
  SetMotorSpeed: SetMotorSpeed,
  GetKeypad: GetKeypad,
  SetPinMode: SetPinMode,
  GetPinValue: GetPinValue,
  SetLEDValue: SetLEDValue,
  GetEncoder: GetEncoder,
  GetIntensity: GetIntensity,
  get_barcode: get_barcode,
  GetDistance: GetDistance,
  SetServoAngle: SetServoAngle,
  get_virtual_color: get_virtual_color,
  GetIntensityDigital: GetIntensityDigital,
  Move: Move,
  SetPinValue: SetPinValue,
};
