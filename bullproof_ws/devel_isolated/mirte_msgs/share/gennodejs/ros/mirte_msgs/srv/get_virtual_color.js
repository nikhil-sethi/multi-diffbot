// Auto-generated. Do not edit!

// (in-package mirte_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

let color = require('../msg/color.js');

//-----------------------------------------------------------

class get_virtual_colorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.direction = null;
    }
    else {
      if (initObj.hasOwnProperty('direction')) {
        this.direction = initObj.direction
      }
      else {
        this.direction = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_virtual_colorRequest
    // Serialize message field [direction]
    bufferOffset = _serializer.string(obj.direction, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_virtual_colorRequest
    let len;
    let data = new get_virtual_colorRequest(null);
    // Deserialize message field [direction]
    data.direction = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.direction);
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mirte_msgs/get_virtual_colorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '326e9417def5db9a05a2704a4d8de15e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string direction
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_virtual_colorRequest(null);
    if (msg.direction !== undefined) {
      resolved.direction = msg.direction;
    }
    else {
      resolved.direction = ''
    }

    return resolved;
    }
};

class get_virtual_colorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.data = null;
    }
    else {
      if (initObj.hasOwnProperty('data')) {
        this.data = initObj.data
      }
      else {
        this.data = new color();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type get_virtual_colorResponse
    // Serialize message field [data]
    bufferOffset = color.serialize(obj.data, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type get_virtual_colorResponse
    let len;
    let data = new get_virtual_colorResponse(null);
    // Deserialize message field [data]
    data.data = color.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a service object
    return 'mirte_msgs/get_virtual_colorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '2f4914cb5eb412d500cdee0301977e80';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    mirte_msgs/color data
    
    
    ================================================================================
    MSG: mirte_msgs/color
    int32 red
    int32 green
    int32 blue
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new get_virtual_colorResponse(null);
    if (msg.data !== undefined) {
      resolved.data = color.Resolve(msg.data)
    }
    else {
      resolved.data = new color()
    }

    return resolved;
    }
};

module.exports = {
  Request: get_virtual_colorRequest,
  Response: get_virtual_colorResponse,
  md5sum() { return '8ce5cc0c46b1c53043e61c14e381c577'; },
  datatype() { return 'mirte_msgs/get_virtual_color'; }
};
