// Auto-generated. Do not edit!

// (in-package armc_visual.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class VisualVarRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.a = null;
    }
    else {
      if (initObj.hasOwnProperty('a')) {
        this.a = initObj.a
      }
      else {
        this.a = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualVarRequest
    // Serialize message field [a]
    bufferOffset = _serializer.int32(obj.a, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualVarRequest
    let len;
    let data = new VisualVarRequest(null);
    // Deserialize message field [a]
    data.a = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armc_visual/VisualVarRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '5c9fb1a886e81e3162a5c87bf55c072b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 a
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisualVarRequest(null);
    if (msg.a !== undefined) {
      resolved.a = msg.a;
    }
    else {
      resolved.a = 0
    }

    return resolved;
    }
};

class VisualVarResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.T = null;
      this.flag = null;
    }
    else {
      if (initObj.hasOwnProperty('T')) {
        this.T = initObj.T
      }
      else {
        this.T = new Array(12).fill(0);
      }
      if (initObj.hasOwnProperty('flag')) {
        this.flag = initObj.flag
      }
      else {
        this.flag = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type VisualVarResponse
    // Check that the constant length array field [T] has the right length
    if (obj.T.length !== 12) {
      throw new Error('Unable to serialize array field T - length must be 12')
    }
    // Serialize message field [T]
    bufferOffset = _arraySerializer.float32(obj.T, buffer, bufferOffset, 12);
    // Serialize message field [flag]
    bufferOffset = _serializer.bool(obj.flag, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type VisualVarResponse
    let len;
    let data = new VisualVarResponse(null);
    // Deserialize message field [T]
    data.T = _arrayDeserializer.float32(buffer, bufferOffset, 12)
    // Deserialize message field [flag]
    data.flag = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 49;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armc_visual/VisualVarResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '92e9ecd5f21b90ca055ac7ee64822e61';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32[12] T
    bool flag
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new VisualVarResponse(null);
    if (msg.T !== undefined) {
      resolved.T = msg.T;
    }
    else {
      resolved.T = new Array(12).fill(0)
    }

    if (msg.flag !== undefined) {
      resolved.flag = msg.flag;
    }
    else {
      resolved.flag = false
    }

    return resolved;
    }
};

module.exports = {
  Request: VisualVarRequest,
  Response: VisualVarResponse,
  md5sum() { return 'd56d25bda35878684a98158f8a53e101'; },
  datatype() { return 'armc_visual/VisualVar'; }
};
