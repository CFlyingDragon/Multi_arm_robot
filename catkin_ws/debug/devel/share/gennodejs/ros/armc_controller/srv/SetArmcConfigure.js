// Auto-generated. Do not edit!

// (in-package armc_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class SetArmcConfigureRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.configure = null;
    }
    else {
      if (initObj.hasOwnProperty('configure')) {
        this.configure = initObj.configure
      }
      else {
        this.configure = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetArmcConfigureRequest
    // Serialize message field [configure]
    bufferOffset = _serializer.string(obj.configure, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetArmcConfigureRequest
    let len;
    let data = new SetArmcConfigureRequest(null);
    // Deserialize message field [configure]
    data.configure = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.configure.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armc_controller/SetArmcConfigureRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'dcee6d46171d1275f3230f04cb248288';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string configure
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetArmcConfigureRequest(null);
    if (msg.configure !== undefined) {
      resolved.configure = msg.configure;
    }
    else {
      resolved.configure = ''
    }

    return resolved;
    }
};

class SetArmcConfigureResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.result = null;
    }
    else {
      if (initObj.hasOwnProperty('result')) {
        this.result = initObj.result
      }
      else {
        this.result = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetArmcConfigureResponse
    // Serialize message field [result]
    bufferOffset = _serializer.bool(obj.result, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetArmcConfigureResponse
    let len;
    let data = new SetArmcConfigureResponse(null);
    // Deserialize message field [result]
    data.result = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'armc_controller/SetArmcConfigureResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'eb13ac1f1354ccecb7941ee8fa2192e8';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool result
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetArmcConfigureResponse(null);
    if (msg.result !== undefined) {
      resolved.result = msg.result;
    }
    else {
      resolved.result = false
    }

    return resolved;
    }
};

module.exports = {
  Request: SetArmcConfigureRequest,
  Response: SetArmcConfigureResponse,
  md5sum() { return 'e4d1173ab6495a30091da13a92937cf6'; },
  datatype() { return 'armc_controller/SetArmcConfigure'; }
};
