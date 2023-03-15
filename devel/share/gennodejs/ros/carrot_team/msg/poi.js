// Auto-generated. Do not edit!

// (in-package carrot_team.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class poi {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.poi = null;
    }
    else {
      if (initObj.hasOwnProperty('poi')) {
        this.poi = initObj.poi
      }
      else {
        this.poi = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type poi
    // Serialize message field [poi]
    // Serialize the length for message field [poi]
    bufferOffset = _serializer.uint32(obj.poi.length, buffer, bufferOffset);
    obj.poi.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type poi
    let len;
    let data = new poi(null);
    // Deserialize message field [poi]
    // Deserialize array length for message field [poi]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.poi = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.poi[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.poi.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'carrot_team/poi';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd4ec992df6b6add1e81dc8da9c38c0ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    geometry_msgs/Point[] poi
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new poi(null);
    if (msg.poi !== undefined) {
      resolved.poi = new Array(msg.poi.length);
      for (let i = 0; i < resolved.poi.length; ++i) {
        resolved.poi[i] = geometry_msgs.msg.Point.Resolve(msg.poi[i]);
      }
    }
    else {
      resolved.poi = []
    }

    return resolved;
    }
};

module.exports = poi;
