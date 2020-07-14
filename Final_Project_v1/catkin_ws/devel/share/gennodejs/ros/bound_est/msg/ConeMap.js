// Auto-generated. Do not edit!

// (in-package bound_est.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Conepos = require('./Conepos.js');
let Pos = require('./Pos.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class ConeMap {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.cones = null;
      this.car = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('cones')) {
        this.cones = initObj.cones
      }
      else {
        this.cones = [];
      }
      if (initObj.hasOwnProperty('car')) {
        this.car = initObj.car
      }
      else {
        this.car = new Pos();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ConeMap
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [cones]
    // Serialize the length for message field [cones]
    bufferOffset = _serializer.uint32(obj.cones.length, buffer, bufferOffset);
    obj.cones.forEach((val) => {
      bufferOffset = Conepos.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [car]
    bufferOffset = Pos.serialize(obj.car, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ConeMap
    let len;
    let data = new ConeMap(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [cones]
    // Deserialize array length for message field [cones]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.cones = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.cones[i] = Conepos.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [car]
    data.car = Pos.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += 9 * object.cones.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'bound_est/ConeMap';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '918995278f3905a157d2c2d750f8e21a';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    Conepos[] cones
    Pos car
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: bound_est/Conepos
    float32 x
    float32 y
    int8 color 
    
    ================================================================================
    MSG: bound_est/Pos
    float32 x
    float32 y
    float32 theta
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ConeMap(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.cones !== undefined) {
      resolved.cones = new Array(msg.cones.length);
      for (let i = 0; i < resolved.cones.length; ++i) {
        resolved.cones[i] = Conepos.Resolve(msg.cones[i]);
      }
    }
    else {
      resolved.cones = []
    }

    if (msg.car !== undefined) {
      resolved.car = Pos.Resolve(msg.car)
    }
    else {
      resolved.car = new Pos()
    }

    return resolved;
    }
};

module.exports = ConeMap;
