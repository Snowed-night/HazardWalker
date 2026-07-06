// Auto-generated. Do not edit!

// (in-package building_generator_interfaces.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class CallElevatorRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.elevator_id = null;
      this.target_floor = null;
      this.open_doors = null;
    }
    else {
      if (initObj.hasOwnProperty('elevator_id')) {
        this.elevator_id = initObj.elevator_id
      }
      else {
        this.elevator_id = '';
      }
      if (initObj.hasOwnProperty('target_floor')) {
        this.target_floor = initObj.target_floor
      }
      else {
        this.target_floor = 0;
      }
      if (initObj.hasOwnProperty('open_doors')) {
        this.open_doors = initObj.open_doors
      }
      else {
        this.open_doors = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CallElevatorRequest
    // Serialize message field [elevator_id]
    bufferOffset = _serializer.string(obj.elevator_id, buffer, bufferOffset);
    // Serialize message field [target_floor]
    bufferOffset = _serializer.int32(obj.target_floor, buffer, bufferOffset);
    // Serialize message field [open_doors]
    bufferOffset = _serializer.bool(obj.open_doors, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CallElevatorRequest
    let len;
    let data = new CallElevatorRequest(null);
    // Deserialize message field [elevator_id]
    data.elevator_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [target_floor]
    data.target_floor = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [open_doors]
    data.open_doors = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.elevator_id);
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'building_generator_interfaces/CallElevatorRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0d444968ff6f7638a0dbaaddb632567';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string elevator_id
    int32 target_floor
    bool open_doors
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CallElevatorRequest(null);
    if (msg.elevator_id !== undefined) {
      resolved.elevator_id = msg.elevator_id;
    }
    else {
      resolved.elevator_id = ''
    }

    if (msg.target_floor !== undefined) {
      resolved.target_floor = msg.target_floor;
    }
    else {
      resolved.target_floor = 0
    }

    if (msg.open_doors !== undefined) {
      resolved.open_doors = msg.open_doors;
    }
    else {
      resolved.open_doors = false
    }

    return resolved;
    }
};

class CallElevatorResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accepted = null;
      this.current_floor = null;
      this.state = null;
      this.message = null;
    }
    else {
      if (initObj.hasOwnProperty('accepted')) {
        this.accepted = initObj.accepted
      }
      else {
        this.accepted = false;
      }
      if (initObj.hasOwnProperty('current_floor')) {
        this.current_floor = initObj.current_floor
      }
      else {
        this.current_floor = 0;
      }
      if (initObj.hasOwnProperty('state')) {
        this.state = initObj.state
      }
      else {
        this.state = '';
      }
      if (initObj.hasOwnProperty('message')) {
        this.message = initObj.message
      }
      else {
        this.message = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type CallElevatorResponse
    // Serialize message field [accepted]
    bufferOffset = _serializer.bool(obj.accepted, buffer, bufferOffset);
    // Serialize message field [current_floor]
    bufferOffset = _serializer.int32(obj.current_floor, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type CallElevatorResponse
    let len;
    let data = new CallElevatorResponse(null);
    // Deserialize message field [accepted]
    data.accepted = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [current_floor]
    data.current_floor = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [state]
    data.state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [message]
    data.message = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.state);
    length += _getByteLength(object.message);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a service object
    return 'building_generator_interfaces/CallElevatorResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '801c9da3d94557801078ddbfbf5627f7';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool accepted
    int32 current_floor
    string state
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new CallElevatorResponse(null);
    if (msg.accepted !== undefined) {
      resolved.accepted = msg.accepted;
    }
    else {
      resolved.accepted = false
    }

    if (msg.current_floor !== undefined) {
      resolved.current_floor = msg.current_floor;
    }
    else {
      resolved.current_floor = 0
    }

    if (msg.state !== undefined) {
      resolved.state = msg.state;
    }
    else {
      resolved.state = ''
    }

    if (msg.message !== undefined) {
      resolved.message = msg.message;
    }
    else {
      resolved.message = ''
    }

    return resolved;
    }
};

module.exports = {
  Request: CallElevatorRequest,
  Response: CallElevatorResponse,
  md5sum() { return 'b2f9babc7642b34ecdc8d6d6e7e3cebf'; },
  datatype() { return 'building_generator_interfaces/CallElevator'; }
};
