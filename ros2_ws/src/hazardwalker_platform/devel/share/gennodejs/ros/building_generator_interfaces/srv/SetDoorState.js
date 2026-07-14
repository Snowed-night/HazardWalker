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

class SetDoorStateRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.door_id = null;
      this.open = null;
    }
    else {
      if (initObj.hasOwnProperty('door_id')) {
        this.door_id = initObj.door_id
      }
      else {
        this.door_id = '';
      }
      if (initObj.hasOwnProperty('open')) {
        this.open = initObj.open
      }
      else {
        this.open = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SetDoorStateRequest
    // Serialize message field [door_id]
    bufferOffset = _serializer.string(obj.door_id, buffer, bufferOffset);
    // Serialize message field [open]
    bufferOffset = _serializer.bool(obj.open, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDoorStateRequest
    let len;
    let data = new SetDoorStateRequest(null);
    // Deserialize message field [door_id]
    data.door_id = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [open]
    data.open = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.door_id);
    return length + 5;
  }

  static datatype() {
    // Returns string type for a service object
    return 'building_generator_interfaces/SetDoorStateRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '20f6b07b5c0945c531e68f4e6d4076be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string door_id
    bool open
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDoorStateRequest(null);
    if (msg.door_id !== undefined) {
      resolved.door_id = msg.door_id;
    }
    else {
      resolved.door_id = ''
    }

    if (msg.open !== undefined) {
      resolved.open = msg.open;
    }
    else {
      resolved.open = false
    }

    return resolved;
    }
};

class SetDoorStateResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.accepted = null;
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
    // Serializes a message object of type SetDoorStateResponse
    // Serialize message field [accepted]
    bufferOffset = _serializer.bool(obj.accepted, buffer, bufferOffset);
    // Serialize message field [state]
    bufferOffset = _serializer.string(obj.state, buffer, bufferOffset);
    // Serialize message field [message]
    bufferOffset = _serializer.string(obj.message, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SetDoorStateResponse
    let len;
    let data = new SetDoorStateResponse(null);
    // Deserialize message field [accepted]
    data.accepted = _deserializer.bool(buffer, bufferOffset);
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
    return length + 9;
  }

  static datatype() {
    // Returns string type for a service object
    return 'building_generator_interfaces/SetDoorStateResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '0e5c296abb230eb14a2576256399997f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool accepted
    string state
    string message
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SetDoorStateResponse(null);
    if (msg.accepted !== undefined) {
      resolved.accepted = msg.accepted;
    }
    else {
      resolved.accepted = false
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
  Request: SetDoorStateRequest,
  Response: SetDoorStateResponse,
  md5sum() { return 'e5374c1c5454908412f943b22596890b'; },
  datatype() { return 'building_generator_interfaces/SetDoorState'; }
};
