// Auto-generated. Do not edit!

// (in-package emotion.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class emotion {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.dominant_emotion = null;
      this.face_found = null;
      this.model_confidence = null;
    }
    else {
      if (initObj.hasOwnProperty('dominant_emotion')) {
        this.dominant_emotion = initObj.dominant_emotion
      }
      else {
        this.dominant_emotion = '';
      }
      if (initObj.hasOwnProperty('face_found')) {
        this.face_found = initObj.face_found
      }
      else {
        this.face_found = false;
      }
      if (initObj.hasOwnProperty('model_confidence')) {
        this.model_confidence = initObj.model_confidence
      }
      else {
        this.model_confidence = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type emotion
    // Serialize message field [dominant_emotion]
    bufferOffset = _serializer.string(obj.dominant_emotion, buffer, bufferOffset);
    // Serialize message field [face_found]
    bufferOffset = _serializer.bool(obj.face_found, buffer, bufferOffset);
    // Serialize message field [model_confidence]
    bufferOffset = _serializer.float64(obj.model_confidence, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type emotion
    let len;
    let data = new emotion(null);
    // Deserialize message field [dominant_emotion]
    data.dominant_emotion = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [face_found]
    data.face_found = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [model_confidence]
    data.model_confidence = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.dominant_emotion);
    return length + 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'emotion/emotion';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '8f1330b96c69645b621bb929fd2da385';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string dominant_emotion
    bool face_found
    float64 model_confidence
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new emotion(null);
    if (msg.dominant_emotion !== undefined) {
      resolved.dominant_emotion = msg.dominant_emotion;
    }
    else {
      resolved.dominant_emotion = ''
    }

    if (msg.face_found !== undefined) {
      resolved.face_found = msg.face_found;
    }
    else {
      resolved.face_found = false
    }

    if (msg.model_confidence !== undefined) {
      resolved.model_confidence = msg.model_confidence;
    }
    else {
      resolved.model_confidence = 0.0
    }

    return resolved;
    }
};

module.exports = emotion;
