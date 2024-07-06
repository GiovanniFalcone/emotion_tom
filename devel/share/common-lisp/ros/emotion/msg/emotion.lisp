; Auto-generated. Do not edit!


(cl:in-package emotion-msg)


;//! \htmlinclude emotion.msg.html

(cl:defclass <emotion> (roslisp-msg-protocol:ros-message)
  ((dominant_emotion
    :reader dominant_emotion
    :initarg :dominant_emotion
    :type cl:string
    :initform "")
   (face_found
    :reader face_found
    :initarg :face_found
    :type cl:boolean
    :initform cl:nil)
   (model_confidence
    :reader model_confidence
    :initarg :model_confidence
    :type cl:float
    :initform 0.0))
)

(cl:defclass emotion (<emotion>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <emotion>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'emotion)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name emotion-msg:<emotion> is deprecated: use emotion-msg:emotion instead.")))

(cl:ensure-generic-function 'dominant_emotion-val :lambda-list '(m))
(cl:defmethod dominant_emotion-val ((m <emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader emotion-msg:dominant_emotion-val is deprecated.  Use emotion-msg:dominant_emotion instead.")
  (dominant_emotion m))

(cl:ensure-generic-function 'face_found-val :lambda-list '(m))
(cl:defmethod face_found-val ((m <emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader emotion-msg:face_found-val is deprecated.  Use emotion-msg:face_found instead.")
  (face_found m))

(cl:ensure-generic-function 'model_confidence-val :lambda-list '(m))
(cl:defmethod model_confidence-val ((m <emotion>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader emotion-msg:model_confidence-val is deprecated.  Use emotion-msg:model_confidence instead.")
  (model_confidence m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <emotion>) ostream)
  "Serializes a message object of type '<emotion>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'dominant_emotion))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'dominant_emotion))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'face_found) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'model_confidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <emotion>) istream)
  "Deserializes a message object of type '<emotion>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'dominant_emotion) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'dominant_emotion) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'face_found) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'model_confidence) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<emotion>)))
  "Returns string type for a message object of type '<emotion>"
  "emotion/emotion")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'emotion)))
  "Returns string type for a message object of type 'emotion"
  "emotion/emotion")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<emotion>)))
  "Returns md5sum for a message object of type '<emotion>"
  "8f1330b96c69645b621bb929fd2da385")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'emotion)))
  "Returns md5sum for a message object of type 'emotion"
  "8f1330b96c69645b621bb929fd2da385")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<emotion>)))
  "Returns full string definition for message of type '<emotion>"
  (cl:format cl:nil "string dominant_emotion~%bool face_found~%float64 model_confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'emotion)))
  "Returns full string definition for message of type 'emotion"
  (cl:format cl:nil "string dominant_emotion~%bool face_found~%float64 model_confidence~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <emotion>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'dominant_emotion))
     1
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <emotion>))
  "Converts a ROS message object to a list"
  (cl:list 'emotion
    (cl:cons ':dominant_emotion (dominant_emotion msg))
    (cl:cons ':face_found (face_found msg))
    (cl:cons ':model_confidence (model_confidence msg))
))
