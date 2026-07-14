; Auto-generated. Do not edit!


(cl:in-package building_generator_interfaces-srv)


;//! \htmlinclude SetDoorState-request.msg.html

(cl:defclass <SetDoorState-request> (roslisp-msg-protocol:ros-message)
  ((door_id
    :reader door_id
    :initarg :door_id
    :type cl:string
    :initform "")
   (open
    :reader open
    :initarg :open
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetDoorState-request (<SetDoorState-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDoorState-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDoorState-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name building_generator_interfaces-srv:<SetDoorState-request> is deprecated: use building_generator_interfaces-srv:SetDoorState-request instead.")))

(cl:ensure-generic-function 'door_id-val :lambda-list '(m))
(cl:defmethod door_id-val ((m <SetDoorState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:door_id-val is deprecated.  Use building_generator_interfaces-srv:door_id instead.")
  (door_id m))

(cl:ensure-generic-function 'open-val :lambda-list '(m))
(cl:defmethod open-val ((m <SetDoorState-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:open-val is deprecated.  Use building_generator_interfaces-srv:open instead.")
  (open m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDoorState-request>) ostream)
  "Serializes a message object of type '<SetDoorState-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'door_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'door_id))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'open) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDoorState-request>) istream)
  "Deserializes a message object of type '<SetDoorState-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'door_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'door_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:setf (cl:slot-value msg 'open) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDoorState-request>)))
  "Returns string type for a service object of type '<SetDoorState-request>"
  "building_generator_interfaces/SetDoorStateRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDoorState-request)))
  "Returns string type for a service object of type 'SetDoorState-request"
  "building_generator_interfaces/SetDoorStateRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDoorState-request>)))
  "Returns md5sum for a message object of type '<SetDoorState-request>"
  "e5374c1c5454908412f943b22596890b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDoorState-request)))
  "Returns md5sum for a message object of type 'SetDoorState-request"
  "e5374c1c5454908412f943b22596890b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDoorState-request>)))
  "Returns full string definition for message of type '<SetDoorState-request>"
  (cl:format cl:nil "string door_id~%bool open~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDoorState-request)))
  "Returns full string definition for message of type 'SetDoorState-request"
  (cl:format cl:nil "string door_id~%bool open~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDoorState-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'door_id))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDoorState-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDoorState-request
    (cl:cons ':door_id (door_id msg))
    (cl:cons ':open (open msg))
))
;//! \htmlinclude SetDoorState-response.msg.html

(cl:defclass <SetDoorState-response> (roslisp-msg-protocol:ros-message)
  ((accepted
    :reader accepted
    :initarg :accepted
    :type cl:boolean
    :initform cl:nil)
   (state
    :reader state
    :initarg :state
    :type cl:string
    :initform "")
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetDoorState-response (<SetDoorState-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetDoorState-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetDoorState-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name building_generator_interfaces-srv:<SetDoorState-response> is deprecated: use building_generator_interfaces-srv:SetDoorState-response instead.")))

(cl:ensure-generic-function 'accepted-val :lambda-list '(m))
(cl:defmethod accepted-val ((m <SetDoorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:accepted-val is deprecated.  Use building_generator_interfaces-srv:accepted instead.")
  (accepted m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <SetDoorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:state-val is deprecated.  Use building_generator_interfaces-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetDoorState-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:message-val is deprecated.  Use building_generator_interfaces-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetDoorState-response>) ostream)
  "Serializes a message object of type '<SetDoorState-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'accepted) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'state))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'state))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetDoorState-response>) istream)
  "Deserializes a message object of type '<SetDoorState-response>"
    (cl:setf (cl:slot-value msg 'accepted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'state) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'message) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'message) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetDoorState-response>)))
  "Returns string type for a service object of type '<SetDoorState-response>"
  "building_generator_interfaces/SetDoorStateResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDoorState-response)))
  "Returns string type for a service object of type 'SetDoorState-response"
  "building_generator_interfaces/SetDoorStateResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetDoorState-response>)))
  "Returns md5sum for a message object of type '<SetDoorState-response>"
  "e5374c1c5454908412f943b22596890b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetDoorState-response)))
  "Returns md5sum for a message object of type 'SetDoorState-response"
  "e5374c1c5454908412f943b22596890b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetDoorState-response>)))
  "Returns full string definition for message of type '<SetDoorState-response>"
  (cl:format cl:nil "bool accepted~%string state~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetDoorState-response)))
  "Returns full string definition for message of type 'SetDoorState-response"
  (cl:format cl:nil "bool accepted~%string state~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetDoorState-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'state))
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetDoorState-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetDoorState-response
    (cl:cons ':accepted (accepted msg))
    (cl:cons ':state (state msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetDoorState)))
  'SetDoorState-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetDoorState)))
  'SetDoorState-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetDoorState)))
  "Returns string type for a service object of type '<SetDoorState>"
  "building_generator_interfaces/SetDoorState")