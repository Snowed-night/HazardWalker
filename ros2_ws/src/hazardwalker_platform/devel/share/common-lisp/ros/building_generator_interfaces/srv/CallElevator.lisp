; Auto-generated. Do not edit!


(cl:in-package building_generator_interfaces-srv)


;//! \htmlinclude CallElevator-request.msg.html

(cl:defclass <CallElevator-request> (roslisp-msg-protocol:ros-message)
  ((elevator_id
    :reader elevator_id
    :initarg :elevator_id
    :type cl:string
    :initform "")
   (target_floor
    :reader target_floor
    :initarg :target_floor
    :type cl:integer
    :initform 0)
   (open_doors
    :reader open_doors
    :initarg :open_doors
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass CallElevator-request (<CallElevator-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallElevator-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallElevator-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name building_generator_interfaces-srv:<CallElevator-request> is deprecated: use building_generator_interfaces-srv:CallElevator-request instead.")))

(cl:ensure-generic-function 'elevator_id-val :lambda-list '(m))
(cl:defmethod elevator_id-val ((m <CallElevator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:elevator_id-val is deprecated.  Use building_generator_interfaces-srv:elevator_id instead.")
  (elevator_id m))

(cl:ensure-generic-function 'target_floor-val :lambda-list '(m))
(cl:defmethod target_floor-val ((m <CallElevator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:target_floor-val is deprecated.  Use building_generator_interfaces-srv:target_floor instead.")
  (target_floor m))

(cl:ensure-generic-function 'open_doors-val :lambda-list '(m))
(cl:defmethod open_doors-val ((m <CallElevator-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:open_doors-val is deprecated.  Use building_generator_interfaces-srv:open_doors instead.")
  (open_doors m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallElevator-request>) ostream)
  "Serializes a message object of type '<CallElevator-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'elevator_id))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'elevator_id))
  (cl:let* ((signed (cl:slot-value msg 'target_floor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'open_doors) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallElevator-request>) istream)
  "Deserializes a message object of type '<CallElevator-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'elevator_id) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'elevator_id) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'target_floor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'open_doors) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallElevator-request>)))
  "Returns string type for a service object of type '<CallElevator-request>"
  "building_generator_interfaces/CallElevatorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallElevator-request)))
  "Returns string type for a service object of type 'CallElevator-request"
  "building_generator_interfaces/CallElevatorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallElevator-request>)))
  "Returns md5sum for a message object of type '<CallElevator-request>"
  "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallElevator-request)))
  "Returns md5sum for a message object of type 'CallElevator-request"
  "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallElevator-request>)))
  "Returns full string definition for message of type '<CallElevator-request>"
  (cl:format cl:nil "string elevator_id~%int32 target_floor~%bool open_doors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallElevator-request)))
  "Returns full string definition for message of type 'CallElevator-request"
  (cl:format cl:nil "string elevator_id~%int32 target_floor~%bool open_doors~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallElevator-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'elevator_id))
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallElevator-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CallElevator-request
    (cl:cons ':elevator_id (elevator_id msg))
    (cl:cons ':target_floor (target_floor msg))
    (cl:cons ':open_doors (open_doors msg))
))
;//! \htmlinclude CallElevator-response.msg.html

(cl:defclass <CallElevator-response> (roslisp-msg-protocol:ros-message)
  ((accepted
    :reader accepted
    :initarg :accepted
    :type cl:boolean
    :initform cl:nil)
   (current_floor
    :reader current_floor
    :initarg :current_floor
    :type cl:integer
    :initform 0)
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

(cl:defclass CallElevator-response (<CallElevator-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CallElevator-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CallElevator-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name building_generator_interfaces-srv:<CallElevator-response> is deprecated: use building_generator_interfaces-srv:CallElevator-response instead.")))

(cl:ensure-generic-function 'accepted-val :lambda-list '(m))
(cl:defmethod accepted-val ((m <CallElevator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:accepted-val is deprecated.  Use building_generator_interfaces-srv:accepted instead.")
  (accepted m))

(cl:ensure-generic-function 'current_floor-val :lambda-list '(m))
(cl:defmethod current_floor-val ((m <CallElevator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:current_floor-val is deprecated.  Use building_generator_interfaces-srv:current_floor instead.")
  (current_floor m))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <CallElevator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:state-val is deprecated.  Use building_generator_interfaces-srv:state instead.")
  (state m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <CallElevator-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader building_generator_interfaces-srv:message-val is deprecated.  Use building_generator_interfaces-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CallElevator-response>) ostream)
  "Serializes a message object of type '<CallElevator-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'accepted) 1 0)) ostream)
  (cl:let* ((signed (cl:slot-value msg 'current_floor)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CallElevator-response>) istream)
  "Deserializes a message object of type '<CallElevator-response>"
    (cl:setf (cl:slot-value msg 'accepted) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'current_floor) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CallElevator-response>)))
  "Returns string type for a service object of type '<CallElevator-response>"
  "building_generator_interfaces/CallElevatorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallElevator-response)))
  "Returns string type for a service object of type 'CallElevator-response"
  "building_generator_interfaces/CallElevatorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CallElevator-response>)))
  "Returns md5sum for a message object of type '<CallElevator-response>"
  "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CallElevator-response)))
  "Returns md5sum for a message object of type 'CallElevator-response"
  "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CallElevator-response>)))
  "Returns full string definition for message of type '<CallElevator-response>"
  (cl:format cl:nil "bool accepted~%int32 current_floor~%string state~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CallElevator-response)))
  "Returns full string definition for message of type 'CallElevator-response"
  (cl:format cl:nil "bool accepted~%int32 current_floor~%string state~%string message~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CallElevator-response>))
  (cl:+ 0
     1
     4
     4 (cl:length (cl:slot-value msg 'state))
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CallElevator-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CallElevator-response
    (cl:cons ':accepted (accepted msg))
    (cl:cons ':current_floor (current_floor msg))
    (cl:cons ':state (state msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CallElevator)))
  'CallElevator-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CallElevator)))
  'CallElevator-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CallElevator)))
  "Returns string type for a service object of type '<CallElevator>"
  "building_generator_interfaces/CallElevator")