;; Auto-generated. Do not edit!


(when (boundp 'building_generator_interfaces::SetDoorState)
  (if (not (find-package "BUILDING_GENERATOR_INTERFACES"))
    (make-package "BUILDING_GENERATOR_INTERFACES"))
  (shadow 'SetDoorState (find-package "BUILDING_GENERATOR_INTERFACES")))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATE")
  (make-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATE"))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATEREQUEST")
  (make-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATEREQUEST"))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATERESPONSE")
  (make-package "BUILDING_GENERATOR_INTERFACES::SETDOORSTATERESPONSE"))

(in-package "ROS")





(defclass building_generator_interfaces::SetDoorStateRequest
  :super ros::object
  :slots (_door_id _open ))

(defmethod building_generator_interfaces::SetDoorStateRequest
  (:init
   (&key
    ((:door_id __door_id) "")
    ((:open __open) nil)
    )
   (send-super :init)
   (setq _door_id (string __door_id))
   (setq _open __open)
   self)
  (:door_id
   (&optional __door_id)
   (if __door_id (setq _door_id __door_id)) _door_id)
  (:open
   (&optional (__open :null))
   (if (not (eq __open :null)) (setq _open __open)) _open)
  (:serialization-length
   ()
   (+
    ;; string _door_id
    4 (length _door_id)
    ;; bool _open
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _door_id
       (write-long (length _door_id) s) (princ _door_id s)
     ;; bool _open
       (if _open (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _door_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _door_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _open
     (setq _open (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass building_generator_interfaces::SetDoorStateResponse
  :super ros::object
  :slots (_accepted _state _message ))

(defmethod building_generator_interfaces::SetDoorStateResponse
  (:init
   (&key
    ((:accepted __accepted) nil)
    ((:state __state) "")
    ((:message __message) "")
    )
   (send-super :init)
   (setq _accepted __accepted)
   (setq _state (string __state))
   (setq _message (string __message))
   self)
  (:accepted
   (&optional (__accepted :null))
   (if (not (eq __accepted :null)) (setq _accepted __accepted)) _accepted)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _accepted
    1
    ;; string _state
    4 (length _state)
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _accepted
       (if _accepted (write-byte -1 s) (write-byte 0 s))
     ;; string _state
       (write-long (length _state) s) (princ _state s)
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _accepted
     (setq _accepted (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass building_generator_interfaces::SetDoorState
  :super ros::object
  :slots ())

(setf (get building_generator_interfaces::SetDoorState :md5sum-) "e5374c1c5454908412f943b22596890b")
(setf (get building_generator_interfaces::SetDoorState :datatype-) "building_generator_interfaces/SetDoorState")
(setf (get building_generator_interfaces::SetDoorState :request) building_generator_interfaces::SetDoorStateRequest)
(setf (get building_generator_interfaces::SetDoorState :response) building_generator_interfaces::SetDoorStateResponse)

(defmethod building_generator_interfaces::SetDoorStateRequest
  (:response () (instance building_generator_interfaces::SetDoorStateResponse :init)))

(setf (get building_generator_interfaces::SetDoorStateRequest :md5sum-) "e5374c1c5454908412f943b22596890b")
(setf (get building_generator_interfaces::SetDoorStateRequest :datatype-) "building_generator_interfaces/SetDoorStateRequest")
(setf (get building_generator_interfaces::SetDoorStateRequest :definition-)
      "string door_id
bool open
---
bool accepted
string state
string message

")

(setf (get building_generator_interfaces::SetDoorStateResponse :md5sum-) "e5374c1c5454908412f943b22596890b")
(setf (get building_generator_interfaces::SetDoorStateResponse :datatype-) "building_generator_interfaces/SetDoorStateResponse")
(setf (get building_generator_interfaces::SetDoorStateResponse :definition-)
      "string door_id
bool open
---
bool accepted
string state
string message

")



(provide :building_generator_interfaces/SetDoorState "e5374c1c5454908412f943b22596890b")


