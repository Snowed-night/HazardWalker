;; Auto-generated. Do not edit!


(when (boundp 'building_generator_interfaces::CallElevator)
  (if (not (find-package "BUILDING_GENERATOR_INTERFACES"))
    (make-package "BUILDING_GENERATOR_INTERFACES"))
  (shadow 'CallElevator (find-package "BUILDING_GENERATOR_INTERFACES")))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATOR")
  (make-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATOR"))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATORREQUEST")
  (make-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATORREQUEST"))
(unless (find-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATORRESPONSE")
  (make-package "BUILDING_GENERATOR_INTERFACES::CALLELEVATORRESPONSE"))

(in-package "ROS")





(defclass building_generator_interfaces::CallElevatorRequest
  :super ros::object
  :slots (_elevator_id _target_floor _open_doors ))

(defmethod building_generator_interfaces::CallElevatorRequest
  (:init
   (&key
    ((:elevator_id __elevator_id) "")
    ((:target_floor __target_floor) 0)
    ((:open_doors __open_doors) nil)
    )
   (send-super :init)
   (setq _elevator_id (string __elevator_id))
   (setq _target_floor (round __target_floor))
   (setq _open_doors __open_doors)
   self)
  (:elevator_id
   (&optional __elevator_id)
   (if __elevator_id (setq _elevator_id __elevator_id)) _elevator_id)
  (:target_floor
   (&optional __target_floor)
   (if __target_floor (setq _target_floor __target_floor)) _target_floor)
  (:open_doors
   (&optional (__open_doors :null))
   (if (not (eq __open_doors :null)) (setq _open_doors __open_doors)) _open_doors)
  (:serialization-length
   ()
   (+
    ;; string _elevator_id
    4 (length _elevator_id)
    ;; int32 _target_floor
    4
    ;; bool _open_doors
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _elevator_id
       (write-long (length _elevator_id) s) (princ _elevator_id s)
     ;; int32 _target_floor
       (write-long _target_floor s)
     ;; bool _open_doors
       (if _open_doors (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _elevator_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _elevator_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _target_floor
     (setq _target_floor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _open_doors
     (setq _open_doors (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass building_generator_interfaces::CallElevatorResponse
  :super ros::object
  :slots (_accepted _current_floor _state _message ))

(defmethod building_generator_interfaces::CallElevatorResponse
  (:init
   (&key
    ((:accepted __accepted) nil)
    ((:current_floor __current_floor) 0)
    ((:state __state) "")
    ((:message __message) "")
    )
   (send-super :init)
   (setq _accepted __accepted)
   (setq _current_floor (round __current_floor))
   (setq _state (string __state))
   (setq _message (string __message))
   self)
  (:accepted
   (&optional (__accepted :null))
   (if (not (eq __accepted :null)) (setq _accepted __accepted)) _accepted)
  (:current_floor
   (&optional __current_floor)
   (if __current_floor (setq _current_floor __current_floor)) _current_floor)
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
    ;; int32 _current_floor
    4
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
     ;; int32 _current_floor
       (write-long _current_floor s)
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
   ;; int32 _current_floor
     (setq _current_floor (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; string _state
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _state (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass building_generator_interfaces::CallElevator
  :super ros::object
  :slots ())

(setf (get building_generator_interfaces::CallElevator :md5sum-) "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(setf (get building_generator_interfaces::CallElevator :datatype-) "building_generator_interfaces/CallElevator")
(setf (get building_generator_interfaces::CallElevator :request) building_generator_interfaces::CallElevatorRequest)
(setf (get building_generator_interfaces::CallElevator :response) building_generator_interfaces::CallElevatorResponse)

(defmethod building_generator_interfaces::CallElevatorRequest
  (:response () (instance building_generator_interfaces::CallElevatorResponse :init)))

(setf (get building_generator_interfaces::CallElevatorRequest :md5sum-) "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(setf (get building_generator_interfaces::CallElevatorRequest :datatype-) "building_generator_interfaces/CallElevatorRequest")
(setf (get building_generator_interfaces::CallElevatorRequest :definition-)
      "string elevator_id
int32 target_floor
bool open_doors
---
bool accepted
int32 current_floor
string state
string message

")

(setf (get building_generator_interfaces::CallElevatorResponse :md5sum-) "b2f9babc7642b34ecdc8d6d6e7e3cebf")
(setf (get building_generator_interfaces::CallElevatorResponse :datatype-) "building_generator_interfaces/CallElevatorResponse")
(setf (get building_generator_interfaces::CallElevatorResponse :definition-)
      "string elevator_id
int32 target_floor
bool open_doors
---
bool accepted
int32 current_floor
string state
string message

")



(provide :building_generator_interfaces/CallElevator "b2f9babc7642b34ecdc8d6d6e7e3cebf")


