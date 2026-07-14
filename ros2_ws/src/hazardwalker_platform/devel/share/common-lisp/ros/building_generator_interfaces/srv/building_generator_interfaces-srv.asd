
(cl:in-package :asdf)

(defsystem "building_generator_interfaces-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "CallElevator" :depends-on ("_package_CallElevator"))
    (:file "_package_CallElevator" :depends-on ("_package"))
    (:file "SetDoorState" :depends-on ("_package_SetDoorState"))
    (:file "_package_SetDoorState" :depends-on ("_package"))
  ))