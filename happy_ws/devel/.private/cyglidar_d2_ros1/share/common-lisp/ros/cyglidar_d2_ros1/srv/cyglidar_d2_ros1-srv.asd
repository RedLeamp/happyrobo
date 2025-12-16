
(cl:in-package :asdf)

(defsystem "cyglidar_d2_ros1-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetRunMode" :depends-on ("_package_SetRunMode"))
    (:file "_package_SetRunMode" :depends-on ("_package"))
  ))