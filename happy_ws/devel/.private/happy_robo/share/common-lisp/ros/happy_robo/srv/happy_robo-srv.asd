
(cl:in-package :asdf)

(defsystem "happy_robo-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "TriggerWithCommand" :depends-on ("_package_TriggerWithCommand"))
    (:file "_package_TriggerWithCommand" :depends-on ("_package"))
  ))