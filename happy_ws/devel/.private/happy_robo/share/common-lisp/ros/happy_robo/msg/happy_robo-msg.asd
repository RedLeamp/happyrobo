
(cl:in-package :asdf)

(defsystem "happy_robo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "AlignAndGoalPoseStamped" :depends-on ("_package_AlignAndGoalPoseStamped"))
    (:file "_package_AlignAndGoalPoseStamped" :depends-on ("_package"))
  ))