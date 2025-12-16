;; Auto-generated. Do not edit!


(when (boundp 'happy_robo::AlignAndGoalPoseStamped)
  (if (not (find-package "HAPPY_ROBO"))
    (make-package "HAPPY_ROBO"))
  (shadow 'AlignAndGoalPoseStamped (find-package "HAPPY_ROBO")))
(unless (find-package "HAPPY_ROBO::ALIGNANDGOALPOSESTAMPED")
  (make-package "HAPPY_ROBO::ALIGNANDGOALPOSESTAMPED"))

(in-package "ROS")
;;//! \htmlinclude AlignAndGoalPoseStamped.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass happy_robo::AlignAndGoalPoseStamped
  :super ros::object
  :slots (_header _align_pose _goal_pose ))

(defmethod happy_robo::AlignAndGoalPoseStamped
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:align_pose __align_pose) (instance geometry_msgs::PoseStamped :init))
    ((:goal_pose __goal_pose) (instance geometry_msgs::PoseStamped :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _align_pose __align_pose)
   (setq _goal_pose __goal_pose)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:align_pose
   (&rest __align_pose)
   (if (keywordp (car __align_pose))
       (send* _align_pose __align_pose)
     (progn
       (if __align_pose (setq _align_pose (car __align_pose)))
       _align_pose)))
  (:goal_pose
   (&rest __goal_pose)
   (if (keywordp (car __goal_pose))
       (send* _goal_pose __goal_pose)
     (progn
       (if __goal_pose (setq _goal_pose (car __goal_pose)))
       _goal_pose)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped _align_pose
    (send _align_pose :serialization-length)
    ;; geometry_msgs/PoseStamped _goal_pose
    (send _goal_pose :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped _align_pose
       (send _align_pose :serialize s)
     ;; geometry_msgs/PoseStamped _goal_pose
       (send _goal_pose :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped _align_pose
     (send _align_pose :deserialize buf ptr-) (incf ptr- (send _align_pose :serialization-length))
   ;; geometry_msgs/PoseStamped _goal_pose
     (send _goal_pose :deserialize buf ptr-) (incf ptr- (send _goal_pose :serialization-length))
   ;;
   self)
  )

(setf (get happy_robo::AlignAndGoalPoseStamped :md5sum-) "bed5bcb1a5fb9623874074ff155a0f6d")
(setf (get happy_robo::AlignAndGoalPoseStamped :datatype-) "happy_robo/AlignAndGoalPoseStamped")
(setf (get happy_robo::AlignAndGoalPoseStamped :definition-)
      "Header header
geometry_msgs/PoseStamped align_pose
geometry_msgs/PoseStamped goal_pose
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: geometry_msgs/PoseStamped
# A Pose with reference coordinate frame and timestamp
Header header
Pose pose

================================================================================
MSG: geometry_msgs/Pose
# A representation of pose in free space, composed of position and orientation. 
Point position
Quaternion orientation

================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

================================================================================
MSG: geometry_msgs/Quaternion
# This represents an orientation in free space in quaternion form.

float64 x
float64 y
float64 z
float64 w

")



(provide :happy_robo/AlignAndGoalPoseStamped "bed5bcb1a5fb9623874074ff155a0f6d")


