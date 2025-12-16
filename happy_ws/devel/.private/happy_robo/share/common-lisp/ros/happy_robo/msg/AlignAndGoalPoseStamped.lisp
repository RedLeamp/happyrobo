; Auto-generated. Do not edit!


(cl:in-package happy_robo-msg)


;//! \htmlinclude AlignAndGoalPoseStamped.msg.html

(cl:defclass <AlignAndGoalPoseStamped> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (align_pose
    :reader align_pose
    :initarg :align_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (goal_pose
    :reader goal_pose
    :initarg :goal_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped)))
)

(cl:defclass AlignAndGoalPoseStamped (<AlignAndGoalPoseStamped>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <AlignAndGoalPoseStamped>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'AlignAndGoalPoseStamped)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name happy_robo-msg:<AlignAndGoalPoseStamped> is deprecated: use happy_robo-msg:AlignAndGoalPoseStamped instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <AlignAndGoalPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-msg:header-val is deprecated.  Use happy_robo-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'align_pose-val :lambda-list '(m))
(cl:defmethod align_pose-val ((m <AlignAndGoalPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-msg:align_pose-val is deprecated.  Use happy_robo-msg:align_pose instead.")
  (align_pose m))

(cl:ensure-generic-function 'goal_pose-val :lambda-list '(m))
(cl:defmethod goal_pose-val ((m <AlignAndGoalPoseStamped>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-msg:goal_pose-val is deprecated.  Use happy_robo-msg:goal_pose instead.")
  (goal_pose m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <AlignAndGoalPoseStamped>) ostream)
  "Serializes a message object of type '<AlignAndGoalPoseStamped>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'align_pose) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_pose) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <AlignAndGoalPoseStamped>) istream)
  "Deserializes a message object of type '<AlignAndGoalPoseStamped>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'align_pose) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_pose) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<AlignAndGoalPoseStamped>)))
  "Returns string type for a message object of type '<AlignAndGoalPoseStamped>"
  "happy_robo/AlignAndGoalPoseStamped")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'AlignAndGoalPoseStamped)))
  "Returns string type for a message object of type 'AlignAndGoalPoseStamped"
  "happy_robo/AlignAndGoalPoseStamped")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<AlignAndGoalPoseStamped>)))
  "Returns md5sum for a message object of type '<AlignAndGoalPoseStamped>"
  "bed5bcb1a5fb9623874074ff155a0f6d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'AlignAndGoalPoseStamped)))
  "Returns md5sum for a message object of type 'AlignAndGoalPoseStamped"
  "bed5bcb1a5fb9623874074ff155a0f6d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<AlignAndGoalPoseStamped>)))
  "Returns full string definition for message of type '<AlignAndGoalPoseStamped>"
  (cl:format cl:nil "Header header~%geometry_msgs/PoseStamped align_pose~%geometry_msgs/PoseStamped goal_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'AlignAndGoalPoseStamped)))
  "Returns full string definition for message of type 'AlignAndGoalPoseStamped"
  (cl:format cl:nil "Header header~%geometry_msgs/PoseStamped align_pose~%geometry_msgs/PoseStamped goal_pose~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <AlignAndGoalPoseStamped>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'align_pose))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_pose))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <AlignAndGoalPoseStamped>))
  "Converts a ROS message object to a list"
  (cl:list 'AlignAndGoalPoseStamped
    (cl:cons ':header (header msg))
    (cl:cons ':align_pose (align_pose msg))
    (cl:cons ':goal_pose (goal_pose msg))
))
