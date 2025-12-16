; Auto-generated. Do not edit!


(cl:in-package cyglidar_d2_ros1-srv)


;//! \htmlinclude SetRunMode-request.msg.html

(cl:defclass <SetRunMode-request> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0))
)

(cl:defclass SetRunMode-request (<SetRunMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRunMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRunMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyglidar_d2_ros1-srv:<SetRunMode-request> is deprecated: use cyglidar_d2_ros1-srv:SetRunMode-request instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetRunMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyglidar_d2_ros1-srv:mode-val is deprecated.  Use cyglidar_d2_ros1-srv:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRunMode-request>) ostream)
  "Serializes a message object of type '<SetRunMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRunMode-request>) istream)
  "Deserializes a message object of type '<SetRunMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRunMode-request>)))
  "Returns string type for a service object of type '<SetRunMode-request>"
  "cyglidar_d2_ros1/SetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode-request)))
  "Returns string type for a service object of type 'SetRunMode-request"
  "cyglidar_d2_ros1/SetRunModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRunMode-request>)))
  "Returns md5sum for a message object of type '<SetRunMode-request>"
  "7fe3d509a33a499434e32bc293dc9f6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRunMode-request)))
  "Returns md5sum for a message object of type 'SetRunMode-request"
  "7fe3d509a33a499434e32bc293dc9f6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRunMode-request>)))
  "Returns full string definition for message of type '<SetRunMode-request>"
  (cl:format cl:nil "int32 mode   # 0: 2D, 1: 3D, 2: 2D&3D Dual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRunMode-request)))
  "Returns full string definition for message of type 'SetRunMode-request"
  (cl:format cl:nil "int32 mode   # 0: 2D, 1: 3D, 2: 2D&3D Dual~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRunMode-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRunMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRunMode-request
    (cl:cons ':mode (mode msg))
))
;//! \htmlinclude SetRunMode-response.msg.html

(cl:defclass <SetRunMode-response> (roslisp-msg-protocol:ros-message)
  ((success
    :reader success
    :initarg :success
    :type cl:boolean
    :initform cl:nil)
   (message
    :reader message
    :initarg :message
    :type cl:string
    :initform ""))
)

(cl:defclass SetRunMode-response (<SetRunMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetRunMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetRunMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name cyglidar_d2_ros1-srv:<SetRunMode-response> is deprecated: use cyglidar_d2_ros1-srv:SetRunMode-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <SetRunMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyglidar_d2_ros1-srv:success-val is deprecated.  Use cyglidar_d2_ros1-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <SetRunMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader cyglidar_d2_ros1-srv:message-val is deprecated.  Use cyglidar_d2_ros1-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetRunMode-response>) ostream)
  "Serializes a message object of type '<SetRunMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetRunMode-response>) istream)
  "Deserializes a message object of type '<SetRunMode-response>"
    (cl:setf (cl:slot-value msg 'success) (cl:not (cl:zerop (cl:read-byte istream))))
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetRunMode-response>)))
  "Returns string type for a service object of type '<SetRunMode-response>"
  "cyglidar_d2_ros1/SetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode-response)))
  "Returns string type for a service object of type 'SetRunMode-response"
  "cyglidar_d2_ros1/SetRunModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetRunMode-response>)))
  "Returns md5sum for a message object of type '<SetRunMode-response>"
  "7fe3d509a33a499434e32bc293dc9f6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetRunMode-response)))
  "Returns md5sum for a message object of type 'SetRunMode-response"
  "7fe3d509a33a499434e32bc293dc9f6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetRunMode-response>)))
  "Returns full string definition for message of type '<SetRunMode-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetRunMode-response)))
  "Returns full string definition for message of type 'SetRunMode-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetRunMode-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetRunMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetRunMode-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetRunMode)))
  'SetRunMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetRunMode)))
  'SetRunMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetRunMode)))
  "Returns string type for a service object of type '<SetRunMode>"
  "cyglidar_d2_ros1/SetRunMode")