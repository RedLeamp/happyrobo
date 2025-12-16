; Auto-generated. Do not edit!


(cl:in-package happy_robo-srv)


;//! \htmlinclude TriggerWithCommand-request.msg.html

(cl:defclass <TriggerWithCommand-request> (roslisp-msg-protocol:ros-message)
  ((command
    :reader command
    :initarg :command
    :type cl:integer
    :initform 0)
   (degree
    :reader degree
    :initarg :degree
    :type cl:float
    :initform 0.0))
)

(cl:defclass TriggerWithCommand-request (<TriggerWithCommand-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerWithCommand-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerWithCommand-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name happy_robo-srv:<TriggerWithCommand-request> is deprecated: use happy_robo-srv:TriggerWithCommand-request instead.")))

(cl:ensure-generic-function 'command-val :lambda-list '(m))
(cl:defmethod command-val ((m <TriggerWithCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-srv:command-val is deprecated.  Use happy_robo-srv:command instead.")
  (command m))

(cl:ensure-generic-function 'degree-val :lambda-list '(m))
(cl:defmethod degree-val ((m <TriggerWithCommand-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-srv:degree-val is deprecated.  Use happy_robo-srv:degree instead.")
  (degree m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerWithCommand-request>) ostream)
  "Serializes a message object of type '<TriggerWithCommand-request>"
  (cl:let* ((signed (cl:slot-value msg 'command)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'degree))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerWithCommand-request>) istream)
  "Deserializes a message object of type '<TriggerWithCommand-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'command) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'degree) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerWithCommand-request>)))
  "Returns string type for a service object of type '<TriggerWithCommand-request>"
  "happy_robo/TriggerWithCommandRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerWithCommand-request)))
  "Returns string type for a service object of type 'TriggerWithCommand-request"
  "happy_robo/TriggerWithCommandRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerWithCommand-request>)))
  "Returns md5sum for a message object of type '<TriggerWithCommand-request>"
  "47151e7a2db9a3278116246d030bbe6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerWithCommand-request)))
  "Returns md5sum for a message object of type 'TriggerWithCommand-request"
  "47151e7a2db9a3278116246d030bbe6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerWithCommand-request>)))
  "Returns full string definition for message of type '<TriggerWithCommand-request>"
  (cl:format cl:nil "int32 command~%float64 degree  # 새로 추가~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerWithCommand-request)))
  "Returns full string definition for message of type 'TriggerWithCommand-request"
  (cl:format cl:nil "int32 command~%float64 degree  # 새로 추가~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerWithCommand-request>))
  (cl:+ 0
     4
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerWithCommand-request>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerWithCommand-request
    (cl:cons ':command (command msg))
    (cl:cons ':degree (degree msg))
))
;//! \htmlinclude TriggerWithCommand-response.msg.html

(cl:defclass <TriggerWithCommand-response> (roslisp-msg-protocol:ros-message)
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

(cl:defclass TriggerWithCommand-response (<TriggerWithCommand-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <TriggerWithCommand-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'TriggerWithCommand-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name happy_robo-srv:<TriggerWithCommand-response> is deprecated: use happy_robo-srv:TriggerWithCommand-response instead.")))

(cl:ensure-generic-function 'success-val :lambda-list '(m))
(cl:defmethod success-val ((m <TriggerWithCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-srv:success-val is deprecated.  Use happy_robo-srv:success instead.")
  (success m))

(cl:ensure-generic-function 'message-val :lambda-list '(m))
(cl:defmethod message-val ((m <TriggerWithCommand-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader happy_robo-srv:message-val is deprecated.  Use happy_robo-srv:message instead.")
  (message m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <TriggerWithCommand-response>) ostream)
  "Serializes a message object of type '<TriggerWithCommand-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'success) 1 0)) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'message))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'message))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <TriggerWithCommand-response>) istream)
  "Deserializes a message object of type '<TriggerWithCommand-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<TriggerWithCommand-response>)))
  "Returns string type for a service object of type '<TriggerWithCommand-response>"
  "happy_robo/TriggerWithCommandResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerWithCommand-response)))
  "Returns string type for a service object of type 'TriggerWithCommand-response"
  "happy_robo/TriggerWithCommandResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<TriggerWithCommand-response>)))
  "Returns md5sum for a message object of type '<TriggerWithCommand-response>"
  "47151e7a2db9a3278116246d030bbe6b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'TriggerWithCommand-response)))
  "Returns md5sum for a message object of type 'TriggerWithCommand-response"
  "47151e7a2db9a3278116246d030bbe6b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<TriggerWithCommand-response>)))
  "Returns full string definition for message of type '<TriggerWithCommand-response>"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'TriggerWithCommand-response)))
  "Returns full string definition for message of type 'TriggerWithCommand-response"
  (cl:format cl:nil "bool success~%string message~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <TriggerWithCommand-response>))
  (cl:+ 0
     1
     4 (cl:length (cl:slot-value msg 'message))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <TriggerWithCommand-response>))
  "Converts a ROS message object to a list"
  (cl:list 'TriggerWithCommand-response
    (cl:cons ':success (success msg))
    (cl:cons ':message (message msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'TriggerWithCommand)))
  'TriggerWithCommand-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'TriggerWithCommand)))
  'TriggerWithCommand-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'TriggerWithCommand)))
  "Returns string type for a service object of type '<TriggerWithCommand>"
  "happy_robo/TriggerWithCommand")