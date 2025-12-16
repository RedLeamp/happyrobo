;; Auto-generated. Do not edit!


(when (boundp 'cyglidar_d2_ros1::SetRunMode)
  (if (not (find-package "CYGLIDAR_D2_ROS1"))
    (make-package "CYGLIDAR_D2_ROS1"))
  (shadow 'SetRunMode (find-package "CYGLIDAR_D2_ROS1")))
(unless (find-package "CYGLIDAR_D2_ROS1::SETRUNMODE")
  (make-package "CYGLIDAR_D2_ROS1::SETRUNMODE"))
(unless (find-package "CYGLIDAR_D2_ROS1::SETRUNMODEREQUEST")
  (make-package "CYGLIDAR_D2_ROS1::SETRUNMODEREQUEST"))
(unless (find-package "CYGLIDAR_D2_ROS1::SETRUNMODERESPONSE")
  (make-package "CYGLIDAR_D2_ROS1::SETRUNMODERESPONSE"))

(in-package "ROS")





(defclass cyglidar_d2_ros1::SetRunModeRequest
  :super ros::object
  :slots (_mode ))

(defmethod cyglidar_d2_ros1::SetRunModeRequest
  (:init
   (&key
    ((:mode __mode) 0)
    )
   (send-super :init)
   (setq _mode (round __mode))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:serialization-length
   ()
   (+
    ;; int32 _mode
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _mode
       (write-long _mode s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _mode
     (setq _mode (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass cyglidar_d2_ros1::SetRunModeResponse
  :super ros::object
  :slots (_success _message ))

(defmethod cyglidar_d2_ros1::SetRunModeResponse
  (:init
   (&key
    ((:success __success) nil)
    ((:message __message) "")
    )
   (send-super :init)
   (setq _success __success)
   (setq _message (string __message))
   self)
  (:success
   (&optional (__success :null))
   (if (not (eq __success :null)) (setq _success __success)) _success)
  (:message
   (&optional __message)
   (if __message (setq _message __message)) _message)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ;; string _message
    4 (length _message)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;; string _message
       (write-long (length _message) s) (princ _message s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass cyglidar_d2_ros1::SetRunMode
  :super ros::object
  :slots ())

(setf (get cyglidar_d2_ros1::SetRunMode :md5sum-) "7fe3d509a33a499434e32bc293dc9f6b")
(setf (get cyglidar_d2_ros1::SetRunMode :datatype-) "cyglidar_d2_ros1/SetRunMode")
(setf (get cyglidar_d2_ros1::SetRunMode :request) cyglidar_d2_ros1::SetRunModeRequest)
(setf (get cyglidar_d2_ros1::SetRunMode :response) cyglidar_d2_ros1::SetRunModeResponse)

(defmethod cyglidar_d2_ros1::SetRunModeRequest
  (:response () (instance cyglidar_d2_ros1::SetRunModeResponse :init)))

(setf (get cyglidar_d2_ros1::SetRunModeRequest :md5sum-) "7fe3d509a33a499434e32bc293dc9f6b")
(setf (get cyglidar_d2_ros1::SetRunModeRequest :datatype-) "cyglidar_d2_ros1/SetRunModeRequest")
(setf (get cyglidar_d2_ros1::SetRunModeRequest :definition-)
      "int32 mode   # 0: 2D, 1: 3D, 2: 2D&3D Dual
---
bool success
string message
")

(setf (get cyglidar_d2_ros1::SetRunModeResponse :md5sum-) "7fe3d509a33a499434e32bc293dc9f6b")
(setf (get cyglidar_d2_ros1::SetRunModeResponse :datatype-) "cyglidar_d2_ros1/SetRunModeResponse")
(setf (get cyglidar_d2_ros1::SetRunModeResponse :definition-)
      "int32 mode   # 0: 2D, 1: 3D, 2: 2D&3D Dual
---
bool success
string message
")



(provide :cyglidar_d2_ros1/SetRunMode "7fe3d509a33a499434e32bc293dc9f6b")


