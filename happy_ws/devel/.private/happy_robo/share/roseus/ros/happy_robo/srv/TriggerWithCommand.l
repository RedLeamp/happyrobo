;; Auto-generated. Do not edit!


(when (boundp 'happy_robo::TriggerWithCommand)
  (if (not (find-package "HAPPY_ROBO"))
    (make-package "HAPPY_ROBO"))
  (shadow 'TriggerWithCommand (find-package "HAPPY_ROBO")))
(unless (find-package "HAPPY_ROBO::TRIGGERWITHCOMMAND")
  (make-package "HAPPY_ROBO::TRIGGERWITHCOMMAND"))
(unless (find-package "HAPPY_ROBO::TRIGGERWITHCOMMANDREQUEST")
  (make-package "HAPPY_ROBO::TRIGGERWITHCOMMANDREQUEST"))
(unless (find-package "HAPPY_ROBO::TRIGGERWITHCOMMANDRESPONSE")
  (make-package "HAPPY_ROBO::TRIGGERWITHCOMMANDRESPONSE"))

(in-package "ROS")





(defclass happy_robo::TriggerWithCommandRequest
  :super ros::object
  :slots (_command _degree ))

(defmethod happy_robo::TriggerWithCommandRequest
  (:init
   (&key
    ((:command __command) 0)
    ((:degree __degree) 0.0)
    )
   (send-super :init)
   (setq _command (round __command))
   (setq _degree (float __degree))
   self)
  (:command
   (&optional __command)
   (if __command (setq _command __command)) _command)
  (:degree
   (&optional __degree)
   (if __degree (setq _degree __degree)) _degree)
  (:serialization-length
   ()
   (+
    ;; int32 _command
    4
    ;; float64 _degree
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _command
       (write-long _command s)
     ;; float64 _degree
       (sys::poke _degree (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _command
     (setq _command (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; float64 _degree
     (setq _degree (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(defclass happy_robo::TriggerWithCommandResponse
  :super ros::object
  :slots (_success _message ))

(defmethod happy_robo::TriggerWithCommandResponse
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

(defclass happy_robo::TriggerWithCommand
  :super ros::object
  :slots ())

(setf (get happy_robo::TriggerWithCommand :md5sum-) "47151e7a2db9a3278116246d030bbe6b")
(setf (get happy_robo::TriggerWithCommand :datatype-) "happy_robo/TriggerWithCommand")
(setf (get happy_robo::TriggerWithCommand :request) happy_robo::TriggerWithCommandRequest)
(setf (get happy_robo::TriggerWithCommand :response) happy_robo::TriggerWithCommandResponse)

(defmethod happy_robo::TriggerWithCommandRequest
  (:response () (instance happy_robo::TriggerWithCommandResponse :init)))

(setf (get happy_robo::TriggerWithCommandRequest :md5sum-) "47151e7a2db9a3278116246d030bbe6b")
(setf (get happy_robo::TriggerWithCommandRequest :datatype-) "happy_robo/TriggerWithCommandRequest")
(setf (get happy_robo::TriggerWithCommandRequest :definition-)
      "int32 command
float64 degree  # 새로 추가
---
bool success
string message
")

(setf (get happy_robo::TriggerWithCommandResponse :md5sum-) "47151e7a2db9a3278116246d030bbe6b")
(setf (get happy_robo::TriggerWithCommandResponse :datatype-) "happy_robo/TriggerWithCommandResponse")
(setf (get happy_robo::TriggerWithCommandResponse :definition-)
      "int32 command
float64 degree  # 새로 추가
---
bool success
string message
")



(provide :happy_robo/TriggerWithCommand "47151e7a2db9a3278116246d030bbe6b")


