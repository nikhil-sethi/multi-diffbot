;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::SetMotorSpeed)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'SetMotorSpeed (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::SETMOTORSPEED")
  (make-package "MIRTE_MSGS::SETMOTORSPEED"))
(unless (find-package "MIRTE_MSGS::SETMOTORSPEEDREQUEST")
  (make-package "MIRTE_MSGS::SETMOTORSPEEDREQUEST"))
(unless (find-package "MIRTE_MSGS::SETMOTORSPEEDRESPONSE")
  (make-package "MIRTE_MSGS::SETMOTORSPEEDRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::SetMotorSpeedRequest
  :super ros::object
  :slots (_speed ))

(defmethod mirte_msgs::SetMotorSpeedRequest
  (:init
   (&key
    ((:speed __speed) 0)
    )
   (send-super :init)
   (setq _speed (round __speed))
   self)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; int32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _speed
       (write-long _speed s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _speed
     (setq _speed (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::SetMotorSpeedResponse
  :super ros::object
  :slots (_status ))

(defmethod mirte_msgs::SetMotorSpeedResponse
  (:init
   (&key
    ((:status __status) nil)
    )
   (send-super :init)
   (setq _status __status)
   self)
  (:status
   (&optional (__status :null))
   (if (not (eq __status :null)) (setq _status __status)) _status)
  (:serialization-length
   ()
   (+
    ;; bool _status
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _status
       (if _status (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _status
     (setq _status (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mirte_msgs::SetMotorSpeed
  :super ros::object
  :slots ())

(setf (get mirte_msgs::SetMotorSpeed :md5sum-) "ec6a2ec4df43df120c9889aac5800d51")
(setf (get mirte_msgs::SetMotorSpeed :datatype-) "mirte_msgs/SetMotorSpeed")
(setf (get mirte_msgs::SetMotorSpeed :request) mirte_msgs::SetMotorSpeedRequest)
(setf (get mirte_msgs::SetMotorSpeed :response) mirte_msgs::SetMotorSpeedResponse)

(defmethod mirte_msgs::SetMotorSpeedRequest
  (:response () (instance mirte_msgs::SetMotorSpeedResponse :init)))

(setf (get mirte_msgs::SetMotorSpeedRequest :md5sum-) "ec6a2ec4df43df120c9889aac5800d51")
(setf (get mirte_msgs::SetMotorSpeedRequest :datatype-) "mirte_msgs/SetMotorSpeedRequest")
(setf (get mirte_msgs::SetMotorSpeedRequest :definition-)
      "int32 speed
---
bool status

")

(setf (get mirte_msgs::SetMotorSpeedResponse :md5sum-) "ec6a2ec4df43df120c9889aac5800d51")
(setf (get mirte_msgs::SetMotorSpeedResponse :datatype-) "mirte_msgs/SetMotorSpeedResponse")
(setf (get mirte_msgs::SetMotorSpeedResponse :definition-)
      "int32 speed
---
bool status

")



(provide :mirte_msgs/SetMotorSpeed "ec6a2ec4df43df120c9889aac5800d51")


