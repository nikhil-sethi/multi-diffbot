;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::SetLEDValue)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'SetLEDValue (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::SETLEDVALUE")
  (make-package "MIRTE_MSGS::SETLEDVALUE"))
(unless (find-package "MIRTE_MSGS::SETLEDVALUEREQUEST")
  (make-package "MIRTE_MSGS::SETLEDVALUEREQUEST"))
(unless (find-package "MIRTE_MSGS::SETLEDVALUERESPONSE")
  (make-package "MIRTE_MSGS::SETLEDVALUERESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::SetLEDValueRequest
  :super ros::object
  :slots (_value ))

(defmethod mirte_msgs::SetLEDValueRequest
  (:init
   (&key
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _value (round __value))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::SetLEDValueResponse
  :super ros::object
  :slots (_status ))

(defmethod mirte_msgs::SetLEDValueResponse
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

(defclass mirte_msgs::SetLEDValue
  :super ros::object
  :slots ())

(setf (get mirte_msgs::SetLEDValue :md5sum-) "42048ea920ba4286ecdfb4bc68063c32")
(setf (get mirte_msgs::SetLEDValue :datatype-) "mirte_msgs/SetLEDValue")
(setf (get mirte_msgs::SetLEDValue :request) mirte_msgs::SetLEDValueRequest)
(setf (get mirte_msgs::SetLEDValue :response) mirte_msgs::SetLEDValueResponse)

(defmethod mirte_msgs::SetLEDValueRequest
  (:response () (instance mirte_msgs::SetLEDValueResponse :init)))

(setf (get mirte_msgs::SetLEDValueRequest :md5sum-) "42048ea920ba4286ecdfb4bc68063c32")
(setf (get mirte_msgs::SetLEDValueRequest :datatype-) "mirte_msgs/SetLEDValueRequest")
(setf (get mirte_msgs::SetLEDValueRequest :definition-)
      "int32 value
---
bool status


")

(setf (get mirte_msgs::SetLEDValueResponse :md5sum-) "42048ea920ba4286ecdfb4bc68063c32")
(setf (get mirte_msgs::SetLEDValueResponse :datatype-) "mirte_msgs/SetLEDValueResponse")
(setf (get mirte_msgs::SetLEDValueResponse :definition-)
      "int32 value
---
bool status


")



(provide :mirte_msgs/SetLEDValue "42048ea920ba4286ecdfb4bc68063c32")


