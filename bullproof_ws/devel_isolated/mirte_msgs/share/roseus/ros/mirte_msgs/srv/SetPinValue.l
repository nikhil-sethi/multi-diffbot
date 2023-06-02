;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::SetPinValue)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'SetPinValue (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::SETPINVALUE")
  (make-package "MIRTE_MSGS::SETPINVALUE"))
(unless (find-package "MIRTE_MSGS::SETPINVALUEREQUEST")
  (make-package "MIRTE_MSGS::SETPINVALUEREQUEST"))
(unless (find-package "MIRTE_MSGS::SETPINVALUERESPONSE")
  (make-package "MIRTE_MSGS::SETPINVALUERESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::SetPinValueRequest
  :super ros::object
  :slots (_pin _type _value ))

(defmethod mirte_msgs::SetPinValueRequest
  (:init
   (&key
    ((:pin __pin) "")
    ((:type __type) "")
    ((:value __value) 0)
    )
   (send-super :init)
   (setq _pin (string __pin))
   (setq _type (string __type))
   (setq _value (round __value))
   self)
  (:pin
   (&optional __pin)
   (if __pin (setq _pin __pin)) _pin)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _pin
    4 (length _pin)
    ;; string _type
    4 (length _type)
    ;; int32 _value
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _pin
       (write-long (length _pin) s) (princ _pin s)
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; int32 _value
       (write-long _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _pin
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _pin (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _value
     (setq _value (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::SetPinValueResponse
  :super ros::object
  :slots (_status ))

(defmethod mirte_msgs::SetPinValueResponse
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

(defclass mirte_msgs::SetPinValue
  :super ros::object
  :slots ())

(setf (get mirte_msgs::SetPinValue :md5sum-) "ca22cfaaa89cb1d56388f4014610ebb5")
(setf (get mirte_msgs::SetPinValue :datatype-) "mirte_msgs/SetPinValue")
(setf (get mirte_msgs::SetPinValue :request) mirte_msgs::SetPinValueRequest)
(setf (get mirte_msgs::SetPinValue :response) mirte_msgs::SetPinValueResponse)

(defmethod mirte_msgs::SetPinValueRequest
  (:response () (instance mirte_msgs::SetPinValueResponse :init)))

(setf (get mirte_msgs::SetPinValueRequest :md5sum-) "ca22cfaaa89cb1d56388f4014610ebb5")
(setf (get mirte_msgs::SetPinValueRequest :datatype-) "mirte_msgs/SetPinValueRequest")
(setf (get mirte_msgs::SetPinValueRequest :definition-)
      "string pin
string type
int32 value
---
bool status


")

(setf (get mirte_msgs::SetPinValueResponse :md5sum-) "ca22cfaaa89cb1d56388f4014610ebb5")
(setf (get mirte_msgs::SetPinValueResponse :datatype-) "mirte_msgs/SetPinValueResponse")
(setf (get mirte_msgs::SetPinValueResponse :definition-)
      "string pin
string type
int32 value
---
bool status


")



(provide :mirte_msgs/SetPinValue "ca22cfaaa89cb1d56388f4014610ebb5")


