;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::SetOLEDImage)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'SetOLEDImage (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::SETOLEDIMAGE")
  (make-package "MIRTE_MSGS::SETOLEDIMAGE"))
(unless (find-package "MIRTE_MSGS::SETOLEDIMAGEREQUEST")
  (make-package "MIRTE_MSGS::SETOLEDIMAGEREQUEST"))
(unless (find-package "MIRTE_MSGS::SETOLEDIMAGERESPONSE")
  (make-package "MIRTE_MSGS::SETOLEDIMAGERESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::SetOLEDImageRequest
  :super ros::object
  :slots (_type _value ))

(defmethod mirte_msgs::SetOLEDImageRequest
  (:init
   (&key
    ((:type __type) "")
    ((:value __value) "")
    )
   (send-super :init)
   (setq _type (string __type))
   (setq _value (string __value))
   self)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:serialization-length
   ()
   (+
    ;; string _type
    4 (length _type)
    ;; string _value
    4 (length _value)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _type
       (write-long (length _type) s) (princ _type s)
     ;; string _value
       (write-long (length _value) s) (princ _value s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _value
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _value (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mirte_msgs::SetOLEDImageResponse
  :super ros::object
  :slots (_status ))

(defmethod mirte_msgs::SetOLEDImageResponse
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

(defclass mirte_msgs::SetOLEDImage
  :super ros::object
  :slots ())

(setf (get mirte_msgs::SetOLEDImage :md5sum-) "52215026eb827c220b801bf2b2c8f4d1")
(setf (get mirte_msgs::SetOLEDImage :datatype-) "mirte_msgs/SetOLEDImage")
(setf (get mirte_msgs::SetOLEDImage :request) mirte_msgs::SetOLEDImageRequest)
(setf (get mirte_msgs::SetOLEDImage :response) mirte_msgs::SetOLEDImageResponse)

(defmethod mirte_msgs::SetOLEDImageRequest
  (:response () (instance mirte_msgs::SetOLEDImageResponse :init)))

(setf (get mirte_msgs::SetOLEDImageRequest :md5sum-) "52215026eb827c220b801bf2b2c8f4d1")
(setf (get mirte_msgs::SetOLEDImageRequest :datatype-) "mirte_msgs/SetOLEDImageRequest")
(setf (get mirte_msgs::SetOLEDImageRequest :definition-)
      "string type
string value
---
bool status


")

(setf (get mirte_msgs::SetOLEDImageResponse :md5sum-) "52215026eb827c220b801bf2b2c8f4d1")
(setf (get mirte_msgs::SetOLEDImageResponse :datatype-) "mirte_msgs/SetOLEDImageResponse")
(setf (get mirte_msgs::SetOLEDImageResponse :definition-)
      "string type
string value
---
bool status


")



(provide :mirte_msgs/SetOLEDImage "52215026eb827c220b801bf2b2c8f4d1")


