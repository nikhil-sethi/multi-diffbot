;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::GetIntensityDigital)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'GetIntensityDigital (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GETINTENSITYDIGITAL")
  (make-package "MIRTE_MSGS::GETINTENSITYDIGITAL"))
(unless (find-package "MIRTE_MSGS::GETINTENSITYDIGITALREQUEST")
  (make-package "MIRTE_MSGS::GETINTENSITYDIGITALREQUEST"))
(unless (find-package "MIRTE_MSGS::GETINTENSITYDIGITALRESPONSE")
  (make-package "MIRTE_MSGS::GETINTENSITYDIGITALRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::GetIntensityDigitalRequest
  :super ros::object
  :slots ())

(defmethod mirte_msgs::GetIntensityDigitalRequest
  (:init
   (&key
    )
   (send-super :init)
   self)
  (:serialization-length
   ()
   (+
    0
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;;
   self)
  )

(defclass mirte_msgs::GetIntensityDigitalResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::GetIntensityDigitalResponse
  (:init
   (&key
    ((:data __data) nil)
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&optional (__data :null))
   (if (not (eq __data :null)) (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; bool _data
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _data
       (if _data (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _data
     (setq _data (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mirte_msgs::GetIntensityDigital
  :super ros::object
  :slots ())

(setf (get mirte_msgs::GetIntensityDigital :md5sum-) "8b94c1b53db61fb6aed406028ad6332a")
(setf (get mirte_msgs::GetIntensityDigital :datatype-) "mirte_msgs/GetIntensityDigital")
(setf (get mirte_msgs::GetIntensityDigital :request) mirte_msgs::GetIntensityDigitalRequest)
(setf (get mirte_msgs::GetIntensityDigital :response) mirte_msgs::GetIntensityDigitalResponse)

(defmethod mirte_msgs::GetIntensityDigitalRequest
  (:response () (instance mirte_msgs::GetIntensityDigitalResponse :init)))

(setf (get mirte_msgs::GetIntensityDigitalRequest :md5sum-) "8b94c1b53db61fb6aed406028ad6332a")
(setf (get mirte_msgs::GetIntensityDigitalRequest :datatype-) "mirte_msgs/GetIntensityDigitalRequest")
(setf (get mirte_msgs::GetIntensityDigitalRequest :definition-)
      "---
bool data

")

(setf (get mirte_msgs::GetIntensityDigitalResponse :md5sum-) "8b94c1b53db61fb6aed406028ad6332a")
(setf (get mirte_msgs::GetIntensityDigitalResponse :datatype-) "mirte_msgs/GetIntensityDigitalResponse")
(setf (get mirte_msgs::GetIntensityDigitalResponse :definition-)
      "---
bool data

")



(provide :mirte_msgs/GetIntensityDigital "8b94c1b53db61fb6aed406028ad6332a")


