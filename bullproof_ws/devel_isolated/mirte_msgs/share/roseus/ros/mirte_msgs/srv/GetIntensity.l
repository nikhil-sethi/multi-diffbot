;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::GetIntensity)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'GetIntensity (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GETINTENSITY")
  (make-package "MIRTE_MSGS::GETINTENSITY"))
(unless (find-package "MIRTE_MSGS::GETINTENSITYREQUEST")
  (make-package "MIRTE_MSGS::GETINTENSITYREQUEST"))
(unless (find-package "MIRTE_MSGS::GETINTENSITYRESPONSE")
  (make-package "MIRTE_MSGS::GETINTENSITYRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::GetIntensityRequest
  :super ros::object
  :slots ())

(defmethod mirte_msgs::GetIntensityRequest
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

(defclass mirte_msgs::GetIntensityResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::GetIntensityResponse
  (:init
   (&key
    ((:data __data) 0)
    )
   (send-super :init)
   (setq _data (round __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; int32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _data
       (write-long _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _data
     (setq _data (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::GetIntensity
  :super ros::object
  :slots ())

(setf (get mirte_msgs::GetIntensity :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetIntensity :datatype-) "mirte_msgs/GetIntensity")
(setf (get mirte_msgs::GetIntensity :request) mirte_msgs::GetIntensityRequest)
(setf (get mirte_msgs::GetIntensity :response) mirte_msgs::GetIntensityResponse)

(defmethod mirte_msgs::GetIntensityRequest
  (:response () (instance mirte_msgs::GetIntensityResponse :init)))

(setf (get mirte_msgs::GetIntensityRequest :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetIntensityRequest :datatype-) "mirte_msgs/GetIntensityRequest")
(setf (get mirte_msgs::GetIntensityRequest :definition-)
      "---
int32 data

")

(setf (get mirte_msgs::GetIntensityResponse :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetIntensityResponse :datatype-) "mirte_msgs/GetIntensityResponse")
(setf (get mirte_msgs::GetIntensityResponse :definition-)
      "---
int32 data

")



(provide :mirte_msgs/GetIntensity "da5909fbe378aeaf85e547e830cc1bb7")


