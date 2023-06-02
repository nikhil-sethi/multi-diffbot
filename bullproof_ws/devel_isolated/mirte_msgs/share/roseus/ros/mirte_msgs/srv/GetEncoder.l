;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::GetEncoder)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'GetEncoder (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GETENCODER")
  (make-package "MIRTE_MSGS::GETENCODER"))
(unless (find-package "MIRTE_MSGS::GETENCODERREQUEST")
  (make-package "MIRTE_MSGS::GETENCODERREQUEST"))
(unless (find-package "MIRTE_MSGS::GETENCODERRESPONSE")
  (make-package "MIRTE_MSGS::GETENCODERRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::GetEncoderRequest
  :super ros::object
  :slots ())

(defmethod mirte_msgs::GetEncoderRequest
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

(defclass mirte_msgs::GetEncoderResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::GetEncoderResponse
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

(defclass mirte_msgs::GetEncoder
  :super ros::object
  :slots ())

(setf (get mirte_msgs::GetEncoder :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetEncoder :datatype-) "mirte_msgs/GetEncoder")
(setf (get mirte_msgs::GetEncoder :request) mirte_msgs::GetEncoderRequest)
(setf (get mirte_msgs::GetEncoder :response) mirte_msgs::GetEncoderResponse)

(defmethod mirte_msgs::GetEncoderRequest
  (:response () (instance mirte_msgs::GetEncoderResponse :init)))

(setf (get mirte_msgs::GetEncoderRequest :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetEncoderRequest :datatype-) "mirte_msgs/GetEncoderRequest")
(setf (get mirte_msgs::GetEncoderRequest :definition-)
      "---
int32 data

")

(setf (get mirte_msgs::GetEncoderResponse :md5sum-) "da5909fbe378aeaf85e547e830cc1bb7")
(setf (get mirte_msgs::GetEncoderResponse :datatype-) "mirte_msgs/GetEncoderResponse")
(setf (get mirte_msgs::GetEncoderResponse :definition-)
      "---
int32 data

")



(provide :mirte_msgs/GetEncoder "da5909fbe378aeaf85e547e830cc1bb7")


