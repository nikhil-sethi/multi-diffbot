;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::GetDistance)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'GetDistance (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GETDISTANCE")
  (make-package "MIRTE_MSGS::GETDISTANCE"))
(unless (find-package "MIRTE_MSGS::GETDISTANCEREQUEST")
  (make-package "MIRTE_MSGS::GETDISTANCEREQUEST"))
(unless (find-package "MIRTE_MSGS::GETDISTANCERESPONSE")
  (make-package "MIRTE_MSGS::GETDISTANCERESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::GetDistanceRequest
  :super ros::object
  :slots ())

(defmethod mirte_msgs::GetDistanceRequest
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

(defclass mirte_msgs::GetDistanceResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::GetDistanceResponse
  (:init
   (&key
    ((:data __data) 0.0)
    )
   (send-super :init)
   (setq _data (float __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; float32 _data
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _data
       (sys::poke _data (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _data
     (setq _data (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::GetDistance
  :super ros::object
  :slots ())

(setf (get mirte_msgs::GetDistance :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get mirte_msgs::GetDistance :datatype-) "mirte_msgs/GetDistance")
(setf (get mirte_msgs::GetDistance :request) mirte_msgs::GetDistanceRequest)
(setf (get mirte_msgs::GetDistance :response) mirte_msgs::GetDistanceResponse)

(defmethod mirte_msgs::GetDistanceRequest
  (:response () (instance mirte_msgs::GetDistanceResponse :init)))

(setf (get mirte_msgs::GetDistanceRequest :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get mirte_msgs::GetDistanceRequest :datatype-) "mirte_msgs/GetDistanceRequest")
(setf (get mirte_msgs::GetDistanceRequest :definition-)
      "---
float32 data

")

(setf (get mirte_msgs::GetDistanceResponse :md5sum-) "73fcbf46b49191e672908e50842a83d4")
(setf (get mirte_msgs::GetDistanceResponse :datatype-) "mirte_msgs/GetDistanceResponse")
(setf (get mirte_msgs::GetDistanceResponse :definition-)
      "---
float32 data

")



(provide :mirte_msgs/GetDistance "73fcbf46b49191e672908e50842a83d4")


