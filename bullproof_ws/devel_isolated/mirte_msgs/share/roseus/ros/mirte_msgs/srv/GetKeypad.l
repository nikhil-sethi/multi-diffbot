;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::GetKeypad)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'GetKeypad (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GETKEYPAD")
  (make-package "MIRTE_MSGS::GETKEYPAD"))
(unless (find-package "MIRTE_MSGS::GETKEYPADREQUEST")
  (make-package "MIRTE_MSGS::GETKEYPADREQUEST"))
(unless (find-package "MIRTE_MSGS::GETKEYPADRESPONSE")
  (make-package "MIRTE_MSGS::GETKEYPADRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::GetKeypadRequest
  :super ros::object
  :slots ())

(defmethod mirte_msgs::GetKeypadRequest
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

(defclass mirte_msgs::GetKeypadResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::GetKeypadResponse
  (:init
   (&key
    ((:data __data) "")
    )
   (send-super :init)
   (setq _data (string __data))
   self)
  (:data
   (&optional __data)
   (if __data (setq _data __data)) _data)
  (:serialization-length
   ()
   (+
    ;; string _data
    4 (length _data)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _data
       (write-long (length _data) s) (princ _data s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _data
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _data (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mirte_msgs::GetKeypad
  :super ros::object
  :slots ())

(setf (get mirte_msgs::GetKeypad :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get mirte_msgs::GetKeypad :datatype-) "mirte_msgs/GetKeypad")
(setf (get mirte_msgs::GetKeypad :request) mirte_msgs::GetKeypadRequest)
(setf (get mirte_msgs::GetKeypad :response) mirte_msgs::GetKeypadResponse)

(defmethod mirte_msgs::GetKeypadRequest
  (:response () (instance mirte_msgs::GetKeypadResponse :init)))

(setf (get mirte_msgs::GetKeypadRequest :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get mirte_msgs::GetKeypadRequest :datatype-) "mirte_msgs/GetKeypadRequest")
(setf (get mirte_msgs::GetKeypadRequest :definition-)
      "---
string data

")

(setf (get mirte_msgs::GetKeypadResponse :md5sum-) "992ce8a1687cec8c8bd883ec73ca41d1")
(setf (get mirte_msgs::GetKeypadResponse :datatype-) "mirte_msgs/GetKeypadResponse")
(setf (get mirte_msgs::GetKeypadResponse :definition-)
      "---
string data

")



(provide :mirte_msgs/GetKeypad "992ce8a1687cec8c8bd883ec73ca41d1")


