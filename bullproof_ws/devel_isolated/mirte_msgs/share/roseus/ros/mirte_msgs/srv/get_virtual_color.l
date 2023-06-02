;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::get_virtual_color)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'get_virtual_color (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::GET_VIRTUAL_COLOR")
  (make-package "MIRTE_MSGS::GET_VIRTUAL_COLOR"))
(unless (find-package "MIRTE_MSGS::GET_VIRTUAL_COLORREQUEST")
  (make-package "MIRTE_MSGS::GET_VIRTUAL_COLORREQUEST"))
(unless (find-package "MIRTE_MSGS::GET_VIRTUAL_COLORRESPONSE")
  (make-package "MIRTE_MSGS::GET_VIRTUAL_COLORRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::get_virtual_colorRequest
  :super ros::object
  :slots (_direction ))

(defmethod mirte_msgs::get_virtual_colorRequest
  (:init
   (&key
    ((:direction __direction) "")
    )
   (send-super :init)
   (setq _direction (string __direction))
   self)
  (:direction
   (&optional __direction)
   (if __direction (setq _direction __direction)) _direction)
  (:serialization-length
   ()
   (+
    ;; string _direction
    4 (length _direction)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _direction
       (write-long (length _direction) s) (princ _direction s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _direction
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _direction (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass mirte_msgs::get_virtual_colorResponse
  :super ros::object
  :slots (_data ))

(defmethod mirte_msgs::get_virtual_colorResponse
  (:init
   (&key
    ((:data __data) (instance mirte_msgs::color :init))
    )
   (send-super :init)
   (setq _data __data)
   self)
  (:data
   (&rest __data)
   (if (keywordp (car __data))
       (send* _data __data)
     (progn
       (if __data (setq _data (car __data)))
       _data)))
  (:serialization-length
   ()
   (+
    ;; mirte_msgs/color _data
    (send _data :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; mirte_msgs/color _data
       (send _data :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; mirte_msgs/color _data
     (send _data :deserialize buf ptr-) (incf ptr- (send _data :serialization-length))
   ;;
   self)
  )

(defclass mirte_msgs::get_virtual_color
  :super ros::object
  :slots ())

(setf (get mirte_msgs::get_virtual_color :md5sum-) "8ce5cc0c46b1c53043e61c14e381c577")
(setf (get mirte_msgs::get_virtual_color :datatype-) "mirte_msgs/get_virtual_color")
(setf (get mirte_msgs::get_virtual_color :request) mirte_msgs::get_virtual_colorRequest)
(setf (get mirte_msgs::get_virtual_color :response) mirte_msgs::get_virtual_colorResponse)

(defmethod mirte_msgs::get_virtual_colorRequest
  (:response () (instance mirte_msgs::get_virtual_colorResponse :init)))

(setf (get mirte_msgs::get_virtual_colorRequest :md5sum-) "8ce5cc0c46b1c53043e61c14e381c577")
(setf (get mirte_msgs::get_virtual_colorRequest :datatype-) "mirte_msgs/get_virtual_colorRequest")
(setf (get mirte_msgs::get_virtual_colorRequest :definition-)
      "string direction
---
mirte_msgs/color data


================================================================================
MSG: mirte_msgs/color
int32 red
int32 green
")

(setf (get mirte_msgs::get_virtual_colorResponse :md5sum-) "8ce5cc0c46b1c53043e61c14e381c577")
(setf (get mirte_msgs::get_virtual_colorResponse :datatype-) "mirte_msgs/get_virtual_colorResponse")
(setf (get mirte_msgs::get_virtual_colorResponse :definition-)
      "string direction
---
mirte_msgs/color data


================================================================================
MSG: mirte_msgs/color
int32 red
int32 green
")



(provide :mirte_msgs/get_virtual_color "8ce5cc0c46b1c53043e61c14e381c577")


