;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::color)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'color (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::COLOR")
  (make-package "MIRTE_MSGS::COLOR"))

(in-package "ROS")
;;//! \htmlinclude color.msg.html


(defclass mirte_msgs::color
  :super ros::object
  :slots (_red _green _blue ))

(defmethod mirte_msgs::color
  (:init
   (&key
    ((:red __red) 0)
    ((:green __green) 0)
    ((:blue __blue) 0)
    )
   (send-super :init)
   (setq _red (round __red))
   (setq _green (round __green))
   (setq _blue (round __blue))
   self)
  (:red
   (&optional __red)
   (if __red (setq _red __red)) _red)
  (:green
   (&optional __green)
   (if __green (setq _green __green)) _green)
  (:blue
   (&optional __blue)
   (if __blue (setq _blue __blue)) _blue)
  (:serialization-length
   ()
   (+
    ;; int32 _red
    4
    ;; int32 _green
    4
    ;; int32 _blue
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _red
       (write-long _red s)
     ;; int32 _green
       (write-long _green s)
     ;; int32 _blue
       (write-long _blue s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _red
     (setq _red (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _green
     (setq _green (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _blue
     (setq _blue (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get mirte_msgs::color :md5sum-) "c68492118bb6d7180f53625820775c5d")
(setf (get mirte_msgs::color :datatype-) "mirte_msgs/color")
(setf (get mirte_msgs::color :definition-)
      "int32 red
int32 green
int32 blue
")



(provide :mirte_msgs/color "c68492118bb6d7180f53625820775c5d")


