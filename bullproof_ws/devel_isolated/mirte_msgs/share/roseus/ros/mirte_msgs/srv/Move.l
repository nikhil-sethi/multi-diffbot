;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::Move)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'Move (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::MOVE")
  (make-package "MIRTE_MSGS::MOVE"))
(unless (find-package "MIRTE_MSGS::MOVEREQUEST")
  (make-package "MIRTE_MSGS::MOVEREQUEST"))
(unless (find-package "MIRTE_MSGS::MOVERESPONSE")
  (make-package "MIRTE_MSGS::MOVERESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::MoveRequest
  :super ros::object
  :slots (_distance _speed ))

(defmethod mirte_msgs::MoveRequest
  (:init
   (&key
    ((:distance __distance) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _distance (float __distance))
   (setq _speed (float __speed))
   self)
  (:distance
   (&optional __distance)
   (if __distance (setq _distance __distance)) _distance)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _distance
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _distance
       (sys::poke _distance (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _distance
     (setq _distance (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::MoveResponse
  :super ros::object
  :slots (_finished ))

(defmethod mirte_msgs::MoveResponse
  (:init
   (&key
    ((:finished __finished) nil)
    )
   (send-super :init)
   (setq _finished __finished)
   self)
  (:finished
   (&optional (__finished :null))
   (if (not (eq __finished :null)) (setq _finished __finished)) _finished)
  (:serialization-length
   ()
   (+
    ;; bool _finished
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _finished
       (if _finished (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _finished
     (setq _finished (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass mirte_msgs::Move
  :super ros::object
  :slots ())

(setf (get mirte_msgs::Move :md5sum-) "6b4f1d96a2bee68005f89c9b5b4fba0d")
(setf (get mirte_msgs::Move :datatype-) "mirte_msgs/Move")
(setf (get mirte_msgs::Move :request) mirte_msgs::MoveRequest)
(setf (get mirte_msgs::Move :response) mirte_msgs::MoveResponse)

(defmethod mirte_msgs::MoveRequest
  (:response () (instance mirte_msgs::MoveResponse :init)))

(setf (get mirte_msgs::MoveRequest :md5sum-) "6b4f1d96a2bee68005f89c9b5b4fba0d")
(setf (get mirte_msgs::MoveRequest :datatype-) "mirte_msgs/MoveRequest")
(setf (get mirte_msgs::MoveRequest :definition-)
      "float32 distance    # m
float32 speed       # m/s
---
bool finished

")

(setf (get mirte_msgs::MoveResponse :md5sum-) "6b4f1d96a2bee68005f89c9b5b4fba0d")
(setf (get mirte_msgs::MoveResponse :datatype-) "mirte_msgs/MoveResponse")
(setf (get mirte_msgs::MoveResponse :definition-)
      "float32 distance    # m
float32 speed       # m/s
---
bool finished

")



(provide :mirte_msgs/Move "6b4f1d96a2bee68005f89c9b5b4fba0d")


