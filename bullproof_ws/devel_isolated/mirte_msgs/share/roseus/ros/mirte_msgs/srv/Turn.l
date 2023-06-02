;; Auto-generated. Do not edit!


(when (boundp 'mirte_msgs::Turn)
  (if (not (find-package "MIRTE_MSGS"))
    (make-package "MIRTE_MSGS"))
  (shadow 'Turn (find-package "MIRTE_MSGS")))
(unless (find-package "MIRTE_MSGS::TURN")
  (make-package "MIRTE_MSGS::TURN"))
(unless (find-package "MIRTE_MSGS::TURNREQUEST")
  (make-package "MIRTE_MSGS::TURNREQUEST"))
(unless (find-package "MIRTE_MSGS::TURNRESPONSE")
  (make-package "MIRTE_MSGS::TURNRESPONSE"))

(in-package "ROS")





(defclass mirte_msgs::TurnRequest
  :super ros::object
  :slots (_angle _speed ))

(defmethod mirte_msgs::TurnRequest
  (:init
   (&key
    ((:angle __angle) 0.0)
    ((:speed __speed) 0.0)
    )
   (send-super :init)
   (setq _angle (float __angle))
   (setq _speed (float __speed))
   self)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:speed
   (&optional __speed)
   (if __speed (setq _speed __speed)) _speed)
  (:serialization-length
   ()
   (+
    ;; float32 _angle
    4
    ;; float32 _speed
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _speed
       (sys::poke _speed (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _speed
     (setq _speed (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(defclass mirte_msgs::TurnResponse
  :super ros::object
  :slots (_finished ))

(defmethod mirte_msgs::TurnResponse
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

(defclass mirte_msgs::Turn
  :super ros::object
  :slots ())

(setf (get mirte_msgs::Turn :md5sum-) "55b88347e6bff17eb9eddc282d36da9d")
(setf (get mirte_msgs::Turn :datatype-) "mirte_msgs/Turn")
(setf (get mirte_msgs::Turn :request) mirte_msgs::TurnRequest)
(setf (get mirte_msgs::Turn :response) mirte_msgs::TurnResponse)

(defmethod mirte_msgs::TurnRequest
  (:response () (instance mirte_msgs::TurnResponse :init)))

(setf (get mirte_msgs::TurnRequest :md5sum-) "55b88347e6bff17eb9eddc282d36da9d")
(setf (get mirte_msgs::TurnRequest :datatype-) "mirte_msgs/TurnRequest")
(setf (get mirte_msgs::TurnRequest :definition-)
      "float32 angle       # rad
float32 speed       # rad/s
---
bool finished

")

(setf (get mirte_msgs::TurnResponse :md5sum-) "55b88347e6bff17eb9eddc282d36da9d")
(setf (get mirte_msgs::TurnResponse :datatype-) "mirte_msgs/TurnResponse")
(setf (get mirte_msgs::TurnResponse :definition-)
      "float32 angle       # rad
float32 speed       # rad/s
---
bool finished

")



(provide :mirte_msgs/Turn "55b88347e6bff17eb9eddc282d36da9d")


