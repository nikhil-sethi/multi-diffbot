; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-msg)


;//! \htmlinclude Intensity.msg.html

(cl:defclass <Intensity> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass Intensity (<Intensity>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Intensity>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Intensity)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-msg:<Intensity> is deprecated: use mirte_msgs-msg:Intensity instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Intensity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:header-val is deprecated.  Use mirte_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <Intensity>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:value-val is deprecated.  Use mirte_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Intensity>) ostream)
  "Serializes a message object of type '<Intensity>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Intensity>) istream)
  "Deserializes a message object of type '<Intensity>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Intensity>)))
  "Returns string type for a message object of type '<Intensity>"
  "mirte_msgs/Intensity")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Intensity)))
  "Returns string type for a message object of type 'Intensity"
  "mirte_msgs/Intensity")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Intensity>)))
  "Returns md5sum for a message object of type '<Intensity>"
  "f231299e824089b22a987717bd7ace62")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Intensity)))
  "Returns md5sum for a message object of type 'Intensity"
  "f231299e824089b22a987717bd7ace62")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Intensity>)))
  "Returns full string definition for message of type '<Intensity>"
  (cl:format cl:nil "Header header~%int32 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Intensity)))
  "Returns full string definition for message of type 'Intensity"
  (cl:format cl:nil "Header header~%int32 value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Intensity>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Intensity>))
  "Converts a ROS message object to a list"
  (cl:list 'Intensity
    (cl:cons ':header (header msg))
    (cl:cons ':value (value msg))
))
