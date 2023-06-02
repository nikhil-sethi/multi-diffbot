; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-msg)


;//! \htmlinclude IntensityDigital.msg.html

(cl:defclass <IntensityDigital> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (value
    :reader value
    :initarg :value
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass IntensityDigital (<IntensityDigital>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IntensityDigital>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IntensityDigital)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-msg:<IntensityDigital> is deprecated: use mirte_msgs-msg:IntensityDigital instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <IntensityDigital>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:header-val is deprecated.  Use mirte_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <IntensityDigital>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:value-val is deprecated.  Use mirte_msgs-msg:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IntensityDigital>) ostream)
  "Serializes a message object of type '<IntensityDigital>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'value) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IntensityDigital>) istream)
  "Deserializes a message object of type '<IntensityDigital>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'value) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IntensityDigital>)))
  "Returns string type for a message object of type '<IntensityDigital>"
  "mirte_msgs/IntensityDigital")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IntensityDigital)))
  "Returns string type for a message object of type 'IntensityDigital"
  "mirte_msgs/IntensityDigital")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IntensityDigital>)))
  "Returns md5sum for a message object of type '<IntensityDigital>"
  "2a502021a9e661290bab60c5754fb8cd")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IntensityDigital)))
  "Returns md5sum for a message object of type 'IntensityDigital"
  "2a502021a9e661290bab60c5754fb8cd")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IntensityDigital>)))
  "Returns full string definition for message of type '<IntensityDigital>"
  (cl:format cl:nil "Header header~%bool value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IntensityDigital)))
  "Returns full string definition for message of type 'IntensityDigital"
  (cl:format cl:nil "Header header~%bool value~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IntensityDigital>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IntensityDigital>))
  "Converts a ROS message object to a list"
  (cl:list 'IntensityDigital
    (cl:cons ':header (header msg))
    (cl:cons ':value (value msg))
))
