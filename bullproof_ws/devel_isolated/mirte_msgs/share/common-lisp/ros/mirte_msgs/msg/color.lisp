; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-msg)


;//! \htmlinclude color.msg.html

(cl:defclass <color> (roslisp-msg-protocol:ros-message)
  ((red
    :reader red
    :initarg :red
    :type cl:integer
    :initform 0)
   (green
    :reader green
    :initarg :green
    :type cl:integer
    :initform 0)
   (blue
    :reader blue
    :initarg :blue
    :type cl:integer
    :initform 0))
)

(cl:defclass color (<color>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <color>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'color)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-msg:<color> is deprecated: use mirte_msgs-msg:color instead.")))

(cl:ensure-generic-function 'red-val :lambda-list '(m))
(cl:defmethod red-val ((m <color>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:red-val is deprecated.  Use mirte_msgs-msg:red instead.")
  (red m))

(cl:ensure-generic-function 'green-val :lambda-list '(m))
(cl:defmethod green-val ((m <color>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:green-val is deprecated.  Use mirte_msgs-msg:green instead.")
  (green m))

(cl:ensure-generic-function 'blue-val :lambda-list '(m))
(cl:defmethod blue-val ((m <color>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-msg:blue-val is deprecated.  Use mirte_msgs-msg:blue instead.")
  (blue m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <color>) ostream)
  "Serializes a message object of type '<color>"
  (cl:let* ((signed (cl:slot-value msg 'red)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'green)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'blue)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <color>) istream)
  "Deserializes a message object of type '<color>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'red) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'green) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'blue) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<color>)))
  "Returns string type for a message object of type '<color>"
  "mirte_msgs/color")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'color)))
  "Returns string type for a message object of type 'color"
  "mirte_msgs/color")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<color>)))
  "Returns md5sum for a message object of type '<color>"
  "c68492118bb6d7180f53625820775c5d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'color)))
  "Returns md5sum for a message object of type 'color"
  "c68492118bb6d7180f53625820775c5d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<color>)))
  "Returns full string definition for message of type '<color>"
  (cl:format cl:nil "int32 red~%int32 green~%int32 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'color)))
  "Returns full string definition for message of type 'color"
  (cl:format cl:nil "int32 red~%int32 green~%int32 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <color>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <color>))
  "Converts a ROS message object to a list"
  (cl:list 'color
    (cl:cons ':red (red msg))
    (cl:cons ':green (green msg))
    (cl:cons ':blue (blue msg))
))
