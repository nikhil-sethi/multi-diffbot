; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetEncoder-request.msg.html

(cl:defclass <GetEncoder-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetEncoder-request (<GetEncoder-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEncoder-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEncoder-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetEncoder-request> is deprecated: use mirte_msgs-srv:GetEncoder-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEncoder-request>) ostream)
  "Serializes a message object of type '<GetEncoder-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEncoder-request>) istream)
  "Deserializes a message object of type '<GetEncoder-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEncoder-request>)))
  "Returns string type for a service object of type '<GetEncoder-request>"
  "mirte_msgs/GetEncoderRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEncoder-request)))
  "Returns string type for a service object of type 'GetEncoder-request"
  "mirte_msgs/GetEncoderRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEncoder-request>)))
  "Returns md5sum for a message object of type '<GetEncoder-request>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEncoder-request)))
  "Returns md5sum for a message object of type 'GetEncoder-request"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEncoder-request>)))
  "Returns full string definition for message of type '<GetEncoder-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEncoder-request)))
  "Returns full string definition for message of type 'GetEncoder-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEncoder-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEncoder-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEncoder-request
))
;//! \htmlinclude GetEncoder-response.msg.html

(cl:defclass <GetEncoder-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass GetEncoder-response (<GetEncoder-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetEncoder-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetEncoder-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetEncoder-response> is deprecated: use mirte_msgs-srv:GetEncoder-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetEncoder-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetEncoder-response>) ostream)
  "Serializes a message object of type '<GetEncoder-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetEncoder-response>) istream)
  "Deserializes a message object of type '<GetEncoder-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetEncoder-response>)))
  "Returns string type for a service object of type '<GetEncoder-response>"
  "mirte_msgs/GetEncoderResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEncoder-response)))
  "Returns string type for a service object of type 'GetEncoder-response"
  "mirte_msgs/GetEncoderResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetEncoder-response>)))
  "Returns md5sum for a message object of type '<GetEncoder-response>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetEncoder-response)))
  "Returns md5sum for a message object of type 'GetEncoder-response"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetEncoder-response>)))
  "Returns full string definition for message of type '<GetEncoder-response>"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetEncoder-response)))
  "Returns full string definition for message of type 'GetEncoder-response"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetEncoder-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetEncoder-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetEncoder-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetEncoder)))
  'GetEncoder-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetEncoder)))
  'GetEncoder-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetEncoder)))
  "Returns string type for a service object of type '<GetEncoder>"
  "mirte_msgs/GetEncoder")