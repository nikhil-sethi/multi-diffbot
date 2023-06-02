; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetIntensity-request.msg.html

(cl:defclass <GetIntensity-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetIntensity-request (<GetIntensity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntensity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntensity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetIntensity-request> is deprecated: use mirte_msgs-srv:GetIntensity-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntensity-request>) ostream)
  "Serializes a message object of type '<GetIntensity-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntensity-request>) istream)
  "Deserializes a message object of type '<GetIntensity-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntensity-request>)))
  "Returns string type for a service object of type '<GetIntensity-request>"
  "mirte_msgs/GetIntensityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensity-request)))
  "Returns string type for a service object of type 'GetIntensity-request"
  "mirte_msgs/GetIntensityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntensity-request>)))
  "Returns md5sum for a message object of type '<GetIntensity-request>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntensity-request)))
  "Returns md5sum for a message object of type 'GetIntensity-request"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntensity-request>)))
  "Returns full string definition for message of type '<GetIntensity-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntensity-request)))
  "Returns full string definition for message of type 'GetIntensity-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntensity-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntensity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntensity-request
))
;//! \htmlinclude GetIntensity-response.msg.html

(cl:defclass <GetIntensity-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass GetIntensity-response (<GetIntensity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntensity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntensity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetIntensity-response> is deprecated: use mirte_msgs-srv:GetIntensity-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetIntensity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntensity-response>) ostream)
  "Serializes a message object of type '<GetIntensity-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntensity-response>) istream)
  "Deserializes a message object of type '<GetIntensity-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntensity-response>)))
  "Returns string type for a service object of type '<GetIntensity-response>"
  "mirte_msgs/GetIntensityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensity-response)))
  "Returns string type for a service object of type 'GetIntensity-response"
  "mirte_msgs/GetIntensityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntensity-response>)))
  "Returns md5sum for a message object of type '<GetIntensity-response>"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntensity-response)))
  "Returns md5sum for a message object of type 'GetIntensity-response"
  "da5909fbe378aeaf85e547e830cc1bb7")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntensity-response>)))
  "Returns full string definition for message of type '<GetIntensity-response>"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntensity-response)))
  "Returns full string definition for message of type 'GetIntensity-response"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntensity-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntensity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntensity-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntensity)))
  'GetIntensity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntensity)))
  'GetIntensity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensity)))
  "Returns string type for a service object of type '<GetIntensity>"
  "mirte_msgs/GetIntensity")