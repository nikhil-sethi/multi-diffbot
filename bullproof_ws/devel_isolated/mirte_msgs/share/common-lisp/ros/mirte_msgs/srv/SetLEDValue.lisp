; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude SetLEDValue-request.msg.html

(cl:defclass <SetLEDValue-request> (roslisp-msg-protocol:ros-message)
  ((value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass SetLEDValue-request (<SetLEDValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLEDValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLEDValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetLEDValue-request> is deprecated: use mirte_msgs-srv:SetLEDValue-request instead.")))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetLEDValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:value-val is deprecated.  Use mirte_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLEDValue-request>) ostream)
  "Serializes a message object of type '<SetLEDValue-request>"
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLEDValue-request>) istream)
  "Deserializes a message object of type '<SetLEDValue-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLEDValue-request>)))
  "Returns string type for a service object of type '<SetLEDValue-request>"
  "mirte_msgs/SetLEDValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDValue-request)))
  "Returns string type for a service object of type 'SetLEDValue-request"
  "mirte_msgs/SetLEDValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLEDValue-request>)))
  "Returns md5sum for a message object of type '<SetLEDValue-request>"
  "42048ea920ba4286ecdfb4bc68063c32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLEDValue-request)))
  "Returns md5sum for a message object of type 'SetLEDValue-request"
  "42048ea920ba4286ecdfb4bc68063c32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLEDValue-request>)))
  "Returns full string definition for message of type '<SetLEDValue-request>"
  (cl:format cl:nil "int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLEDValue-request)))
  "Returns full string definition for message of type 'SetLEDValue-request"
  (cl:format cl:nil "int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLEDValue-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLEDValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLEDValue-request
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetLEDValue-response.msg.html

(cl:defclass <SetLEDValue-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetLEDValue-response (<SetLEDValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetLEDValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetLEDValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetLEDValue-response> is deprecated: use mirte_msgs-srv:SetLEDValue-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetLEDValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:status-val is deprecated.  Use mirte_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetLEDValue-response>) ostream)
  "Serializes a message object of type '<SetLEDValue-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetLEDValue-response>) istream)
  "Deserializes a message object of type '<SetLEDValue-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetLEDValue-response>)))
  "Returns string type for a service object of type '<SetLEDValue-response>"
  "mirte_msgs/SetLEDValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDValue-response)))
  "Returns string type for a service object of type 'SetLEDValue-response"
  "mirte_msgs/SetLEDValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetLEDValue-response>)))
  "Returns md5sum for a message object of type '<SetLEDValue-response>"
  "42048ea920ba4286ecdfb4bc68063c32")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetLEDValue-response)))
  "Returns md5sum for a message object of type 'SetLEDValue-response"
  "42048ea920ba4286ecdfb4bc68063c32")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetLEDValue-response>)))
  "Returns full string definition for message of type '<SetLEDValue-response>"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetLEDValue-response)))
  "Returns full string definition for message of type 'SetLEDValue-response"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetLEDValue-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetLEDValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetLEDValue-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetLEDValue)))
  'SetLEDValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetLEDValue)))
  'SetLEDValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetLEDValue)))
  "Returns string type for a service object of type '<SetLEDValue>"
  "mirte_msgs/SetLEDValue")