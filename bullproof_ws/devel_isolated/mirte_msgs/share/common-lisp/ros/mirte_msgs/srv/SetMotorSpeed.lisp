; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude SetMotorSpeed-request.msg.html

(cl:defclass <SetMotorSpeed-request> (roslisp-msg-protocol:ros-message)
  ((speed
    :reader speed
    :initarg :speed
    :type cl:integer
    :initform 0))
)

(cl:defclass SetMotorSpeed-request (<SetMotorSpeed-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotorSpeed-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotorSpeed-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetMotorSpeed-request> is deprecated: use mirte_msgs-srv:SetMotorSpeed-request instead.")))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <SetMotorSpeed-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:speed-val is deprecated.  Use mirte_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotorSpeed-request>) ostream)
  "Serializes a message object of type '<SetMotorSpeed-request>"
  (cl:let* ((signed (cl:slot-value msg 'speed)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotorSpeed-request>) istream)
  "Deserializes a message object of type '<SetMotorSpeed-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'speed) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotorSpeed-request>)))
  "Returns string type for a service object of type '<SetMotorSpeed-request>"
  "mirte_msgs/SetMotorSpeedRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorSpeed-request)))
  "Returns string type for a service object of type 'SetMotorSpeed-request"
  "mirte_msgs/SetMotorSpeedRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotorSpeed-request>)))
  "Returns md5sum for a message object of type '<SetMotorSpeed-request>"
  "ec6a2ec4df43df120c9889aac5800d51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotorSpeed-request)))
  "Returns md5sum for a message object of type 'SetMotorSpeed-request"
  "ec6a2ec4df43df120c9889aac5800d51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotorSpeed-request>)))
  "Returns full string definition for message of type '<SetMotorSpeed-request>"
  (cl:format cl:nil "int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotorSpeed-request)))
  "Returns full string definition for message of type 'SetMotorSpeed-request"
  (cl:format cl:nil "int32 speed~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotorSpeed-request>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotorSpeed-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotorSpeed-request
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude SetMotorSpeed-response.msg.html

(cl:defclass <SetMotorSpeed-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetMotorSpeed-response (<SetMotorSpeed-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetMotorSpeed-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetMotorSpeed-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetMotorSpeed-response> is deprecated: use mirte_msgs-srv:SetMotorSpeed-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetMotorSpeed-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:status-val is deprecated.  Use mirte_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetMotorSpeed-response>) ostream)
  "Serializes a message object of type '<SetMotorSpeed-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetMotorSpeed-response>) istream)
  "Deserializes a message object of type '<SetMotorSpeed-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetMotorSpeed-response>)))
  "Returns string type for a service object of type '<SetMotorSpeed-response>"
  "mirte_msgs/SetMotorSpeedResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorSpeed-response)))
  "Returns string type for a service object of type 'SetMotorSpeed-response"
  "mirte_msgs/SetMotorSpeedResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetMotorSpeed-response>)))
  "Returns md5sum for a message object of type '<SetMotorSpeed-response>"
  "ec6a2ec4df43df120c9889aac5800d51")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetMotorSpeed-response)))
  "Returns md5sum for a message object of type 'SetMotorSpeed-response"
  "ec6a2ec4df43df120c9889aac5800d51")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetMotorSpeed-response>)))
  "Returns full string definition for message of type '<SetMotorSpeed-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetMotorSpeed-response)))
  "Returns full string definition for message of type 'SetMotorSpeed-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetMotorSpeed-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetMotorSpeed-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetMotorSpeed-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetMotorSpeed)))
  'SetMotorSpeed-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetMotorSpeed)))
  'SetMotorSpeed-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetMotorSpeed)))
  "Returns string type for a service object of type '<SetMotorSpeed>"
  "mirte_msgs/SetMotorSpeed")