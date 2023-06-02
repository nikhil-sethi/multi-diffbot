; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude SetPinMode-request.msg.html

(cl:defclass <SetPinMode-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:integer
    :initform 0)
   (mode
    :reader mode
    :initarg :mode
    :type cl:integer
    :initform 0)
   (type
    :reader type
    :initarg :type
    :type cl:integer
    :initform 0))
)

(cl:defclass SetPinMode-request (<SetPinMode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPinMode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPinMode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetPinMode-request> is deprecated: use mirte_msgs-srv:SetPinMode-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <SetPinMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:pin-val is deprecated.  Use mirte_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <SetPinMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:mode-val is deprecated.  Use mirte_msgs-srv:mode instead.")
  (mode m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SetPinMode-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:type-val is deprecated.  Use mirte_msgs-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPinMode-request>) ostream)
  "Serializes a message object of type '<SetPinMode-request>"
  (cl:let* ((signed (cl:slot-value msg 'pin)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'mode)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'type)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPinMode-request>) istream)
  "Deserializes a message object of type '<SetPinMode-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pin) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPinMode-request>)))
  "Returns string type for a service object of type '<SetPinMode-request>"
  "mirte_msgs/SetPinModeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinMode-request)))
  "Returns string type for a service object of type 'SetPinMode-request"
  "mirte_msgs/SetPinModeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPinMode-request>)))
  "Returns md5sum for a message object of type '<SetPinMode-request>"
  "f8d9d67f15a539c1c17373473c0a782d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPinMode-request)))
  "Returns md5sum for a message object of type 'SetPinMode-request"
  "f8d9d67f15a539c1c17373473c0a782d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPinMode-request>)))
  "Returns full string definition for message of type '<SetPinMode-request>"
  (cl:format cl:nil "int32 pin~%int32 mode~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPinMode-request)))
  "Returns full string definition for message of type 'SetPinMode-request"
  (cl:format cl:nil "int32 pin~%int32 mode~%int32 type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPinMode-request>))
  (cl:+ 0
     4
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPinMode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPinMode-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':mode (mode msg))
    (cl:cons ':type (type msg))
))
;//! \htmlinclude SetPinMode-response.msg.html

(cl:defclass <SetPinMode-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPinMode-response (<SetPinMode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPinMode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPinMode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetPinMode-response> is deprecated: use mirte_msgs-srv:SetPinMode-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetPinMode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:status-val is deprecated.  Use mirte_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPinMode-response>) ostream)
  "Serializes a message object of type '<SetPinMode-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPinMode-response>) istream)
  "Deserializes a message object of type '<SetPinMode-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPinMode-response>)))
  "Returns string type for a service object of type '<SetPinMode-response>"
  "mirte_msgs/SetPinModeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinMode-response)))
  "Returns string type for a service object of type 'SetPinMode-response"
  "mirte_msgs/SetPinModeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPinMode-response>)))
  "Returns md5sum for a message object of type '<SetPinMode-response>"
  "f8d9d67f15a539c1c17373473c0a782d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPinMode-response)))
  "Returns md5sum for a message object of type 'SetPinMode-response"
  "f8d9d67f15a539c1c17373473c0a782d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPinMode-response>)))
  "Returns full string definition for message of type '<SetPinMode-response>"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPinMode-response)))
  "Returns full string definition for message of type 'SetPinMode-response"
  (cl:format cl:nil "bool status~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPinMode-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPinMode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPinMode-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPinMode)))
  'SetPinMode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPinMode)))
  'SetPinMode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinMode)))
  "Returns string type for a service object of type '<SetPinMode>"
  "mirte_msgs/SetPinMode")