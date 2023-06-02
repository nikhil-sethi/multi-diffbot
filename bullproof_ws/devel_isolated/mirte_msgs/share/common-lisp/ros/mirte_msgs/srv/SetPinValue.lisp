; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude SetPinValue-request.msg.html

(cl:defclass <SetPinValue-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:integer
    :initform 0))
)

(cl:defclass SetPinValue-request (<SetPinValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPinValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPinValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetPinValue-request> is deprecated: use mirte_msgs-srv:SetPinValue-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <SetPinValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:pin-val is deprecated.  Use mirte_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SetPinValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:type-val is deprecated.  Use mirte_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetPinValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:value-val is deprecated.  Use mirte_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPinValue-request>) ostream)
  "Serializes a message object of type '<SetPinValue-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'pin))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'pin))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let* ((signed (cl:slot-value msg 'value)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPinValue-request>) istream)
  "Deserializes a message object of type '<SetPinValue-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'pin) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'pin) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPinValue-request>)))
  "Returns string type for a service object of type '<SetPinValue-request>"
  "mirte_msgs/SetPinValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinValue-request)))
  "Returns string type for a service object of type 'SetPinValue-request"
  "mirte_msgs/SetPinValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPinValue-request>)))
  "Returns md5sum for a message object of type '<SetPinValue-request>"
  "ca22cfaaa89cb1d56388f4014610ebb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPinValue-request)))
  "Returns md5sum for a message object of type 'SetPinValue-request"
  "ca22cfaaa89cb1d56388f4014610ebb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPinValue-request>)))
  "Returns full string definition for message of type '<SetPinValue-request>"
  (cl:format cl:nil "string pin~%string type~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPinValue-request)))
  "Returns full string definition for message of type 'SetPinValue-request"
  (cl:format cl:nil "string pin~%string type~%int32 value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPinValue-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pin))
     4 (cl:length (cl:slot-value msg 'type))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPinValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPinValue-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetPinValue-response.msg.html

(cl:defclass <SetPinValue-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetPinValue-response (<SetPinValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetPinValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetPinValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetPinValue-response> is deprecated: use mirte_msgs-srv:SetPinValue-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetPinValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:status-val is deprecated.  Use mirte_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetPinValue-response>) ostream)
  "Serializes a message object of type '<SetPinValue-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetPinValue-response>) istream)
  "Deserializes a message object of type '<SetPinValue-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetPinValue-response>)))
  "Returns string type for a service object of type '<SetPinValue-response>"
  "mirte_msgs/SetPinValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinValue-response)))
  "Returns string type for a service object of type 'SetPinValue-response"
  "mirte_msgs/SetPinValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetPinValue-response>)))
  "Returns md5sum for a message object of type '<SetPinValue-response>"
  "ca22cfaaa89cb1d56388f4014610ebb5")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetPinValue-response)))
  "Returns md5sum for a message object of type 'SetPinValue-response"
  "ca22cfaaa89cb1d56388f4014610ebb5")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetPinValue-response>)))
  "Returns full string definition for message of type '<SetPinValue-response>"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetPinValue-response)))
  "Returns full string definition for message of type 'SetPinValue-response"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetPinValue-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetPinValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetPinValue-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetPinValue)))
  'SetPinValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetPinValue)))
  'SetPinValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetPinValue)))
  "Returns string type for a service object of type '<SetPinValue>"
  "mirte_msgs/SetPinValue")