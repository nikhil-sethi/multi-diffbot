; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetPinValue-request.msg.html

(cl:defclass <GetPinValue-request> (roslisp-msg-protocol:ros-message)
  ((pin
    :reader pin
    :initarg :pin
    :type cl:string
    :initform "")
   (type
    :reader type
    :initarg :type
    :type cl:string
    :initform ""))
)

(cl:defclass GetPinValue-request (<GetPinValue-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPinValue-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPinValue-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetPinValue-request> is deprecated: use mirte_msgs-srv:GetPinValue-request instead.")))

(cl:ensure-generic-function 'pin-val :lambda-list '(m))
(cl:defmethod pin-val ((m <GetPinValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:pin-val is deprecated.  Use mirte_msgs-srv:pin instead.")
  (pin m))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <GetPinValue-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:type-val is deprecated.  Use mirte_msgs-srv:type instead.")
  (type m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPinValue-request>) ostream)
  "Serializes a message object of type '<GetPinValue-request>"
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
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPinValue-request>) istream)
  "Deserializes a message object of type '<GetPinValue-request>"
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
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPinValue-request>)))
  "Returns string type for a service object of type '<GetPinValue-request>"
  "mirte_msgs/GetPinValueRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPinValue-request)))
  "Returns string type for a service object of type 'GetPinValue-request"
  "mirte_msgs/GetPinValueRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPinValue-request>)))
  "Returns md5sum for a message object of type '<GetPinValue-request>"
  "277ba96458b48c83913177a820e47af8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPinValue-request)))
  "Returns md5sum for a message object of type 'GetPinValue-request"
  "277ba96458b48c83913177a820e47af8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPinValue-request>)))
  "Returns full string definition for message of type '<GetPinValue-request>"
  (cl:format cl:nil "string pin~%string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPinValue-request)))
  "Returns full string definition for message of type 'GetPinValue-request"
  (cl:format cl:nil "string pin~%string type~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPinValue-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'pin))
     4 (cl:length (cl:slot-value msg 'type))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPinValue-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPinValue-request
    (cl:cons ':pin (pin msg))
    (cl:cons ':type (type msg))
))
;//! \htmlinclude GetPinValue-response.msg.html

(cl:defclass <GetPinValue-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:integer
    :initform 0))
)

(cl:defclass GetPinValue-response (<GetPinValue-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetPinValue-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetPinValue-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetPinValue-response> is deprecated: use mirte_msgs-srv:GetPinValue-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetPinValue-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetPinValue-response>) ostream)
  "Serializes a message object of type '<GetPinValue-response>"
  (cl:let* ((signed (cl:slot-value msg 'data)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetPinValue-response>) istream)
  "Deserializes a message object of type '<GetPinValue-response>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetPinValue-response>)))
  "Returns string type for a service object of type '<GetPinValue-response>"
  "mirte_msgs/GetPinValueResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPinValue-response)))
  "Returns string type for a service object of type 'GetPinValue-response"
  "mirte_msgs/GetPinValueResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetPinValue-response>)))
  "Returns md5sum for a message object of type '<GetPinValue-response>"
  "277ba96458b48c83913177a820e47af8")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetPinValue-response)))
  "Returns md5sum for a message object of type 'GetPinValue-response"
  "277ba96458b48c83913177a820e47af8")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetPinValue-response>)))
  "Returns full string definition for message of type '<GetPinValue-response>"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetPinValue-response)))
  "Returns full string definition for message of type 'GetPinValue-response"
  (cl:format cl:nil "int32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetPinValue-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetPinValue-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetPinValue-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetPinValue)))
  'GetPinValue-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetPinValue)))
  'GetPinValue-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetPinValue)))
  "Returns string type for a service object of type '<GetPinValue>"
  "mirte_msgs/GetPinValue")