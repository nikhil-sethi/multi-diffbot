; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude SetOLEDImage-request.msg.html

(cl:defclass <SetOLEDImage-request> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (value
    :reader value
    :initarg :value
    :type cl:string
    :initform ""))
)

(cl:defclass SetOLEDImage-request (<SetOLEDImage-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOLEDImage-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOLEDImage-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetOLEDImage-request> is deprecated: use mirte_msgs-srv:SetOLEDImage-request instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <SetOLEDImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:type-val is deprecated.  Use mirte_msgs-srv:type instead.")
  (type m))

(cl:ensure-generic-function 'value-val :lambda-list '(m))
(cl:defmethod value-val ((m <SetOLEDImage-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:value-val is deprecated.  Use mirte_msgs-srv:value instead.")
  (value m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOLEDImage-request>) ostream)
  "Serializes a message object of type '<SetOLEDImage-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'value))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'value))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOLEDImage-request>) istream)
  "Deserializes a message object of type '<SetOLEDImage-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'value) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'value) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOLEDImage-request>)))
  "Returns string type for a service object of type '<SetOLEDImage-request>"
  "mirte_msgs/SetOLEDImageRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOLEDImage-request)))
  "Returns string type for a service object of type 'SetOLEDImage-request"
  "mirte_msgs/SetOLEDImageRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOLEDImage-request>)))
  "Returns md5sum for a message object of type '<SetOLEDImage-request>"
  "52215026eb827c220b801bf2b2c8f4d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOLEDImage-request)))
  "Returns md5sum for a message object of type 'SetOLEDImage-request"
  "52215026eb827c220b801bf2b2c8f4d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOLEDImage-request>)))
  "Returns full string definition for message of type '<SetOLEDImage-request>"
  (cl:format cl:nil "string type~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOLEDImage-request)))
  "Returns full string definition for message of type 'SetOLEDImage-request"
  (cl:format cl:nil "string type~%string value~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOLEDImage-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     4 (cl:length (cl:slot-value msg 'value))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOLEDImage-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOLEDImage-request
    (cl:cons ':type (type msg))
    (cl:cons ':value (value msg))
))
;//! \htmlinclude SetOLEDImage-response.msg.html

(cl:defclass <SetOLEDImage-response> (roslisp-msg-protocol:ros-message)
  ((status
    :reader status
    :initarg :status
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetOLEDImage-response (<SetOLEDImage-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetOLEDImage-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetOLEDImage-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<SetOLEDImage-response> is deprecated: use mirte_msgs-srv:SetOLEDImage-response instead.")))

(cl:ensure-generic-function 'status-val :lambda-list '(m))
(cl:defmethod status-val ((m <SetOLEDImage-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:status-val is deprecated.  Use mirte_msgs-srv:status instead.")
  (status m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetOLEDImage-response>) ostream)
  "Serializes a message object of type '<SetOLEDImage-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'status) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetOLEDImage-response>) istream)
  "Deserializes a message object of type '<SetOLEDImage-response>"
    (cl:setf (cl:slot-value msg 'status) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetOLEDImage-response>)))
  "Returns string type for a service object of type '<SetOLEDImage-response>"
  "mirte_msgs/SetOLEDImageResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOLEDImage-response)))
  "Returns string type for a service object of type 'SetOLEDImage-response"
  "mirte_msgs/SetOLEDImageResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetOLEDImage-response>)))
  "Returns md5sum for a message object of type '<SetOLEDImage-response>"
  "52215026eb827c220b801bf2b2c8f4d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetOLEDImage-response)))
  "Returns md5sum for a message object of type 'SetOLEDImage-response"
  "52215026eb827c220b801bf2b2c8f4d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetOLEDImage-response>)))
  "Returns full string definition for message of type '<SetOLEDImage-response>"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetOLEDImage-response)))
  "Returns full string definition for message of type 'SetOLEDImage-response"
  (cl:format cl:nil "bool status~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetOLEDImage-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetOLEDImage-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetOLEDImage-response
    (cl:cons ':status (status msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetOLEDImage)))
  'SetOLEDImage-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetOLEDImage)))
  'SetOLEDImage-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetOLEDImage)))
  "Returns string type for a service object of type '<SetOLEDImage>"
  "mirte_msgs/SetOLEDImage")