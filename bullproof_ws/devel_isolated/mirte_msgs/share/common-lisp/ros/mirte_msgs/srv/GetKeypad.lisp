; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetKeypad-request.msg.html

(cl:defclass <GetKeypad-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetKeypad-request (<GetKeypad-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKeypad-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKeypad-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetKeypad-request> is deprecated: use mirte_msgs-srv:GetKeypad-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKeypad-request>) ostream)
  "Serializes a message object of type '<GetKeypad-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKeypad-request>) istream)
  "Deserializes a message object of type '<GetKeypad-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKeypad-request>)))
  "Returns string type for a service object of type '<GetKeypad-request>"
  "mirte_msgs/GetKeypadRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKeypad-request)))
  "Returns string type for a service object of type 'GetKeypad-request"
  "mirte_msgs/GetKeypadRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKeypad-request>)))
  "Returns md5sum for a message object of type '<GetKeypad-request>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKeypad-request)))
  "Returns md5sum for a message object of type 'GetKeypad-request"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKeypad-request>)))
  "Returns full string definition for message of type '<GetKeypad-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKeypad-request)))
  "Returns full string definition for message of type 'GetKeypad-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKeypad-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKeypad-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKeypad-request
))
;//! \htmlinclude GetKeypad-response.msg.html

(cl:defclass <GetKeypad-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass GetKeypad-response (<GetKeypad-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetKeypad-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetKeypad-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetKeypad-response> is deprecated: use mirte_msgs-srv:GetKeypad-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetKeypad-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetKeypad-response>) ostream)
  "Serializes a message object of type '<GetKeypad-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetKeypad-response>) istream)
  "Deserializes a message object of type '<GetKeypad-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'data) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'data) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetKeypad-response>)))
  "Returns string type for a service object of type '<GetKeypad-response>"
  "mirte_msgs/GetKeypadResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKeypad-response)))
  "Returns string type for a service object of type 'GetKeypad-response"
  "mirte_msgs/GetKeypadResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetKeypad-response>)))
  "Returns md5sum for a message object of type '<GetKeypad-response>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetKeypad-response)))
  "Returns md5sum for a message object of type 'GetKeypad-response"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetKeypad-response>)))
  "Returns full string definition for message of type '<GetKeypad-response>"
  (cl:format cl:nil "string data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetKeypad-response)))
  "Returns full string definition for message of type 'GetKeypad-response"
  (cl:format cl:nil "string data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetKeypad-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetKeypad-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetKeypad-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetKeypad)))
  'GetKeypad-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetKeypad)))
  'GetKeypad-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetKeypad)))
  "Returns string type for a service object of type '<GetKeypad>"
  "mirte_msgs/GetKeypad")