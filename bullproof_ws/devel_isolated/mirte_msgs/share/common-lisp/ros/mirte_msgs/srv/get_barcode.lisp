; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude get_barcode-request.msg.html

(cl:defclass <get_barcode-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass get_barcode-request (<get_barcode-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_barcode-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_barcode-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<get_barcode-request> is deprecated: use mirte_msgs-srv:get_barcode-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_barcode-request>) ostream)
  "Serializes a message object of type '<get_barcode-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_barcode-request>) istream)
  "Deserializes a message object of type '<get_barcode-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_barcode-request>)))
  "Returns string type for a service object of type '<get_barcode-request>"
  "mirte_msgs/get_barcodeRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_barcode-request)))
  "Returns string type for a service object of type 'get_barcode-request"
  "mirte_msgs/get_barcodeRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_barcode-request>)))
  "Returns md5sum for a message object of type '<get_barcode-request>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_barcode-request)))
  "Returns md5sum for a message object of type 'get_barcode-request"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_barcode-request>)))
  "Returns full string definition for message of type '<get_barcode-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_barcode-request)))
  "Returns full string definition for message of type 'get_barcode-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_barcode-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_barcode-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_barcode-request
))
;//! \htmlinclude get_barcode-response.msg.html

(cl:defclass <get_barcode-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:string
    :initform ""))
)

(cl:defclass get_barcode-response (<get_barcode-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_barcode-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_barcode-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<get_barcode-response> is deprecated: use mirte_msgs-srv:get_barcode-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <get_barcode-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_barcode-response>) ostream)
  "Serializes a message object of type '<get_barcode-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_barcode-response>) istream)
  "Deserializes a message object of type '<get_barcode-response>"
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
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_barcode-response>)))
  "Returns string type for a service object of type '<get_barcode-response>"
  "mirte_msgs/get_barcodeResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_barcode-response)))
  "Returns string type for a service object of type 'get_barcode-response"
  "mirte_msgs/get_barcodeResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_barcode-response>)))
  "Returns md5sum for a message object of type '<get_barcode-response>"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_barcode-response)))
  "Returns md5sum for a message object of type 'get_barcode-response"
  "992ce8a1687cec8c8bd883ec73ca41d1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_barcode-response>)))
  "Returns full string definition for message of type '<get_barcode-response>"
  (cl:format cl:nil "string data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_barcode-response)))
  "Returns full string definition for message of type 'get_barcode-response"
  (cl:format cl:nil "string data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_barcode-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_barcode-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_barcode-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_barcode)))
  'get_barcode-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_barcode)))
  'get_barcode-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_barcode)))
  "Returns string type for a service object of type '<get_barcode>"
  "mirte_msgs/get_barcode")