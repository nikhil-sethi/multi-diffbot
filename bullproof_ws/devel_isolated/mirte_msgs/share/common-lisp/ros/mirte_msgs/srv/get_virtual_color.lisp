; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude get_virtual_color-request.msg.html

(cl:defclass <get_virtual_color-request> (roslisp-msg-protocol:ros-message)
  ((direction
    :reader direction
    :initarg :direction
    :type cl:string
    :initform ""))
)

(cl:defclass get_virtual_color-request (<get_virtual_color-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_virtual_color-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_virtual_color-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<get_virtual_color-request> is deprecated: use mirte_msgs-srv:get_virtual_color-request instead.")))

(cl:ensure-generic-function 'direction-val :lambda-list '(m))
(cl:defmethod direction-val ((m <get_virtual_color-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:direction-val is deprecated.  Use mirte_msgs-srv:direction instead.")
  (direction m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_virtual_color-request>) ostream)
  "Serializes a message object of type '<get_virtual_color-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'direction))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'direction))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_virtual_color-request>) istream)
  "Deserializes a message object of type '<get_virtual_color-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'direction) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'direction) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_virtual_color-request>)))
  "Returns string type for a service object of type '<get_virtual_color-request>"
  "mirte_msgs/get_virtual_colorRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_virtual_color-request)))
  "Returns string type for a service object of type 'get_virtual_color-request"
  "mirte_msgs/get_virtual_colorRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_virtual_color-request>)))
  "Returns md5sum for a message object of type '<get_virtual_color-request>"
  "8ce5cc0c46b1c53043e61c14e381c577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_virtual_color-request)))
  "Returns md5sum for a message object of type 'get_virtual_color-request"
  "8ce5cc0c46b1c53043e61c14e381c577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_virtual_color-request>)))
  "Returns full string definition for message of type '<get_virtual_color-request>"
  (cl:format cl:nil "string direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_virtual_color-request)))
  "Returns full string definition for message of type 'get_virtual_color-request"
  (cl:format cl:nil "string direction~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_virtual_color-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'direction))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_virtual_color-request>))
  "Converts a ROS message object to a list"
  (cl:list 'get_virtual_color-request
    (cl:cons ':direction (direction msg))
))
;//! \htmlinclude get_virtual_color-response.msg.html

(cl:defclass <get_virtual_color-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type mirte_msgs-msg:color
    :initform (cl:make-instance 'mirte_msgs-msg:color)))
)

(cl:defclass get_virtual_color-response (<get_virtual_color-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <get_virtual_color-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'get_virtual_color-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<get_virtual_color-response> is deprecated: use mirte_msgs-srv:get_virtual_color-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <get_virtual_color-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <get_virtual_color-response>) ostream)
  "Serializes a message object of type '<get_virtual_color-response>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'data) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <get_virtual_color-response>) istream)
  "Deserializes a message object of type '<get_virtual_color-response>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'data) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<get_virtual_color-response>)))
  "Returns string type for a service object of type '<get_virtual_color-response>"
  "mirte_msgs/get_virtual_colorResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_virtual_color-response)))
  "Returns string type for a service object of type 'get_virtual_color-response"
  "mirte_msgs/get_virtual_colorResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<get_virtual_color-response>)))
  "Returns md5sum for a message object of type '<get_virtual_color-response>"
  "8ce5cc0c46b1c53043e61c14e381c577")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'get_virtual_color-response)))
  "Returns md5sum for a message object of type 'get_virtual_color-response"
  "8ce5cc0c46b1c53043e61c14e381c577")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<get_virtual_color-response>)))
  "Returns full string definition for message of type '<get_virtual_color-response>"
  (cl:format cl:nil "mirte_msgs/color data~%~%~%================================================================================~%MSG: mirte_msgs/color~%int32 red~%int32 green~%int32 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'get_virtual_color-response)))
  "Returns full string definition for message of type 'get_virtual_color-response"
  (cl:format cl:nil "mirte_msgs/color data~%~%~%================================================================================~%MSG: mirte_msgs/color~%int32 red~%int32 green~%int32 blue~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <get_virtual_color-response>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'data))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <get_virtual_color-response>))
  "Converts a ROS message object to a list"
  (cl:list 'get_virtual_color-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'get_virtual_color)))
  'get_virtual_color-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'get_virtual_color)))
  'get_virtual_color-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'get_virtual_color)))
  "Returns string type for a service object of type '<get_virtual_color>"
  "mirte_msgs/get_virtual_color")