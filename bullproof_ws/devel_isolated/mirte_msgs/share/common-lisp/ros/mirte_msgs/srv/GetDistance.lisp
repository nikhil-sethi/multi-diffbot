; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetDistance-request.msg.html

(cl:defclass <GetDistance-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetDistance-request (<GetDistance-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDistance-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDistance-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetDistance-request> is deprecated: use mirte_msgs-srv:GetDistance-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDistance-request>) ostream)
  "Serializes a message object of type '<GetDistance-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDistance-request>) istream)
  "Deserializes a message object of type '<GetDistance-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDistance-request>)))
  "Returns string type for a service object of type '<GetDistance-request>"
  "mirte_msgs/GetDistanceRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDistance-request)))
  "Returns string type for a service object of type 'GetDistance-request"
  "mirte_msgs/GetDistanceRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDistance-request>)))
  "Returns md5sum for a message object of type '<GetDistance-request>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDistance-request)))
  "Returns md5sum for a message object of type 'GetDistance-request"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDistance-request>)))
  "Returns full string definition for message of type '<GetDistance-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDistance-request)))
  "Returns full string definition for message of type 'GetDistance-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDistance-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDistance-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDistance-request
))
;//! \htmlinclude GetDistance-response.msg.html

(cl:defclass <GetDistance-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:float
    :initform 0.0))
)

(cl:defclass GetDistance-response (<GetDistance-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetDistance-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetDistance-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetDistance-response> is deprecated: use mirte_msgs-srv:GetDistance-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetDistance-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetDistance-response>) ostream)
  "Serializes a message object of type '<GetDistance-response>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetDistance-response>) istream)
  "Deserializes a message object of type '<GetDistance-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'data) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetDistance-response>)))
  "Returns string type for a service object of type '<GetDistance-response>"
  "mirte_msgs/GetDistanceResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDistance-response)))
  "Returns string type for a service object of type 'GetDistance-response"
  "mirte_msgs/GetDistanceResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetDistance-response>)))
  "Returns md5sum for a message object of type '<GetDistance-response>"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetDistance-response)))
  "Returns md5sum for a message object of type 'GetDistance-response"
  "73fcbf46b49191e672908e50842a83d4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetDistance-response>)))
  "Returns full string definition for message of type '<GetDistance-response>"
  (cl:format cl:nil "float32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetDistance-response)))
  "Returns full string definition for message of type 'GetDistance-response"
  (cl:format cl:nil "float32 data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetDistance-response>))
  (cl:+ 0
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetDistance-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetDistance-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetDistance)))
  'GetDistance-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetDistance)))
  'GetDistance-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetDistance)))
  "Returns string type for a service object of type '<GetDistance>"
  "mirte_msgs/GetDistance")