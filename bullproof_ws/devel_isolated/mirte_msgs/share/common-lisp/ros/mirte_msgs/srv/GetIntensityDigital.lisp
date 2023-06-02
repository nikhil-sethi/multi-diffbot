; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude GetIntensityDigital-request.msg.html

(cl:defclass <GetIntensityDigital-request> (roslisp-msg-protocol:ros-message)
  ()
)

(cl:defclass GetIntensityDigital-request (<GetIntensityDigital-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntensityDigital-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntensityDigital-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetIntensityDigital-request> is deprecated: use mirte_msgs-srv:GetIntensityDigital-request instead.")))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntensityDigital-request>) ostream)
  "Serializes a message object of type '<GetIntensityDigital-request>"
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntensityDigital-request>) istream)
  "Deserializes a message object of type '<GetIntensityDigital-request>"
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntensityDigital-request>)))
  "Returns string type for a service object of type '<GetIntensityDigital-request>"
  "mirte_msgs/GetIntensityDigitalRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensityDigital-request)))
  "Returns string type for a service object of type 'GetIntensityDigital-request"
  "mirte_msgs/GetIntensityDigitalRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntensityDigital-request>)))
  "Returns md5sum for a message object of type '<GetIntensityDigital-request>"
  "8b94c1b53db61fb6aed406028ad6332a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntensityDigital-request)))
  "Returns md5sum for a message object of type 'GetIntensityDigital-request"
  "8b94c1b53db61fb6aed406028ad6332a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntensityDigital-request>)))
  "Returns full string definition for message of type '<GetIntensityDigital-request>"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntensityDigital-request)))
  "Returns full string definition for message of type 'GetIntensityDigital-request"
  (cl:format cl:nil "~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntensityDigital-request>))
  (cl:+ 0
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntensityDigital-request>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntensityDigital-request
))
;//! \htmlinclude GetIntensityDigital-response.msg.html

(cl:defclass <GetIntensityDigital-response> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass GetIntensityDigital-response (<GetIntensityDigital-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <GetIntensityDigital-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'GetIntensityDigital-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<GetIntensityDigital-response> is deprecated: use mirte_msgs-srv:GetIntensityDigital-response instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <GetIntensityDigital-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:data-val is deprecated.  Use mirte_msgs-srv:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <GetIntensityDigital-response>) ostream)
  "Serializes a message object of type '<GetIntensityDigital-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'data) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <GetIntensityDigital-response>) istream)
  "Deserializes a message object of type '<GetIntensityDigital-response>"
    (cl:setf (cl:slot-value msg 'data) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<GetIntensityDigital-response>)))
  "Returns string type for a service object of type '<GetIntensityDigital-response>"
  "mirte_msgs/GetIntensityDigitalResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensityDigital-response)))
  "Returns string type for a service object of type 'GetIntensityDigital-response"
  "mirte_msgs/GetIntensityDigitalResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<GetIntensityDigital-response>)))
  "Returns md5sum for a message object of type '<GetIntensityDigital-response>"
  "8b94c1b53db61fb6aed406028ad6332a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'GetIntensityDigital-response)))
  "Returns md5sum for a message object of type 'GetIntensityDigital-response"
  "8b94c1b53db61fb6aed406028ad6332a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<GetIntensityDigital-response>)))
  "Returns full string definition for message of type '<GetIntensityDigital-response>"
  (cl:format cl:nil "bool data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'GetIntensityDigital-response)))
  "Returns full string definition for message of type 'GetIntensityDigital-response"
  (cl:format cl:nil "bool data~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <GetIntensityDigital-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <GetIntensityDigital-response>))
  "Converts a ROS message object to a list"
  (cl:list 'GetIntensityDigital-response
    (cl:cons ':data (data msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'GetIntensityDigital)))
  'GetIntensityDigital-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'GetIntensityDigital)))
  'GetIntensityDigital-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'GetIntensityDigital)))
  "Returns string type for a service object of type '<GetIntensityDigital>"
  "mirte_msgs/GetIntensityDigital")