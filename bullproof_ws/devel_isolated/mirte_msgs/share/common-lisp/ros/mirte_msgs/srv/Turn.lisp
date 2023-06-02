; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude Turn-request.msg.html

(cl:defclass <Turn-request> (roslisp-msg-protocol:ros-message)
  ((angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Turn-request (<Turn-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Turn-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Turn-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<Turn-request> is deprecated: use mirte_msgs-srv:Turn-request instead.")))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <Turn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:angle-val is deprecated.  Use mirte_msgs-srv:angle instead.")
  (angle m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Turn-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:speed-val is deprecated.  Use mirte_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Turn-request>) ostream)
  "Serializes a message object of type '<Turn-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'speed))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Turn-request>) istream)
  "Deserializes a message object of type '<Turn-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Turn-request>)))
  "Returns string type for a service object of type '<Turn-request>"
  "mirte_msgs/TurnRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Turn-request)))
  "Returns string type for a service object of type 'Turn-request"
  "mirte_msgs/TurnRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Turn-request>)))
  "Returns md5sum for a message object of type '<Turn-request>"
  "55b88347e6bff17eb9eddc282d36da9d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Turn-request)))
  "Returns md5sum for a message object of type 'Turn-request"
  "55b88347e6bff17eb9eddc282d36da9d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Turn-request>)))
  "Returns full string definition for message of type '<Turn-request>"
  (cl:format cl:nil "float32 angle       # rad~%float32 speed       # rad/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Turn-request)))
  "Returns full string definition for message of type 'Turn-request"
  (cl:format cl:nil "float32 angle       # rad~%float32 speed       # rad/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Turn-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Turn-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Turn-request
    (cl:cons ':angle (angle msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude Turn-response.msg.html

(cl:defclass <Turn-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Turn-response (<Turn-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Turn-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Turn-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<Turn-response> is deprecated: use mirte_msgs-srv:Turn-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <Turn-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:finished-val is deprecated.  Use mirte_msgs-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Turn-response>) ostream)
  "Serializes a message object of type '<Turn-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Turn-response>) istream)
  "Deserializes a message object of type '<Turn-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Turn-response>)))
  "Returns string type for a service object of type '<Turn-response>"
  "mirte_msgs/TurnResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Turn-response)))
  "Returns string type for a service object of type 'Turn-response"
  "mirte_msgs/TurnResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Turn-response>)))
  "Returns md5sum for a message object of type '<Turn-response>"
  "55b88347e6bff17eb9eddc282d36da9d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Turn-response)))
  "Returns md5sum for a message object of type 'Turn-response"
  "55b88347e6bff17eb9eddc282d36da9d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Turn-response>)))
  "Returns full string definition for message of type '<Turn-response>"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Turn-response)))
  "Returns full string definition for message of type 'Turn-response"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Turn-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Turn-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Turn-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Turn)))
  'Turn-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Turn)))
  'Turn-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Turn)))
  "Returns string type for a service object of type '<Turn>"
  "mirte_msgs/Turn")