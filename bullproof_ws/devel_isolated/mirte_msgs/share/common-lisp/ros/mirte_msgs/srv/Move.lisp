; Auto-generated. Do not edit!


(cl:in-package mirte_msgs-srv)


;//! \htmlinclude Move-request.msg.html

(cl:defclass <Move-request> (roslisp-msg-protocol:ros-message)
  ((distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (speed
    :reader speed
    :initarg :speed
    :type cl:float
    :initform 0.0))
)

(cl:defclass Move-request (<Move-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Move-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Move-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<Move-request> is deprecated: use mirte_msgs-srv:Move-request instead.")))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <Move-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:distance-val is deprecated.  Use mirte_msgs-srv:distance instead.")
  (distance m))

(cl:ensure-generic-function 'speed-val :lambda-list '(m))
(cl:defmethod speed-val ((m <Move-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:speed-val is deprecated.  Use mirte_msgs-srv:speed instead.")
  (speed m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Move-request>) ostream)
  "Serializes a message object of type '<Move-request>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
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
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Move-request>) istream)
  "Deserializes a message object of type '<Move-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'speed) (roslisp-utils:decode-single-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Move-request>)))
  "Returns string type for a service object of type '<Move-request>"
  "mirte_msgs/MoveRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Move-request)))
  "Returns string type for a service object of type 'Move-request"
  "mirte_msgs/MoveRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Move-request>)))
  "Returns md5sum for a message object of type '<Move-request>"
  "6b4f1d96a2bee68005f89c9b5b4fba0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Move-request)))
  "Returns md5sum for a message object of type 'Move-request"
  "6b4f1d96a2bee68005f89c9b5b4fba0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Move-request>)))
  "Returns full string definition for message of type '<Move-request>"
  (cl:format cl:nil "float32 distance    # m~%float32 speed       # m/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Move-request)))
  "Returns full string definition for message of type 'Move-request"
  (cl:format cl:nil "float32 distance    # m~%float32 speed       # m/s~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Move-request>))
  (cl:+ 0
     4
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Move-request>))
  "Converts a ROS message object to a list"
  (cl:list 'Move-request
    (cl:cons ':distance (distance msg))
    (cl:cons ':speed (speed msg))
))
;//! \htmlinclude Move-response.msg.html

(cl:defclass <Move-response> (roslisp-msg-protocol:ros-message)
  ((finished
    :reader finished
    :initarg :finished
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Move-response (<Move-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Move-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Move-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mirte_msgs-srv:<Move-response> is deprecated: use mirte_msgs-srv:Move-response instead.")))

(cl:ensure-generic-function 'finished-val :lambda-list '(m))
(cl:defmethod finished-val ((m <Move-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mirte_msgs-srv:finished-val is deprecated.  Use mirte_msgs-srv:finished instead.")
  (finished m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Move-response>) ostream)
  "Serializes a message object of type '<Move-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'finished) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Move-response>) istream)
  "Deserializes a message object of type '<Move-response>"
    (cl:setf (cl:slot-value msg 'finished) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Move-response>)))
  "Returns string type for a service object of type '<Move-response>"
  "mirte_msgs/MoveResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Move-response)))
  "Returns string type for a service object of type 'Move-response"
  "mirte_msgs/MoveResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Move-response>)))
  "Returns md5sum for a message object of type '<Move-response>"
  "6b4f1d96a2bee68005f89c9b5b4fba0d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Move-response)))
  "Returns md5sum for a message object of type 'Move-response"
  "6b4f1d96a2bee68005f89c9b5b4fba0d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Move-response>)))
  "Returns full string definition for message of type '<Move-response>"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Move-response)))
  "Returns full string definition for message of type 'Move-response"
  (cl:format cl:nil "bool finished~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Move-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Move-response>))
  "Converts a ROS message object to a list"
  (cl:list 'Move-response
    (cl:cons ':finished (finished msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'Move)))
  'Move-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'Move)))
  'Move-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Move)))
  "Returns string type for a service object of type '<Move>"
  "mirte_msgs/Move")