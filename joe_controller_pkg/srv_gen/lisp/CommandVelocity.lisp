; Auto-generated. Do not edit!


(cl:in-package joe_controller_pkg-srv)


;//! \htmlinclude CommandVelocity-request.msg.html

(cl:defclass <CommandVelocity-request> (roslisp-msg-protocol:ros-message)
  ((linear
    :reader linear
    :initarg :linear
    :type cl:float
    :initform 0.0)
   (angular
    :reader angular
    :initarg :angular
    :type cl:float
    :initform 0.0))
)

(cl:defclass CommandVelocity-request (<CommandVelocity-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandVelocity-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandVelocity-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joe_controller_pkg-srv:<CommandVelocity-request> is deprecated: use joe_controller_pkg-srv:CommandVelocity-request instead.")))

(cl:ensure-generic-function 'linear-val :lambda-list '(m))
(cl:defmethod linear-val ((m <CommandVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joe_controller_pkg-srv:linear-val is deprecated.  Use joe_controller_pkg-srv:linear instead.")
  (linear m))

(cl:ensure-generic-function 'angular-val :lambda-list '(m))
(cl:defmethod angular-val ((m <CommandVelocity-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joe_controller_pkg-srv:angular-val is deprecated.  Use joe_controller_pkg-srv:angular instead.")
  (angular m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandVelocity-request>) ostream)
  "Serializes a message object of type '<CommandVelocity-request>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'linear))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angular))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandVelocity-request>) istream)
  "Deserializes a message object of type '<CommandVelocity-request>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'linear) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angular) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandVelocity-request>)))
  "Returns string type for a service object of type '<CommandVelocity-request>"
  "joe_controller_pkg/CommandVelocityRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandVelocity-request)))
  "Returns string type for a service object of type 'CommandVelocity-request"
  "joe_controller_pkg/CommandVelocityRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandVelocity-request>)))
  "Returns md5sum for a message object of type '<CommandVelocity-request>"
  "1b4d66b9d9e248212f1f6b29ddb376fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandVelocity-request)))
  "Returns md5sum for a message object of type 'CommandVelocity-request"
  "1b4d66b9d9e248212f1f6b29ddb376fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandVelocity-request>)))
  "Returns full string definition for message of type '<CommandVelocity-request>"
  (cl:format cl:nil "~%float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandVelocity-request)))
  "Returns full string definition for message of type 'CommandVelocity-request"
  (cl:format cl:nil "~%float64 linear~%float64 angular~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandVelocity-request>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandVelocity-request>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandVelocity-request
    (cl:cons ':linear (linear msg))
    (cl:cons ':angular (angular msg))
))
;//! \htmlinclude CommandVelocity-response.msg.html

(cl:defclass <CommandVelocity-response> (roslisp-msg-protocol:ros-message)
  ((rotation_left
    :reader rotation_left
    :initarg :rotation_left
    :type cl:float
    :initform 0.0)
   (rotation_right
    :reader rotation_right
    :initarg :rotation_right
    :type cl:float
    :initform 0.0)
   (pivot_angle
    :reader pivot_angle
    :initarg :pivot_angle
    :type cl:float
    :initform 0.0))
)

(cl:defclass CommandVelocity-response (<CommandVelocity-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <CommandVelocity-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'CommandVelocity-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name joe_controller_pkg-srv:<CommandVelocity-response> is deprecated: use joe_controller_pkg-srv:CommandVelocity-response instead.")))

(cl:ensure-generic-function 'rotation_left-val :lambda-list '(m))
(cl:defmethod rotation_left-val ((m <CommandVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joe_controller_pkg-srv:rotation_left-val is deprecated.  Use joe_controller_pkg-srv:rotation_left instead.")
  (rotation_left m))

(cl:ensure-generic-function 'rotation_right-val :lambda-list '(m))
(cl:defmethod rotation_right-val ((m <CommandVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joe_controller_pkg-srv:rotation_right-val is deprecated.  Use joe_controller_pkg-srv:rotation_right instead.")
  (rotation_right m))

(cl:ensure-generic-function 'pivot_angle-val :lambda-list '(m))
(cl:defmethod pivot_angle-val ((m <CommandVelocity-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader joe_controller_pkg-srv:pivot_angle-val is deprecated.  Use joe_controller_pkg-srv:pivot_angle instead.")
  (pivot_angle m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <CommandVelocity-response>) ostream)
  "Serializes a message object of type '<CommandVelocity-response>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotation_left))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'rotation_right))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pivot_angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <CommandVelocity-response>) istream)
  "Deserializes a message object of type '<CommandVelocity-response>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation_left) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'rotation_right) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pivot_angle) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<CommandVelocity-response>)))
  "Returns string type for a service object of type '<CommandVelocity-response>"
  "joe_controller_pkg/CommandVelocityResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandVelocity-response)))
  "Returns string type for a service object of type 'CommandVelocity-response"
  "joe_controller_pkg/CommandVelocityResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<CommandVelocity-response>)))
  "Returns md5sum for a message object of type '<CommandVelocity-response>"
  "1b4d66b9d9e248212f1f6b29ddb376fa")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'CommandVelocity-response)))
  "Returns md5sum for a message object of type 'CommandVelocity-response"
  "1b4d66b9d9e248212f1f6b29ddb376fa")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<CommandVelocity-response>)))
  "Returns full string definition for message of type '<CommandVelocity-response>"
  (cl:format cl:nil "float64 rotation_left~%float64 rotation_right~%float64 pivot_angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'CommandVelocity-response)))
  "Returns full string definition for message of type 'CommandVelocity-response"
  (cl:format cl:nil "float64 rotation_left~%float64 rotation_right~%float64 pivot_angle~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <CommandVelocity-response>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <CommandVelocity-response>))
  "Converts a ROS message object to a list"
  (cl:list 'CommandVelocity-response
    (cl:cons ':rotation_left (rotation_left msg))
    (cl:cons ':rotation_right (rotation_right msg))
    (cl:cons ':pivot_angle (pivot_angle msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'CommandVelocity)))
  'CommandVelocity-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'CommandVelocity)))
  'CommandVelocity-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'CommandVelocity)))
  "Returns string type for a service object of type '<CommandVelocity>"
  "joe_controller_pkg/CommandVelocity")
