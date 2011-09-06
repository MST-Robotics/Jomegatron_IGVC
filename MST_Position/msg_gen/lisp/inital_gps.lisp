; Auto-generated. Do not edit!


(cl:in-package MST_Position-msg)


;//! \htmlinclude inital_gps.msg.html

(cl:defclass <inital_gps> (roslisp-msg-protocol:ros-message)
  ((target_heading
    :reader target_heading
    :initarg :target_heading
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (waypoint
    :reader waypoint
    :initarg :waypoint
    :type cl:float
    :initform 0.0))
)

(cl:defclass inital_gps (<inital_gps>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <inital_gps>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'inital_gps)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name MST_Position-msg:<inital_gps> is deprecated: use MST_Position-msg:inital_gps instead.")))

(cl:ensure-generic-function 'target_heading-val :lambda-list '(m))
(cl:defmethod target_heading-val ((m <inital_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:target_heading-val is deprecated.  Use MST_Position-msg:target_heading instead.")
  (target_heading m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <inital_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:distance-val is deprecated.  Use MST_Position-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <inital_gps>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:waypoint-val is deprecated.  Use MST_Position-msg:waypoint instead.")
  (waypoint m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <inital_gps>) ostream)
  "Serializes a message object of type '<inital_gps>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'target_heading))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'waypoint))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <inital_gps>) istream)
  "Deserializes a message object of type '<inital_gps>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'target_heading) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'waypoint) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<inital_gps>)))
  "Returns string type for a message object of type '<inital_gps>"
  "MST_Position/inital_gps")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'inital_gps)))
  "Returns string type for a message object of type 'inital_gps"
  "MST_Position/inital_gps")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<inital_gps>)))
  "Returns md5sum for a message object of type '<inital_gps>"
  "ba0fbfca81c6cac39788bbb65f584f05")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'inital_gps)))
  "Returns md5sum for a message object of type 'inital_gps"
  "ba0fbfca81c6cac39788bbb65f584f05")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<inital_gps>)))
  "Returns full string definition for message of type '<inital_gps>"
  (cl:format cl:nil "float64     target_heading~%float64     distance~%float64     waypoint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'inital_gps)))
  "Returns full string definition for message of type 'inital_gps"
  (cl:format cl:nil "float64     target_heading~%float64     distance~%float64     waypoint~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <inital_gps>))
  (cl:+ 0
     8
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <inital_gps>))
  "Converts a ROS message object to a list"
  (cl:list 'inital_gps
    (cl:cons ':target_heading (target_heading msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':waypoint (waypoint msg))
))
