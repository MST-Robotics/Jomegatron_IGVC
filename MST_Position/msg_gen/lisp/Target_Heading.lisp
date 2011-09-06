; Auto-generated. Do not edit!


(cl:in-package MST_Position-msg)


;//! \htmlinclude Target_Heading.msg.html

(cl:defclass <Target_Heading> (roslisp-msg-protocol:ros-message)
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
    :type cl:fixnum
    :initform 0)
   (stop_robot
    :reader stop_robot
    :initarg :stop_robot
    :type cl:boolean
    :initform cl:nil)
   (done
    :reader done
    :initarg :done
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Target_Heading (<Target_Heading>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Target_Heading>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Target_Heading)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name MST_Position-msg:<Target_Heading> is deprecated: use MST_Position-msg:Target_Heading instead.")))

(cl:ensure-generic-function 'target_heading-val :lambda-list '(m))
(cl:defmethod target_heading-val ((m <Target_Heading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:target_heading-val is deprecated.  Use MST_Position-msg:target_heading instead.")
  (target_heading m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <Target_Heading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:distance-val is deprecated.  Use MST_Position-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'waypoint-val :lambda-list '(m))
(cl:defmethod waypoint-val ((m <Target_Heading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:waypoint-val is deprecated.  Use MST_Position-msg:waypoint instead.")
  (waypoint m))

(cl:ensure-generic-function 'stop_robot-val :lambda-list '(m))
(cl:defmethod stop_robot-val ((m <Target_Heading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:stop_robot-val is deprecated.  Use MST_Position-msg:stop_robot instead.")
  (stop_robot m))

(cl:ensure-generic-function 'done-val :lambda-list '(m))
(cl:defmethod done-val ((m <Target_Heading>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Position-msg:done-val is deprecated.  Use MST_Position-msg:done instead.")
  (done m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Target_Heading>) ostream)
  "Serializes a message object of type '<Target_Heading>"
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
  (cl:let* ((signed (cl:slot-value msg 'waypoint)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'stop_robot) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'done) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Target_Heading>) istream)
  "Deserializes a message object of type '<Target_Heading>"
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
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'waypoint) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
    (cl:setf (cl:slot-value msg 'stop_robot) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'done) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Target_Heading>)))
  "Returns string type for a message object of type '<Target_Heading>"
  "MST_Position/Target_Heading")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Target_Heading)))
  "Returns string type for a message object of type 'Target_Heading"
  "MST_Position/Target_Heading")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Target_Heading>)))
  "Returns md5sum for a message object of type '<Target_Heading>"
  "a83a1849d285a5fb13b2a15dc5ba888d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Target_Heading)))
  "Returns md5sum for a message object of type 'Target_Heading"
  "a83a1849d285a5fb13b2a15dc5ba888d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Target_Heading>)))
  "Returns full string definition for message of type '<Target_Heading>"
  (cl:format cl:nil "float64     target_heading~%float64     distance~%int8        waypoint~%bool        stop_robot~%bool        done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Target_Heading)))
  "Returns full string definition for message of type 'Target_Heading"
  (cl:format cl:nil "float64     target_heading~%float64     distance~%int8        waypoint~%bool        stop_robot~%bool        done~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Target_Heading>))
  (cl:+ 0
     8
     8
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Target_Heading>))
  "Converts a ROS message object to a list"
  (cl:list 'Target_Heading
    (cl:cons ':target_heading (target_heading msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':waypoint (waypoint msg))
    (cl:cons ':stop_robot (stop_robot msg))
    (cl:cons ':done (done msg))
))
