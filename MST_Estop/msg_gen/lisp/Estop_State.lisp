; Auto-generated. Do not edit!


(cl:in-package MST_Estop-msg)


;//! \htmlinclude Estop_State.msg.html

(cl:defclass <Estop_State> (roslisp-msg-protocol:ros-message)
  ((state
    :reader state
    :initarg :state
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Estop_State (<Estop_State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Estop_State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Estop_State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name MST_Estop-msg:<Estop_State> is deprecated: use MST_Estop-msg:Estop_State instead.")))

(cl:ensure-generic-function 'state-val :lambda-list '(m))
(cl:defmethod state-val ((m <Estop_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Estop-msg:state-val is deprecated.  Use MST_Estop-msg:state instead.")
  (state m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Estop_State>) ostream)
  "Serializes a message object of type '<Estop_State>"
  (cl:let* ((signed (cl:slot-value msg 'state)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Estop_State>) istream)
  "Deserializes a message object of type '<Estop_State>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'state) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Estop_State>)))
  "Returns string type for a message object of type '<Estop_State>"
  "MST_Estop/Estop_State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Estop_State)))
  "Returns string type for a message object of type 'Estop_State"
  "MST_Estop/Estop_State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Estop_State>)))
  "Returns md5sum for a message object of type '<Estop_State>"
  "a33bed68685ae53bd39b0a9242210752")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Estop_State)))
  "Returns md5sum for a message object of type 'Estop_State"
  "a33bed68685ae53bd39b0a9242210752")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Estop_State>)))
  "Returns full string definition for message of type '<Estop_State>"
  (cl:format cl:nil "int8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Estop_State)))
  "Returns full string definition for message of type 'Estop_State"
  (cl:format cl:nil "int8 state~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Estop_State>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Estop_State>))
  "Converts a ROS message object to a list"
  (cl:list 'Estop_State
    (cl:cons ':state (state msg))
))
