; Auto-generated. Do not edit!


(cl:in-package MST_Estop-msg)


;//! \htmlinclude Control_State.msg.html

(cl:defclass <Control_State> (roslisp-msg-protocol:ros-message)
  ((mode
    :reader mode
    :initarg :mode
    :type cl:string
    :initform ""))
)

(cl:defclass Control_State (<Control_State>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Control_State>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Control_State)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name MST_Estop-msg:<Control_State> is deprecated: use MST_Estop-msg:Control_State instead.")))

(cl:ensure-generic-function 'mode-val :lambda-list '(m))
(cl:defmethod mode-val ((m <Control_State>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader MST_Estop-msg:mode-val is deprecated.  Use MST_Estop-msg:mode instead.")
  (mode m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Control_State>) ostream)
  "Serializes a message object of type '<Control_State>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'mode))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'mode))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Control_State>) istream)
  "Deserializes a message object of type '<Control_State>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'mode) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'mode) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Control_State>)))
  "Returns string type for a message object of type '<Control_State>"
  "MST_Estop/Control_State")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Control_State)))
  "Returns string type for a message object of type 'Control_State"
  "MST_Estop/Control_State")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Control_State>)))
  "Returns md5sum for a message object of type '<Control_State>"
  "e84dc3ad5dc323bb64f0aca01c2d1eef")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Control_State)))
  "Returns md5sum for a message object of type 'Control_State"
  "e84dc3ad5dc323bb64f0aca01c2d1eef")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Control_State>)))
  "Returns full string definition for message of type '<Control_State>"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Control_State)))
  "Returns full string definition for message of type 'Control_State"
  (cl:format cl:nil "string mode~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Control_State>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'mode))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Control_State>))
  "Converts a ROS message object to a list"
  (cl:list 'Control_State
    (cl:cons ':mode (mode msg))
))
