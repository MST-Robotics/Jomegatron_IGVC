; Auto-generated. Do not edit!


(cl:in-package mst_common-msg)


;//! \htmlinclude VisionFilter.msg.html

(cl:defclass <VisionFilter> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (color
    :reader color
    :initarg :color
    :type (cl:vector mst_common-msg:ImageFilter)
   :initform (cl:make-array 0 :element-type 'mst_common-msg:ImageFilter :initial-element (cl:make-instance 'mst_common-msg:ImageFilter))))
)

(cl:defclass VisionFilter (<VisionFilter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisionFilter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisionFilter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mst_common-msg:<VisionFilter> is deprecated: use mst_common-msg:VisionFilter instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <VisionFilter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:header-val is deprecated.  Use mst_common-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <VisionFilter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:color-val is deprecated.  Use mst_common-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisionFilter>) ostream)
  "Serializes a message object of type '<VisionFilter>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'color))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'color))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisionFilter>) istream)
  "Deserializes a message object of type '<VisionFilter>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'color) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'color)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mst_common-msg:ImageFilter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisionFilter>)))
  "Returns string type for a message object of type '<VisionFilter>"
  "mst_common/VisionFilter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisionFilter)))
  "Returns string type for a message object of type 'VisionFilter"
  "mst_common/VisionFilter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisionFilter>)))
  "Returns md5sum for a message object of type '<VisionFilter>"
  "444ae704995b1a5f6fd91cb7494a14b0")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisionFilter)))
  "Returns md5sum for a message object of type 'VisionFilter"
  "444ae704995b1a5f6fd91cb7494a14b0")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisionFilter>)))
  "Returns full string definition for message of type '<VisionFilter>"
  (cl:format cl:nil "Header          header~%ImageFilter[]   color~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mst_common/ImageFilter~%string      type~%Filter[3]   filter~%~%================================================================================~%MSG: mst_common/Filter~%uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisionFilter)))
  "Returns full string definition for message of type 'VisionFilter"
  (cl:format cl:nil "Header          header~%ImageFilter[]   color~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: mst_common/ImageFilter~%string      type~%Filter[3]   filter~%~%================================================================================~%MSG: mst_common/Filter~%uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisionFilter>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'color) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisionFilter>))
  "Converts a ROS message object to a list"
  (cl:list 'VisionFilter
    (cl:cons ':header (header msg))
    (cl:cons ':color (color msg))
))
