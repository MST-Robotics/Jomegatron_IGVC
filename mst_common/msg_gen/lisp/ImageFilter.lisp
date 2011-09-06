; Auto-generated. Do not edit!


(cl:in-package mst_common-msg)


;//! \htmlinclude ImageFilter.msg.html

(cl:defclass <ImageFilter> (roslisp-msg-protocol:ros-message)
  ((type
    :reader type
    :initarg :type
    :type cl:string
    :initform "")
   (filter
    :reader filter
    :initarg :filter
    :type (cl:vector mst_common-msg:Filter)
   :initform (cl:make-array 3 :element-type 'mst_common-msg:Filter :initial-element (cl:make-instance 'mst_common-msg:Filter))))
)

(cl:defclass ImageFilter (<ImageFilter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ImageFilter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ImageFilter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mst_common-msg:<ImageFilter> is deprecated: use mst_common-msg:ImageFilter instead.")))

(cl:ensure-generic-function 'type-val :lambda-list '(m))
(cl:defmethod type-val ((m <ImageFilter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:type-val is deprecated.  Use mst_common-msg:type instead.")
  (type m))

(cl:ensure-generic-function 'filter-val :lambda-list '(m))
(cl:defmethod filter-val ((m <ImageFilter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:filter-val is deprecated.  Use mst_common-msg:filter instead.")
  (filter m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ImageFilter>) ostream)
  "Serializes a message object of type '<ImageFilter>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'type))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'filter))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ImageFilter>) istream)
  "Deserializes a message object of type '<ImageFilter>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  (cl:setf (cl:slot-value msg 'filter) (cl:make-array 3))
  (cl:let ((vals (cl:slot-value msg 'filter)))
    (cl:dotimes (i 3)
    (cl:setf (cl:aref vals i) (cl:make-instance 'mst_common-msg:Filter))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ImageFilter>)))
  "Returns string type for a message object of type '<ImageFilter>"
  "mst_common/ImageFilter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ImageFilter)))
  "Returns string type for a message object of type 'ImageFilter"
  "mst_common/ImageFilter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ImageFilter>)))
  "Returns md5sum for a message object of type '<ImageFilter>"
  "fa622389d14d78c43b4f70337cbd1806")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ImageFilter)))
  "Returns md5sum for a message object of type 'ImageFilter"
  "fa622389d14d78c43b4f70337cbd1806")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ImageFilter>)))
  "Returns full string definition for message of type '<ImageFilter>"
  (cl:format cl:nil "string      type~%Filter[3]   filter~%~%================================================================================~%MSG: mst_common/Filter~%uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ImageFilter)))
  "Returns full string definition for message of type 'ImageFilter"
  (cl:format cl:nil "string      type~%Filter[3]   filter~%~%================================================================================~%MSG: mst_common/Filter~%uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ImageFilter>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'type))
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'filter) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ImageFilter>))
  "Converts a ROS message object to a list"
  (cl:list 'ImageFilter
    (cl:cons ':type (type msg))
    (cl:cons ':filter (filter msg))
))
