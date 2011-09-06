; Auto-generated. Do not edit!


(cl:in-package mst_common-msg)


;//! \htmlinclude Raytrace.msg.html

(cl:defclass <Raytrace> (roslisp-msg-protocol:ros-message)
  ((ranges
    :reader ranges
    :initarg :ranges
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (num_ranges
    :reader num_ranges
    :initarg :num_ranges
    :type cl:integer
    :initform 0))
)

(cl:defclass Raytrace (<Raytrace>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Raytrace>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Raytrace)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mst_common-msg:<Raytrace> is deprecated: use mst_common-msg:Raytrace instead.")))

(cl:ensure-generic-function 'ranges-val :lambda-list '(m))
(cl:defmethod ranges-val ((m <Raytrace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:ranges-val is deprecated.  Use mst_common-msg:ranges instead.")
  (ranges m))

(cl:ensure-generic-function 'num_ranges-val :lambda-list '(m))
(cl:defmethod num_ranges-val ((m <Raytrace>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:num_ranges-val is deprecated.  Use mst_common-msg:num_ranges instead.")
  (num_ranges m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Raytrace>) ostream)
  "Serializes a message object of type '<Raytrace>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'ranges))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'ranges))
  (cl:let* ((signed (cl:slot-value msg 'num_ranges)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Raytrace>) istream)
  "Deserializes a message object of type '<Raytrace>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'ranges) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'ranges)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'num_ranges) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Raytrace>)))
  "Returns string type for a message object of type '<Raytrace>"
  "mst_common/Raytrace")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Raytrace)))
  "Returns string type for a message object of type 'Raytrace"
  "mst_common/Raytrace")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Raytrace>)))
  "Returns md5sum for a message object of type '<Raytrace>"
  "77ea022a1e6008d0edfcc5431496c9ff")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Raytrace)))
  "Returns md5sum for a message object of type 'Raytrace"
  "77ea022a1e6008d0edfcc5431496c9ff")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Raytrace>)))
  "Returns full string definition for message of type '<Raytrace>"
  (cl:format cl:nil "float64[]   ranges~%int32       num_ranges~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Raytrace)))
  "Returns full string definition for message of type 'Raytrace"
  (cl:format cl:nil "float64[]   ranges~%int32       num_ranges~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Raytrace>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'ranges) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Raytrace>))
  "Converts a ROS message object to a list"
  (cl:list 'Raytrace
    (cl:cons ':ranges (ranges msg))
    (cl:cons ':num_ranges (num_ranges msg))
))
