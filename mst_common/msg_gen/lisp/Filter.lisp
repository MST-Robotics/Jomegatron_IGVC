; Auto-generated. Do not edit!


(cl:in-package mst_common-msg)


;//! \htmlinclude Filter.msg.html

(cl:defclass <Filter> (roslisp-msg-protocol:ros-message)
  ((gain
    :reader gain
    :initarg :gain
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 256 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass Filter (<Filter>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Filter>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Filter)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name mst_common-msg:<Filter> is deprecated: use mst_common-msg:Filter instead.")))

(cl:ensure-generic-function 'gain-val :lambda-list '(m))
(cl:defmethod gain-val ((m <Filter>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader mst_common-msg:gain-val is deprecated.  Use mst_common-msg:gain instead.")
  (gain m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Filter>) ostream)
  "Serializes a message object of type '<Filter>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:write-byte (cl:ldb (cl:byte 8 0) ele) ostream))
   (cl:slot-value msg 'gain))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Filter>) istream)
  "Deserializes a message object of type '<Filter>"
  (cl:setf (cl:slot-value msg 'gain) (cl:make-array 256))
  (cl:let ((vals (cl:slot-value msg 'gain)))
    (cl:dotimes (i 256)
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:aref vals i)) (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Filter>)))
  "Returns string type for a message object of type '<Filter>"
  "mst_common/Filter")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Filter)))
  "Returns string type for a message object of type 'Filter"
  "mst_common/Filter")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Filter>)))
  "Returns md5sum for a message object of type '<Filter>"
  "2b3708e5d288c1626786105d0a37f96b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Filter)))
  "Returns md5sum for a message object of type 'Filter"
  "2b3708e5d288c1626786105d0a37f96b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Filter>)))
  "Returns full string definition for message of type '<Filter>"
  (cl:format cl:nil "uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Filter)))
  "Returns full string definition for message of type 'Filter"
  (cl:format cl:nil "uint8[256]  gain~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Filter>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'gain) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Filter>))
  "Converts a ROS message object to a list"
  (cl:list 'Filter
    (cl:cons ':gain (gain msg))
))
