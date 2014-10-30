; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude JointOrientationAndType.msg.html

(cl:defclass <JointOrientationAndType> (roslisp-msg-protocol:ros-message)
  ((orientation
    :reader orientation
    :initarg :orientation
    :type geometry_msgs-msg:Quaternion
    :initform (cl:make-instance 'geometry_msgs-msg:Quaternion))
   (jointType
    :reader jointType
    :initarg :jointType
    :type cl:integer
    :initform 0))
)

(cl:defclass JointOrientationAndType (<JointOrientationAndType>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointOrientationAndType>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointOrientationAndType)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<JointOrientationAndType> is deprecated: use k2_client-msg:JointOrientationAndType instead.")))

(cl:ensure-generic-function 'orientation-val :lambda-list '(m))
(cl:defmethod orientation-val ((m <JointOrientationAndType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:orientation-val is deprecated.  Use k2_client-msg:orientation instead.")
  (orientation m))

(cl:ensure-generic-function 'jointType-val :lambda-list '(m))
(cl:defmethod jointType-val ((m <JointOrientationAndType>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:jointType-val is deprecated.  Use k2_client-msg:jointType instead.")
  (jointType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointOrientationAndType>) ostream)
  "Serializes a message object of type '<JointOrientationAndType>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'orientation) ostream)
  (cl:let* ((signed (cl:slot-value msg 'jointType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointOrientationAndType>) istream)
  "Deserializes a message object of type '<JointOrientationAndType>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'orientation) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jointType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointOrientationAndType>)))
  "Returns string type for a message object of type '<JointOrientationAndType>"
  "k2_client/JointOrientationAndType")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointOrientationAndType)))
  "Returns string type for a message object of type 'JointOrientationAndType"
  "k2_client/JointOrientationAndType")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointOrientationAndType>)))
  "Returns md5sum for a message object of type '<JointOrientationAndType>"
  "7cc8c53731606896e746bc308db6a603")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointOrientationAndType)))
  "Returns md5sum for a message object of type 'JointOrientationAndType"
  "7cc8c53731606896e746bc308db6a603")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointOrientationAndType>)))
  "Returns full string definition for message of type '<JointOrientationAndType>"
  (cl:format cl:nil "geometry_msgs/Quaternion orientation~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointOrientationAndType)))
  "Returns full string definition for message of type 'JointOrientationAndType"
  (cl:format cl:nil "geometry_msgs/Quaternion orientation~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointOrientationAndType>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'orientation))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointOrientationAndType>))
  "Converts a ROS message object to a list"
  (cl:list 'JointOrientationAndType
    (cl:cons ':orientation (orientation msg))
    (cl:cons ':jointType (jointType msg))
))
