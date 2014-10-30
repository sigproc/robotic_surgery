; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude JointPositionAndState.msg.html

(cl:defclass <JointPositionAndState> (roslisp-msg-protocol:ros-message)
  ((trackingState
    :reader trackingState
    :initarg :trackingState
    :type cl:boolean
    :initform cl:nil)
   (position
    :reader position
    :initarg :position
    :type geometry_msgs-msg:Point
    :initform (cl:make-instance 'geometry_msgs-msg:Point))
   (jointType
    :reader jointType
    :initarg :jointType
    :type cl:integer
    :initform 0))
)

(cl:defclass JointPositionAndState (<JointPositionAndState>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <JointPositionAndState>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'JointPositionAndState)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<JointPositionAndState> is deprecated: use k2_client-msg:JointPositionAndState instead.")))

(cl:ensure-generic-function 'trackingState-val :lambda-list '(m))
(cl:defmethod trackingState-val ((m <JointPositionAndState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:trackingState-val is deprecated.  Use k2_client-msg:trackingState instead.")
  (trackingState m))

(cl:ensure-generic-function 'position-val :lambda-list '(m))
(cl:defmethod position-val ((m <JointPositionAndState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:position-val is deprecated.  Use k2_client-msg:position instead.")
  (position m))

(cl:ensure-generic-function 'jointType-val :lambda-list '(m))
(cl:defmethod jointType-val ((m <JointPositionAndState>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:jointType-val is deprecated.  Use k2_client-msg:jointType instead.")
  (jointType m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <JointPositionAndState>) ostream)
  "Serializes a message object of type '<JointPositionAndState>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'trackingState) 1 0)) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'position) ostream)
  (cl:let* ((signed (cl:slot-value msg 'jointType)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <JointPositionAndState>) istream)
  "Deserializes a message object of type '<JointPositionAndState>"
    (cl:setf (cl:slot-value msg 'trackingState) (cl:not (cl:zerop (cl:read-byte istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'position) istream)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'jointType) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<JointPositionAndState>)))
  "Returns string type for a message object of type '<JointPositionAndState>"
  "k2_client/JointPositionAndState")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'JointPositionAndState)))
  "Returns string type for a message object of type 'JointPositionAndState"
  "k2_client/JointPositionAndState")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<JointPositionAndState>)))
  "Returns md5sum for a message object of type '<JointPositionAndState>"
  "fffd6acd4cf38509c6d9ae24639d6b49")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'JointPositionAndState)))
  "Returns md5sum for a message object of type 'JointPositionAndState"
  "fffd6acd4cf38509c6d9ae24639d6b49")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<JointPositionAndState>)))
  "Returns full string definition for message of type '<JointPositionAndState>"
  (cl:format cl:nil "bool trackingState~%geometry_msgs/Point position~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'JointPositionAndState)))
  "Returns full string definition for message of type 'JointPositionAndState"
  (cl:format cl:nil "bool trackingState~%geometry_msgs/Point position~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <JointPositionAndState>))
  (cl:+ 0
     1
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'position))
     4
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <JointPositionAndState>))
  "Converts a ROS message object to a list"
  (cl:list 'JointPositionAndState
    (cl:cons ':trackingState (trackingState msg))
    (cl:cons ':position (position msg))
    (cl:cons ':jointType (jointType msg))
))
