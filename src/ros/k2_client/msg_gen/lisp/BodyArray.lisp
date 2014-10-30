; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude BodyArray.msg.html

(cl:defclass <BodyArray> (roslisp-msg-protocol:ros-message)
  ((bodies
    :reader bodies
    :initarg :bodies
    :type (cl:vector k2_client-msg:Body)
   :initform (cl:make-array 0 :element-type 'k2_client-msg:Body :initial-element (cl:make-instance 'k2_client-msg:Body))))
)

(cl:defclass BodyArray (<BodyArray>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <BodyArray>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'BodyArray)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<BodyArray> is deprecated: use k2_client-msg:BodyArray instead.")))

(cl:ensure-generic-function 'bodies-val :lambda-list '(m))
(cl:defmethod bodies-val ((m <BodyArray>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:bodies-val is deprecated.  Use k2_client-msg:bodies instead.")
  (bodies m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <BodyArray>) ostream)
  "Serializes a message object of type '<BodyArray>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'bodies))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'bodies))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <BodyArray>) istream)
  "Deserializes a message object of type '<BodyArray>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'bodies) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'bodies)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'k2_client-msg:Body))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<BodyArray>)))
  "Returns string type for a message object of type '<BodyArray>"
  "k2_client/BodyArray")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'BodyArray)))
  "Returns string type for a message object of type 'BodyArray"
  "k2_client/BodyArray")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<BodyArray>)))
  "Returns md5sum for a message object of type '<BodyArray>"
  "eb5202664bdf63a70294b15966c92b44")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'BodyArray)))
  "Returns md5sum for a message object of type 'BodyArray"
  "eb5202664bdf63a70294b15966c92b44")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<BodyArray>)))
  "Returns full string definition for message of type '<BodyArray>"
  (cl:format cl:nil "Body[] bodies~%================================================================================~%MSG: k2_client/Body~%Header header~%int32 leanTrackingState~%Lean lean~%bool isRestricted~%bool isTracked~%uint64 trackingId~%int32 clippedEdges~%bool engaged~%float32 handRightConfidence~%int32 handRightState~%float32 handLeftConfidence~%int32 handLeftState~%Appearance appearance~%Activities activities~%Expressions expressions~%JointOrientationAndType[] jointOrientations~%JointPositionAndState[] jointPositions~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: k2_client/Lean~%float64 leanX~%float64 leanY~%================================================================================~%MSG: k2_client/Appearance~%bool wearingGlasses~%================================================================================~%MSG: k2_client/Activities~%bool eyeLeftClosed~%bool eyeRightClosed~%bool mouthOpen~%bool mouthMoved~%bool lookingAway~%================================================================================~%MSG: k2_client/Expressions~%bool neutral~%bool happy~%================================================================================~%MSG: k2_client/JointOrientationAndType~%geometry_msgs/Quaternion orientation~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: k2_client/JointPositionAndState~%bool trackingState~%geometry_msgs/Point position~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'BodyArray)))
  "Returns full string definition for message of type 'BodyArray"
  (cl:format cl:nil "Body[] bodies~%================================================================================~%MSG: k2_client/Body~%Header header~%int32 leanTrackingState~%Lean lean~%bool isRestricted~%bool isTracked~%uint64 trackingId~%int32 clippedEdges~%bool engaged~%float32 handRightConfidence~%int32 handRightState~%float32 handLeftConfidence~%int32 handLeftState~%Appearance appearance~%Activities activities~%Expressions expressions~%JointOrientationAndType[] jointOrientations~%JointPositionAndState[] jointPositions~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: k2_client/Lean~%float64 leanX~%float64 leanY~%================================================================================~%MSG: k2_client/Appearance~%bool wearingGlasses~%================================================================================~%MSG: k2_client/Activities~%bool eyeLeftClosed~%bool eyeRightClosed~%bool mouthOpen~%bool mouthMoved~%bool lookingAway~%================================================================================~%MSG: k2_client/Expressions~%bool neutral~%bool happy~%================================================================================~%MSG: k2_client/JointOrientationAndType~%geometry_msgs/Quaternion orientation~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: k2_client/JointPositionAndState~%bool trackingState~%geometry_msgs/Point position~%int32 jointType~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <BodyArray>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'bodies) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <BodyArray>))
  "Converts a ROS message object to a list"
  (cl:list 'BodyArray
    (cl:cons ':bodies (bodies msg))
))
