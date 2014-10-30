; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude Appearance.msg.html

(cl:defclass <Appearance> (roslisp-msg-protocol:ros-message)
  ((wearingGlasses
    :reader wearingGlasses
    :initarg :wearingGlasses
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Appearance (<Appearance>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Appearance>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Appearance)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<Appearance> is deprecated: use k2_client-msg:Appearance instead.")))

(cl:ensure-generic-function 'wearingGlasses-val :lambda-list '(m))
(cl:defmethod wearingGlasses-val ((m <Appearance>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:wearingGlasses-val is deprecated.  Use k2_client-msg:wearingGlasses instead.")
  (wearingGlasses m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Appearance>) ostream)
  "Serializes a message object of type '<Appearance>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wearingGlasses) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Appearance>) istream)
  "Deserializes a message object of type '<Appearance>"
    (cl:setf (cl:slot-value msg 'wearingGlasses) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Appearance>)))
  "Returns string type for a message object of type '<Appearance>"
  "k2_client/Appearance")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Appearance)))
  "Returns string type for a message object of type 'Appearance"
  "k2_client/Appearance")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Appearance>)))
  "Returns md5sum for a message object of type '<Appearance>"
  "69bdacaca29a46fe02e2540c6fec5a99")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Appearance)))
  "Returns md5sum for a message object of type 'Appearance"
  "69bdacaca29a46fe02e2540c6fec5a99")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Appearance>)))
  "Returns full string definition for message of type '<Appearance>"
  (cl:format cl:nil "bool wearingGlasses~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Appearance)))
  "Returns full string definition for message of type 'Appearance"
  (cl:format cl:nil "bool wearingGlasses~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Appearance>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Appearance>))
  "Converts a ROS message object to a list"
  (cl:list 'Appearance
    (cl:cons ':wearingGlasses (wearingGlasses msg))
))
