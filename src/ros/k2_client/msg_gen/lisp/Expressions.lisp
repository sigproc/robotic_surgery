; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude Expressions.msg.html

(cl:defclass <Expressions> (roslisp-msg-protocol:ros-message)
  ((neutral
    :reader neutral
    :initarg :neutral
    :type cl:boolean
    :initform cl:nil)
   (happy
    :reader happy
    :initarg :happy
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Expressions (<Expressions>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Expressions>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Expressions)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<Expressions> is deprecated: use k2_client-msg:Expressions instead.")))

(cl:ensure-generic-function 'neutral-val :lambda-list '(m))
(cl:defmethod neutral-val ((m <Expressions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:neutral-val is deprecated.  Use k2_client-msg:neutral instead.")
  (neutral m))

(cl:ensure-generic-function 'happy-val :lambda-list '(m))
(cl:defmethod happy-val ((m <Expressions>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:happy-val is deprecated.  Use k2_client-msg:happy instead.")
  (happy m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Expressions>) ostream)
  "Serializes a message object of type '<Expressions>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'neutral) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'happy) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Expressions>) istream)
  "Deserializes a message object of type '<Expressions>"
    (cl:setf (cl:slot-value msg 'neutral) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'happy) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Expressions>)))
  "Returns string type for a message object of type '<Expressions>"
  "k2_client/Expressions")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Expressions)))
  "Returns string type for a message object of type 'Expressions"
  "k2_client/Expressions")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Expressions>)))
  "Returns md5sum for a message object of type '<Expressions>"
  "5cb5b766a73d02643dd57183072cb85d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Expressions)))
  "Returns md5sum for a message object of type 'Expressions"
  "5cb5b766a73d02643dd57183072cb85d")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Expressions>)))
  "Returns full string definition for message of type '<Expressions>"
  (cl:format cl:nil "bool neutral~%bool happy~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Expressions)))
  "Returns full string definition for message of type 'Expressions"
  (cl:format cl:nil "bool neutral~%bool happy~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Expressions>))
  (cl:+ 0
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Expressions>))
  "Converts a ROS message object to a list"
  (cl:list 'Expressions
    (cl:cons ':neutral (neutral msg))
    (cl:cons ':happy (happy msg))
))
