; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude Lean.msg.html

(cl:defclass <Lean> (roslisp-msg-protocol:ros-message)
  ((leanX
    :reader leanX
    :initarg :leanX
    :type cl:float
    :initform 0.0)
   (leanY
    :reader leanY
    :initarg :leanY
    :type cl:float
    :initform 0.0))
)

(cl:defclass Lean (<Lean>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Lean>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Lean)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<Lean> is deprecated: use k2_client-msg:Lean instead.")))

(cl:ensure-generic-function 'leanX-val :lambda-list '(m))
(cl:defmethod leanX-val ((m <Lean>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:leanX-val is deprecated.  Use k2_client-msg:leanX instead.")
  (leanX m))

(cl:ensure-generic-function 'leanY-val :lambda-list '(m))
(cl:defmethod leanY-val ((m <Lean>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:leanY-val is deprecated.  Use k2_client-msg:leanY instead.")
  (leanY m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Lean>) ostream)
  "Serializes a message object of type '<Lean>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'leanX))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'leanY))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Lean>) istream)
  "Deserializes a message object of type '<Lean>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leanX) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'leanY) (roslisp-utils:decode-double-float-bits bits)))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Lean>)))
  "Returns string type for a message object of type '<Lean>"
  "k2_client/Lean")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Lean)))
  "Returns string type for a message object of type 'Lean"
  "k2_client/Lean")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Lean>)))
  "Returns md5sum for a message object of type '<Lean>"
  "c40366f6936def86f552f4754f13c41b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Lean)))
  "Returns md5sum for a message object of type 'Lean"
  "c40366f6936def86f552f4754f13c41b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Lean>)))
  "Returns full string definition for message of type '<Lean>"
  (cl:format cl:nil "float64 leanX~%float64 leanY~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Lean)))
  "Returns full string definition for message of type 'Lean"
  (cl:format cl:nil "float64 leanX~%float64 leanY~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Lean>))
  (cl:+ 0
     8
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Lean>))
  "Converts a ROS message object to a list"
  (cl:list 'Lean
    (cl:cons ':leanX (leanX msg))
    (cl:cons ':leanY (leanY msg))
))
