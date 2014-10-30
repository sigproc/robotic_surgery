; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude Activities.msg.html

(cl:defclass <Activities> (roslisp-msg-protocol:ros-message)
  ((eyeLeftClosed
    :reader eyeLeftClosed
    :initarg :eyeLeftClosed
    :type cl:boolean
    :initform cl:nil)
   (eyeRightClosed
    :reader eyeRightClosed
    :initarg :eyeRightClosed
    :type cl:boolean
    :initform cl:nil)
   (mouthOpen
    :reader mouthOpen
    :initarg :mouthOpen
    :type cl:boolean
    :initform cl:nil)
   (mouthMoved
    :reader mouthMoved
    :initarg :mouthMoved
    :type cl:boolean
    :initform cl:nil)
   (lookingAway
    :reader lookingAway
    :initarg :lookingAway
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass Activities (<Activities>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Activities>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Activities)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<Activities> is deprecated: use k2_client-msg:Activities instead.")))

(cl:ensure-generic-function 'eyeLeftClosed-val :lambda-list '(m))
(cl:defmethod eyeLeftClosed-val ((m <Activities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:eyeLeftClosed-val is deprecated.  Use k2_client-msg:eyeLeftClosed instead.")
  (eyeLeftClosed m))

(cl:ensure-generic-function 'eyeRightClosed-val :lambda-list '(m))
(cl:defmethod eyeRightClosed-val ((m <Activities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:eyeRightClosed-val is deprecated.  Use k2_client-msg:eyeRightClosed instead.")
  (eyeRightClosed m))

(cl:ensure-generic-function 'mouthOpen-val :lambda-list '(m))
(cl:defmethod mouthOpen-val ((m <Activities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:mouthOpen-val is deprecated.  Use k2_client-msg:mouthOpen instead.")
  (mouthOpen m))

(cl:ensure-generic-function 'mouthMoved-val :lambda-list '(m))
(cl:defmethod mouthMoved-val ((m <Activities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:mouthMoved-val is deprecated.  Use k2_client-msg:mouthMoved instead.")
  (mouthMoved m))

(cl:ensure-generic-function 'lookingAway-val :lambda-list '(m))
(cl:defmethod lookingAway-val ((m <Activities>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:lookingAway-val is deprecated.  Use k2_client-msg:lookingAway instead.")
  (lookingAway m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Activities>) ostream)
  "Serializes a message object of type '<Activities>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'eyeLeftClosed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'eyeRightClosed) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mouthOpen) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'mouthMoved) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'lookingAway) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Activities>) istream)
  "Deserializes a message object of type '<Activities>"
    (cl:setf (cl:slot-value msg 'eyeLeftClosed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'eyeRightClosed) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mouthOpen) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'mouthMoved) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'lookingAway) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Activities>)))
  "Returns string type for a message object of type '<Activities>"
  "k2_client/Activities")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Activities)))
  "Returns string type for a message object of type 'Activities"
  "k2_client/Activities")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Activities>)))
  "Returns md5sum for a message object of type '<Activities>"
  "92fd995748ea10952cb2ee4d39fc8c66")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Activities)))
  "Returns md5sum for a message object of type 'Activities"
  "92fd995748ea10952cb2ee4d39fc8c66")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Activities>)))
  "Returns full string definition for message of type '<Activities>"
  (cl:format cl:nil "bool eyeLeftClosed~%bool eyeRightClosed~%bool mouthOpen~%bool mouthMoved~%bool lookingAway~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Activities)))
  "Returns full string definition for message of type 'Activities"
  (cl:format cl:nil "bool eyeLeftClosed~%bool eyeRightClosed~%bool mouthOpen~%bool mouthMoved~%bool lookingAway~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Activities>))
  (cl:+ 0
     1
     1
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Activities>))
  "Converts a ROS message object to a list"
  (cl:list 'Activities
    (cl:cons ':eyeLeftClosed (eyeLeftClosed msg))
    (cl:cons ':eyeRightClosed (eyeRightClosed msg))
    (cl:cons ':mouthOpen (mouthOpen msg))
    (cl:cons ':mouthMoved (mouthMoved msg))
    (cl:cons ':lookingAway (lookingAway msg))
))
