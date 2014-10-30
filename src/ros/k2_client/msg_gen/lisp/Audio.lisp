; Auto-generated. Do not edit!


(cl:in-package k2_client-msg)


;//! \htmlinclude Audio.msg.html

(cl:defclass <Audio> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (beamAngle
    :reader beamAngle
    :initarg :beamAngle
    :type cl:float
    :initform 0.0)
   (beamAngleConfidence
    :reader beamAngleConfidence
    :initarg :beamAngleConfidence
    :type cl:float
    :initform 0.0)
   (audioStream
    :reader audioStream
    :initarg :audioStream
    :type (cl:vector cl:float)
   :initform (cl:make-array 0 :element-type 'cl:float :initial-element 0.0))
   (numBytesPerSample
    :reader numBytesPerSample
    :initarg :numBytesPerSample
    :type cl:fixnum
    :initform 0)
   (numSamplesPerFrame
    :reader numSamplesPerFrame
    :initarg :numSamplesPerFrame
    :type cl:fixnum
    :initform 0)
   (frameLifeTime
    :reader frameLifeTime
    :initarg :frameLifeTime
    :type cl:float
    :initform 0.0)
   (samplingFrequency
    :reader samplingFrequency
    :initarg :samplingFrequency
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Audio (<Audio>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Audio>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Audio)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name k2_client-msg:<Audio> is deprecated: use k2_client-msg:Audio instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:header-val is deprecated.  Use k2_client-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'beamAngle-val :lambda-list '(m))
(cl:defmethod beamAngle-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:beamAngle-val is deprecated.  Use k2_client-msg:beamAngle instead.")
  (beamAngle m))

(cl:ensure-generic-function 'beamAngleConfidence-val :lambda-list '(m))
(cl:defmethod beamAngleConfidence-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:beamAngleConfidence-val is deprecated.  Use k2_client-msg:beamAngleConfidence instead.")
  (beamAngleConfidence m))

(cl:ensure-generic-function 'audioStream-val :lambda-list '(m))
(cl:defmethod audioStream-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:audioStream-val is deprecated.  Use k2_client-msg:audioStream instead.")
  (audioStream m))

(cl:ensure-generic-function 'numBytesPerSample-val :lambda-list '(m))
(cl:defmethod numBytesPerSample-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:numBytesPerSample-val is deprecated.  Use k2_client-msg:numBytesPerSample instead.")
  (numBytesPerSample m))

(cl:ensure-generic-function 'numSamplesPerFrame-val :lambda-list '(m))
(cl:defmethod numSamplesPerFrame-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:numSamplesPerFrame-val is deprecated.  Use k2_client-msg:numSamplesPerFrame instead.")
  (numSamplesPerFrame m))

(cl:ensure-generic-function 'frameLifeTime-val :lambda-list '(m))
(cl:defmethod frameLifeTime-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:frameLifeTime-val is deprecated.  Use k2_client-msg:frameLifeTime instead.")
  (frameLifeTime m))

(cl:ensure-generic-function 'samplingFrequency-val :lambda-list '(m))
(cl:defmethod samplingFrequency-val ((m <Audio>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader k2_client-msg:samplingFrequency-val is deprecated.  Use k2_client-msg:samplingFrequency instead.")
  (samplingFrequency m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Audio>) ostream)
  "Serializes a message object of type '<Audio>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'beamAngle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'beamAngleConfidence))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'audioStream))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-single-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)))
   (cl:slot-value msg 'audioStream))
  (cl:let* ((signed (cl:slot-value msg 'numBytesPerSample)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let* ((signed (cl:slot-value msg 'numSamplesPerFrame)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'frameLifeTime))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'samplingFrequency)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 65536) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Audio>) istream)
  "Deserializes a message object of type '<Audio>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'beamAngle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'beamAngleConfidence) (roslisp-utils:decode-double-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'audioStream) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'audioStream)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-single-float-bits bits))))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numBytesPerSample) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'numSamplesPerFrame) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'frameLifeTime) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'samplingFrequency) (cl:if (cl:< unsigned 32768) unsigned (cl:- unsigned 65536))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Audio>)))
  "Returns string type for a message object of type '<Audio>"
  "k2_client/Audio")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Audio)))
  "Returns string type for a message object of type 'Audio"
  "k2_client/Audio")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Audio>)))
  "Returns md5sum for a message object of type '<Audio>"
  "37cc5db6e0123e6d300ef4b2f9d18939")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Audio)))
  "Returns md5sum for a message object of type 'Audio"
  "37cc5db6e0123e6d300ef4b2f9d18939")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Audio>)))
  "Returns full string definition for message of type '<Audio>"
  (cl:format cl:nil "Header header~%float64 beamAngle~%float64 beamAngleConfidence~%float32[] audioStream~%int16 numBytesPerSample~%int16 numSamplesPerFrame~%float64 frameLifeTime~%int16 samplingFrequency~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Audio)))
  "Returns full string definition for message of type 'Audio"
  (cl:format cl:nil "Header header~%float64 beamAngle~%float64 beamAngleConfidence~%float32[] audioStream~%int16 numBytesPerSample~%int16 numSamplesPerFrame~%float64 frameLifeTime~%int16 samplingFrequency~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Audio>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     8
     8
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'audioStream) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 4)))
     2
     2
     8
     2
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Audio>))
  "Converts a ROS message object to a list"
  (cl:list 'Audio
    (cl:cons ':header (header msg))
    (cl:cons ':beamAngle (beamAngle msg))
    (cl:cons ':beamAngleConfidence (beamAngleConfidence msg))
    (cl:cons ':audioStream (audioStream msg))
    (cl:cons ':numBytesPerSample (numBytesPerSample msg))
    (cl:cons ':numSamplesPerFrame (numSamplesPerFrame msg))
    (cl:cons ':frameLifeTime (frameLifeTime msg))
    (cl:cons ':samplingFrequency (samplingFrequency msg))
))
