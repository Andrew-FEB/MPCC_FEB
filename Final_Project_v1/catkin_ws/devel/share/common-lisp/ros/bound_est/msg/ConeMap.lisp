; Auto-generated. Do not edit!


(cl:in-package bound_est-msg)


;//! \htmlinclude ConeMap.msg.html

(cl:defclass <ConeMap> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (cones
    :reader cones
    :initarg :cones
    :type (cl:vector bound_est-msg:Conepos)
   :initform (cl:make-array 0 :element-type 'bound_est-msg:Conepos :initial-element (cl:make-instance 'bound_est-msg:Conepos)))
   (car
    :reader car
    :initarg :car
    :type bound_est-msg:Pos
    :initform (cl:make-instance 'bound_est-msg:Pos)))
)

(cl:defclass ConeMap (<ConeMap>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <ConeMap>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'ConeMap)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bound_est-msg:<ConeMap> is deprecated: use bound_est-msg:ConeMap instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:header-val is deprecated.  Use bound_est-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'cones-val :lambda-list '(m))
(cl:defmethod cones-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:cones-val is deprecated.  Use bound_est-msg:cones instead.")
  (cones m))

(cl:ensure-generic-function 'car-val :lambda-list '(m))
(cl:defmethod car-val ((m <ConeMap>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:car-val is deprecated.  Use bound_est-msg:car instead.")
  (car m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <ConeMap>) ostream)
  "Serializes a message object of type '<ConeMap>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'cones))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'cones))
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'car) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <ConeMap>) istream)
  "Deserializes a message object of type '<ConeMap>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'cones) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'cones)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'bound_est-msg:Conepos))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'car) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<ConeMap>)))
  "Returns string type for a message object of type '<ConeMap>"
  "bound_est/ConeMap")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'ConeMap)))
  "Returns string type for a message object of type 'ConeMap"
  "bound_est/ConeMap")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<ConeMap>)))
  "Returns md5sum for a message object of type '<ConeMap>"
  "918995278f3905a157d2c2d750f8e21a")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'ConeMap)))
  "Returns md5sum for a message object of type 'ConeMap"
  "918995278f3905a157d2c2d750f8e21a")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<ConeMap>)))
  "Returns full string definition for message of type '<ConeMap>"
  (cl:format cl:nil "Header header~%Conepos[] cones~%Pos car~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: bound_est/Conepos~%float32 x~%float32 y~%int8 color ~%~%================================================================================~%MSG: bound_est/Pos~%float32 x~%float32 y~%float32 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'ConeMap)))
  "Returns full string definition for message of type 'ConeMap"
  (cl:format cl:nil "Header header~%Conepos[] cones~%Pos car~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: bound_est/Conepos~%float32 x~%float32 y~%int8 color ~%~%================================================================================~%MSG: bound_est/Pos~%float32 x~%float32 y~%float32 theta~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <ConeMap>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'cones) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'car))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <ConeMap>))
  "Converts a ROS message object to a list"
  (cl:list 'ConeMap
    (cl:cons ':header (header msg))
    (cl:cons ':cones (cones msg))
    (cl:cons ':car (car msg))
))
