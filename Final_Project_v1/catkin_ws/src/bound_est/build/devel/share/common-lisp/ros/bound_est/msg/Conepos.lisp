; Auto-generated. Do not edit!


(cl:in-package bound_est-msg)


;//! \htmlinclude Conepos.msg.html

(cl:defclass <Conepos> (roslisp-msg-protocol:ros-message)
  ((x
    :reader x
    :initarg :x
    :type cl:float
    :initform 0.0)
   (y
    :reader y
    :initarg :y
    :type cl:float
    :initform 0.0)
   (color
    :reader color
    :initarg :color
    :type cl:fixnum
    :initform 0))
)

(cl:defclass Conepos (<Conepos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Conepos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Conepos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name bound_est-msg:<Conepos> is deprecated: use bound_est-msg:Conepos instead.")))

(cl:ensure-generic-function 'x-val :lambda-list '(m))
(cl:defmethod x-val ((m <Conepos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:x-val is deprecated.  Use bound_est-msg:x instead.")
  (x m))

(cl:ensure-generic-function 'y-val :lambda-list '(m))
(cl:defmethod y-val ((m <Conepos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:y-val is deprecated.  Use bound_est-msg:y instead.")
  (y m))

(cl:ensure-generic-function 'color-val :lambda-list '(m))
(cl:defmethod color-val ((m <Conepos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader bound_est-msg:color-val is deprecated.  Use bound_est-msg:color instead.")
  (color m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Conepos>) ostream)
  "Serializes a message object of type '<Conepos>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'color)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Conepos>) istream)
  "Deserializes a message object of type '<Conepos>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'color) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Conepos>)))
  "Returns string type for a message object of type '<Conepos>"
  "bound_est/Conepos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Conepos)))
  "Returns string type for a message object of type 'Conepos"
  "bound_est/Conepos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Conepos>)))
  "Returns md5sum for a message object of type '<Conepos>"
  "fd01361a92846dcb10bc57716aaa13a9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Conepos)))
  "Returns md5sum for a message object of type 'Conepos"
  "fd01361a92846dcb10bc57716aaa13a9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Conepos>)))
  "Returns full string definition for message of type '<Conepos>"
  (cl:format cl:nil "float32 x~%float32 y~%int8 color ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Conepos)))
  "Returns full string definition for message of type 'Conepos"
  (cl:format cl:nil "float32 x~%float32 y~%int8 color ~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Conepos>))
  (cl:+ 0
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Conepos>))
  "Converts a ROS message object to a list"
  (cl:list 'Conepos
    (cl:cons ':x (x msg))
    (cl:cons ':y (y msg))
    (cl:cons ':color (color msg))
))
