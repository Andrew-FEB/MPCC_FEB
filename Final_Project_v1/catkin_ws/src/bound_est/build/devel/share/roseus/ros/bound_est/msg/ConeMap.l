;; Auto-generated. Do not edit!


(when (boundp 'bound_est::ConeMap)
  (if (not (find-package "BOUND_EST"))
    (make-package "BOUND_EST"))
  (shadow 'ConeMap (find-package "BOUND_EST")))
(unless (find-package "BOUND_EST::CONEMAP")
  (make-package "BOUND_EST::CONEMAP"))

(in-package "ROS")
;;//! \htmlinclude ConeMap.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass bound_est::ConeMap
  :super ros::object
  :slots (_header _cones _car ))

(defmethod bound_est::ConeMap
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:cones __cones) (let (r) (dotimes (i 0) (push (instance bound_est::Conepos :init) r)) r))
    ((:car __car) (instance bound_est::Pos :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _cones __cones)
   (setq _car __car)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:cones
   (&rest __cones)
   (if (keywordp (car __cones))
       (send* _cones __cones)
     (progn
       (if __cones (setq _cones (car __cones)))
       _cones)))
  (:car
   (&rest __car)
   (if (keywordp (car __car))
       (send* _car __car)
     (progn
       (if __car (setq _car (car __car)))
       _car)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; bound_est/Conepos[] _cones
    (apply #'+ (send-all _cones :serialization-length)) 4
    ;; bound_est/Pos _car
    (send _car :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; bound_est/Conepos[] _cones
     (write-long (length _cones) s)
     (dolist (elem _cones)
       (send elem :serialize s)
       )
     ;; bound_est/Pos _car
       (send _car :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; bound_est/Conepos[] _cones
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _cones (let (r) (dotimes (i n) (push (instance bound_est::Conepos :init) r)) r))
     (dolist (elem- _cones)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;; bound_est/Pos _car
     (send _car :deserialize buf ptr-) (incf ptr- (send _car :serialization-length))
   ;;
   self)
  )

(setf (get bound_est::ConeMap :md5sum-) "918995278f3905a157d2c2d750f8e21a")
(setf (get bound_est::ConeMap :datatype-) "bound_est/ConeMap")
(setf (get bound_est::ConeMap :definition-)
      "Header header
Conepos[] cones
Pos car
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: bound_est/Conepos
float32 x
float32 y
int8 color 

================================================================================
MSG: bound_est/Pos
float32 x
float32 y
float32 theta
")



(provide :bound_est/ConeMap "918995278f3905a157d2c2d750f8e21a")


