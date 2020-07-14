;; Auto-generated. Do not edit!


(when (boundp 'bound_est::Conepos)
  (if (not (find-package "BOUND_EST"))
    (make-package "BOUND_EST"))
  (shadow 'Conepos (find-package "BOUND_EST")))
(unless (find-package "BOUND_EST::CONEPOS")
  (make-package "BOUND_EST::CONEPOS"))

(in-package "ROS")
;;//! \htmlinclude Conepos.msg.html


(defclass bound_est::Conepos
  :super ros::object
  :slots (_x _y _color ))

(defmethod bound_est::Conepos
  (:init
   (&key
    ((:x __x) 0.0)
    ((:y __y) 0.0)
    ((:color __color) 0)
    )
   (send-super :init)
   (setq _x (float __x))
   (setq _y (float __y))
   (setq _color (round __color))
   self)
  (:x
   (&optional __x)
   (if __x (setq _x __x)) _x)
  (:y
   (&optional __y)
   (if __y (setq _y __y)) _y)
  (:color
   (&optional __color)
   (if __color (setq _color __color)) _color)
  (:serialization-length
   ()
   (+
    ;; float32 _x
    4
    ;; float32 _y
    4
    ;; int8 _color
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _x
       (sys::poke _x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _y
       (sys::poke _y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; int8 _color
       (write-byte _color s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _x
     (setq _x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _y
     (setq _y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; int8 _color
     (setq _color (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _color 127) (setq _color (- _color 256)))
   ;;
   self)
  )

(setf (get bound_est::Conepos :md5sum-) "fd01361a92846dcb10bc57716aaa13a9")
(setf (get bound_est::Conepos :datatype-) "bound_est/Conepos")
(setf (get bound_est::Conepos :definition-)
      "float32 x
float32 y
int8 color 

")



(provide :bound_est/Conepos "fd01361a92846dcb10bc57716aaa13a9")


