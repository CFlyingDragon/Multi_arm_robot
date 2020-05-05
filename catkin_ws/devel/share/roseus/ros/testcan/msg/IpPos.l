;; Auto-generated. Do not edit!


(when (boundp 'testcan::IpPos)
  (if (not (find-package "TESTCAN"))
    (make-package "TESTCAN"))
  (shadow 'IpPos (find-package "TESTCAN")))
(unless (find-package "TESTCAN::IPPOS")
  (make-package "TESTCAN::IPPOS"))

(in-package "ROS")
;;//! \htmlinclude IpPos.msg.html


(defclass testcan::IpPos
  :super ros::object
  :slots (_pos _angle _id ))

(defmethod testcan::IpPos
  (:init
   (&key
    ((:pos __pos) 0.0)
    ((:angle __angle) 0.0)
    ((:id __id) 0)
    )
   (send-super :init)
   (setq _pos (float __pos))
   (setq _angle (float __angle))
   (setq _id (round __id))
   self)
  (:pos
   (&optional __pos)
   (if __pos (setq _pos __pos)) _pos)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:serialization-length
   ()
   (+
    ;; float64 _pos
    8
    ;; float64 _angle
    8
    ;; int8 _id
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64 _pos
       (sys::poke _pos (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; int8 _id
       (write-byte _id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64 _pos
     (setq _pos (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _angle
     (setq _angle (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; int8 _id
     (setq _id (sys::peek buf ptr- :char)) (incf ptr- 1)
     (if (> _id 127) (setq _id (- _id 256)))
   ;;
   self)
  )

(setf (get testcan::IpPos :md5sum-) "4e1eb587452ef16e6a9148c35801e972")
(setf (get testcan::IpPos :datatype-) "testcan/IpPos")
(setf (get testcan::IpPos :definition-)
      "float64 pos
float64 angle
int8 id
")



(provide :testcan/IpPos "4e1eb587452ef16e6a9148c35801e972")


