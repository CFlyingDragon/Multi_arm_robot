;; Auto-generated. Do not edit!


(when (boundp 'action_test::action_msg1)
  (if (not (find-package "ACTION_TEST"))
    (make-package "ACTION_TEST"))
  (shadow 'action_msg1 (find-package "ACTION_TEST")))
(unless (find-package "ACTION_TEST::ACTION_MSG1")
  (make-package "ACTION_TEST::ACTION_MSG1"))

(in-package "ROS")
;;//! \htmlinclude action_msg1.msg.html


(defclass action_test::action_msg1
  :super ros::object
  :slots (_A _B _C ))

(defmethod action_test::action_msg1
  (:init
   (&key
    ((:A __A) 0)
    ((:B __B) 0)
    ((:C __C) 0)
    )
   (send-super :init)
   (setq _A (round __A))
   (setq _B (round __B))
   (setq _C (round __C))
   self)
  (:A
   (&optional __A)
   (if __A (setq _A __A)) _A)
  (:B
   (&optional __B)
   (if __B (setq _B __B)) _B)
  (:C
   (&optional __C)
   (if __C (setq _C __C)) _C)
  (:serialization-length
   ()
   (+
    ;; int32 _A
    4
    ;; int32 _B
    4
    ;; int32 _C
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _A
       (write-long _A s)
     ;; int32 _B
       (write-long _B s)
     ;; int32 _C
       (write-long _C s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _A
     (setq _A (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _B
     (setq _B (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; int32 _C
     (setq _C (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get action_test::action_msg1 :md5sum-) "e7a68ce4e0b75a9719b4950a7069c9d4")
(setf (get action_test::action_msg1 :datatype-) "action_test/action_msg1")
(setf (get action_test::action_msg1 :definition-)
      "int32 A
int32 B
int32 C
")



(provide :action_test/action_msg1 "e7a68ce4e0b75a9719b4950a7069c9d4")


