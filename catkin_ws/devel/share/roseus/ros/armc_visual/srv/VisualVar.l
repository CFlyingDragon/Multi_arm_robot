;; Auto-generated. Do not edit!


(when (boundp 'armc_visual::VisualVar)
  (if (not (find-package "ARMC_VISUAL"))
    (make-package "ARMC_VISUAL"))
  (shadow 'VisualVar (find-package "ARMC_VISUAL")))
(unless (find-package "ARMC_VISUAL::VISUALVAR")
  (make-package "ARMC_VISUAL::VISUALVAR"))
(unless (find-package "ARMC_VISUAL::VISUALVARREQUEST")
  (make-package "ARMC_VISUAL::VISUALVARREQUEST"))
(unless (find-package "ARMC_VISUAL::VISUALVARRESPONSE")
  (make-package "ARMC_VISUAL::VISUALVARRESPONSE"))

(in-package "ROS")





(defclass armc_visual::VisualVarRequest
  :super ros::object
  :slots (_a ))

(defmethod armc_visual::VisualVarRequest
  (:init
   (&key
    ((:a __a) 0)
    )
   (send-super :init)
   (setq _a (round __a))
   self)
  (:a
   (&optional __a)
   (if __a (setq _a __a)) _a)
  (:serialization-length
   ()
   (+
    ;; int32 _a
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _a
       (write-long _a s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _a
     (setq _a (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;;
   self)
  )

(defclass armc_visual::VisualVarResponse
  :super ros::object
  :slots (_T _flag ))

(defmethod armc_visual::VisualVarResponse
  (:init
   (&key
    ((:T __T) (make-array 12 :initial-element 0.0 :element-type :float))
    ((:flag __flag) nil)
    )
   (send-super :init)
   (setq _T __T)
   (setq _flag __flag)
   self)
  (:T
   (&optional __T)
   (if __T (setq _T __T)) _T)
  (:flag
   (&optional __flag)
   (if __flag (setq _flag __flag)) _flag)
  (:serialization-length
   ()
   (+
    ;; float32[12] _T
    (* 4    12)
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32[12] _T
     (dotimes (i 12)
       (sys::poke (elt _T i) (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
       )
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32[12] _T
   (dotimes (i (length _T))
     (setf (elt _T i) (sys::peek buf ptr- :float)) (incf ptr- 4)
     )
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass armc_visual::VisualVar
  :super ros::object
  :slots ())

(setf (get armc_visual::VisualVar :md5sum-) "d56d25bda35878684a98158f8a53e101")
(setf (get armc_visual::VisualVar :datatype-) "armc_visual/VisualVar")
(setf (get armc_visual::VisualVar :request) armc_visual::VisualVarRequest)
(setf (get armc_visual::VisualVar :response) armc_visual::VisualVarResponse)

(defmethod armc_visual::VisualVarRequest
  (:response () (instance armc_visual::VisualVarResponse :init)))

(setf (get armc_visual::VisualVarRequest :md5sum-) "d56d25bda35878684a98158f8a53e101")
(setf (get armc_visual::VisualVarRequest :datatype-) "armc_visual/VisualVarRequest")
(setf (get armc_visual::VisualVarRequest :definition-)
      "int32 a
---
float32[12] T
bool flag

")

(setf (get armc_visual::VisualVarResponse :md5sum-) "d56d25bda35878684a98158f8a53e101")
(setf (get armc_visual::VisualVarResponse :datatype-) "armc_visual/VisualVarResponse")
(setf (get armc_visual::VisualVarResponse :definition-)
      "int32 a
---
float32[12] T
bool flag

")



(provide :armc_visual/VisualVar "d56d25bda35878684a98158f8a53e101")


