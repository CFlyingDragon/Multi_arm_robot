;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::VisualVar)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'VisualVar (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::VISUALVAR")
  (make-package "ROBOT_MSG::VISUALVAR"))
(unless (find-package "ROBOT_MSG::VISUALVARREQUEST")
  (make-package "ROBOT_MSG::VISUALVARREQUEST"))
(unless (find-package "ROBOT_MSG::VISUALVARRESPONSE")
  (make-package "ROBOT_MSG::VISUALVARRESPONSE"))

(in-package "ROS")





(defclass robot_msg::VisualVarRequest
  :super ros::object
  :slots (_a ))

(defmethod robot_msg::VisualVarRequest
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
    ;; int64 _a
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int64 _a
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _a (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _a) (= (length (_a . bv)) 2)) ;; bignum
              (write-long (ash (elt (_a . bv) 0) 0) s)
              (write-long (ash (elt (_a . bv) 1) -1) s))
             ((and (class _a) (= (length (_a . bv)) 1)) ;; big1
              (write-long (elt (_a . bv) 0) s)
              (write-long (if (>= _a 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _a s)(write-long (if (>= _a 0) 0 #xffffffff) s)))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int64 _a
#+(or :alpha :irix6 :x86_64)
      (setf _a (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _a (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;;
   self)
  )

(defclass robot_msg::VisualVarResponse
  :super ros::object
  :slots (_T _flag ))

(defmethod robot_msg::VisualVarResponse
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
    ;; float64[12] _T
    (* 8    12)
    ;; bool _flag
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float64[12] _T
     (dotimes (i 12)
       (sys::poke (elt _T i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _flag
       (if _flag (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float64[12] _T
   (dotimes (i (length _T))
     (setf (elt _T i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     )
   ;; bool _flag
     (setq _flag (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass robot_msg::VisualVar
  :super ros::object
  :slots ())

(setf (get robot_msg::VisualVar :md5sum-) "2cb2062d40f8dd0481d223e6c7570480")
(setf (get robot_msg::VisualVar :datatype-) "robot_msg/VisualVar")
(setf (get robot_msg::VisualVar :request) robot_msg::VisualVarRequest)
(setf (get robot_msg::VisualVar :response) robot_msg::VisualVarResponse)

(defmethod robot_msg::VisualVarRequest
  (:response () (instance robot_msg::VisualVarResponse :init)))

(setf (get robot_msg::VisualVarRequest :md5sum-) "2cb2062d40f8dd0481d223e6c7570480")
(setf (get robot_msg::VisualVarRequest :datatype-) "robot_msg/VisualVarRequest")
(setf (get robot_msg::VisualVarRequest :definition-)
      "int64 a
---
float64[12] T
bool flag

")

(setf (get robot_msg::VisualVarResponse :md5sum-) "2cb2062d40f8dd0481d223e6c7570480")
(setf (get robot_msg::VisualVarResponse :datatype-) "robot_msg/VisualVarResponse")
(setf (get robot_msg::VisualVarResponse :definition-)
      "int64 a
---
float64[12] T
bool flag

")



(provide :robot_msg/VisualVar "2cb2062d40f8dd0481d223e6c7570480")


