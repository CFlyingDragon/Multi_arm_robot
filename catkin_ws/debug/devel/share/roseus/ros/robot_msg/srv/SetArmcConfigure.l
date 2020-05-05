;; Auto-generated. Do not edit!


(when (boundp 'robot_msg::SetArmcConfigure)
  (if (not (find-package "ROBOT_MSG"))
    (make-package "ROBOT_MSG"))
  (shadow 'SetArmcConfigure (find-package "ROBOT_MSG")))
(unless (find-package "ROBOT_MSG::SETARMCCONFIGURE")
  (make-package "ROBOT_MSG::SETARMCCONFIGURE"))
(unless (find-package "ROBOT_MSG::SETARMCCONFIGUREREQUEST")
  (make-package "ROBOT_MSG::SETARMCCONFIGUREREQUEST"))
(unless (find-package "ROBOT_MSG::SETARMCCONFIGURERESPONSE")
  (make-package "ROBOT_MSG::SETARMCCONFIGURERESPONSE"))

(in-package "ROS")





(defclass robot_msg::SetArmcConfigureRequest
  :super ros::object
  :slots (_configure ))

(defmethod robot_msg::SetArmcConfigureRequest
  (:init
   (&key
    ((:configure __configure) "")
    )
   (send-super :init)
   (setq _configure (string __configure))
   self)
  (:configure
   (&optional __configure)
   (if __configure (setq _configure __configure)) _configure)
  (:serialization-length
   ()
   (+
    ;; string _configure
    4 (length _configure)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _configure
       (write-long (length _configure) s) (princ _configure s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _configure
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _configure (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass robot_msg::SetArmcConfigureResponse
  :super ros::object
  :slots (_result ))

(defmethod robot_msg::SetArmcConfigureResponse
  (:init
   (&key
    ((:result __result) nil)
    )
   (send-super :init)
   (setq _result __result)
   self)
  (:result
   (&optional __result)
   (if __result (setq _result __result)) _result)
  (:serialization-length
   ()
   (+
    ;; bool _result
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _result
       (if _result (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _result
     (setq _result (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass robot_msg::SetArmcConfigure
  :super ros::object
  :slots ())

(setf (get robot_msg::SetArmcConfigure :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get robot_msg::SetArmcConfigure :datatype-) "robot_msg/SetArmcConfigure")
(setf (get robot_msg::SetArmcConfigure :request) robot_msg::SetArmcConfigureRequest)
(setf (get robot_msg::SetArmcConfigure :response) robot_msg::SetArmcConfigureResponse)

(defmethod robot_msg::SetArmcConfigureRequest
  (:response () (instance robot_msg::SetArmcConfigureResponse :init)))

(setf (get robot_msg::SetArmcConfigureRequest :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get robot_msg::SetArmcConfigureRequest :datatype-) "robot_msg/SetArmcConfigureRequest")
(setf (get robot_msg::SetArmcConfigureRequest :definition-)
      "string configure
---
bool result

")

(setf (get robot_msg::SetArmcConfigureResponse :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get robot_msg::SetArmcConfigureResponse :datatype-) "robot_msg/SetArmcConfigureResponse")
(setf (get robot_msg::SetArmcConfigureResponse :definition-)
      "string configure
---
bool result

")



(provide :robot_msg/SetArmcConfigure "e4d1173ab6495a30091da13a92937cf6")


