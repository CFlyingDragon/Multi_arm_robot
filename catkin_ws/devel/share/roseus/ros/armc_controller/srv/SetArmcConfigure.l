;; Auto-generated. Do not edit!


(when (boundp 'armc_controller::SetArmcConfigure)
  (if (not (find-package "ARMC_CONTROLLER"))
    (make-package "ARMC_CONTROLLER"))
  (shadow 'SetArmcConfigure (find-package "ARMC_CONTROLLER")))
(unless (find-package "ARMC_CONTROLLER::SETARMCCONFIGURE")
  (make-package "ARMC_CONTROLLER::SETARMCCONFIGURE"))
(unless (find-package "ARMC_CONTROLLER::SETARMCCONFIGUREREQUEST")
  (make-package "ARMC_CONTROLLER::SETARMCCONFIGUREREQUEST"))
(unless (find-package "ARMC_CONTROLLER::SETARMCCONFIGURERESPONSE")
  (make-package "ARMC_CONTROLLER::SETARMCCONFIGURERESPONSE"))

(in-package "ROS")





(defclass armc_controller::SetArmcConfigureRequest
  :super ros::object
  :slots (_configure ))

(defmethod armc_controller::SetArmcConfigureRequest
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

(defclass armc_controller::SetArmcConfigureResponse
  :super ros::object
  :slots (_result ))

(defmethod armc_controller::SetArmcConfigureResponse
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

(defclass armc_controller::SetArmcConfigure
  :super ros::object
  :slots ())

(setf (get armc_controller::SetArmcConfigure :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get armc_controller::SetArmcConfigure :datatype-) "armc_controller/SetArmcConfigure")
(setf (get armc_controller::SetArmcConfigure :request) armc_controller::SetArmcConfigureRequest)
(setf (get armc_controller::SetArmcConfigure :response) armc_controller::SetArmcConfigureResponse)

(defmethod armc_controller::SetArmcConfigureRequest
  (:response () (instance armc_controller::SetArmcConfigureResponse :init)))

(setf (get armc_controller::SetArmcConfigureRequest :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get armc_controller::SetArmcConfigureRequest :datatype-) "armc_controller/SetArmcConfigureRequest")
(setf (get armc_controller::SetArmcConfigureRequest :definition-)
      "string configure
---
bool result

")

(setf (get armc_controller::SetArmcConfigureResponse :md5sum-) "e4d1173ab6495a30091da13a92937cf6")
(setf (get armc_controller::SetArmcConfigureResponse :datatype-) "armc_controller/SetArmcConfigureResponse")
(setf (get armc_controller::SetArmcConfigureResponse :definition-)
      "string configure
---
bool result

")



(provide :armc_controller/SetArmcConfigure "e4d1173ab6495a30091da13a92937cf6")


