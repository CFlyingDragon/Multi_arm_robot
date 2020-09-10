; Auto-generated. Do not edit!


(cl:in-package armc_controller-srv)


;//! \htmlinclude SetArmcConfigure-request.msg.html

(cl:defclass <SetArmcConfigure-request> (roslisp-msg-protocol:ros-message)
  ((configure
    :reader configure
    :initarg :configure
    :type cl:string
    :initform ""))
)

(cl:defclass SetArmcConfigure-request (<SetArmcConfigure-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetArmcConfigure-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetArmcConfigure-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name armc_controller-srv:<SetArmcConfigure-request> is deprecated: use armc_controller-srv:SetArmcConfigure-request instead.")))

(cl:ensure-generic-function 'configure-val :lambda-list '(m))
(cl:defmethod configure-val ((m <SetArmcConfigure-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armc_controller-srv:configure-val is deprecated.  Use armc_controller-srv:configure instead.")
  (configure m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetArmcConfigure-request>) ostream)
  "Serializes a message object of type '<SetArmcConfigure-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'configure))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'configure))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetArmcConfigure-request>) istream)
  "Deserializes a message object of type '<SetArmcConfigure-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'configure) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'configure) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetArmcConfigure-request>)))
  "Returns string type for a service object of type '<SetArmcConfigure-request>"
  "armc_controller/SetArmcConfigureRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetArmcConfigure-request)))
  "Returns string type for a service object of type 'SetArmcConfigure-request"
  "armc_controller/SetArmcConfigureRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetArmcConfigure-request>)))
  "Returns md5sum for a message object of type '<SetArmcConfigure-request>"
  "e4d1173ab6495a30091da13a92937cf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetArmcConfigure-request)))
  "Returns md5sum for a message object of type 'SetArmcConfigure-request"
  "e4d1173ab6495a30091da13a92937cf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetArmcConfigure-request>)))
  "Returns full string definition for message of type '<SetArmcConfigure-request>"
  (cl:format cl:nil "string configure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetArmcConfigure-request)))
  "Returns full string definition for message of type 'SetArmcConfigure-request"
  (cl:format cl:nil "string configure~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetArmcConfigure-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'configure))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetArmcConfigure-request>))
  "Converts a ROS message object to a list"
  (cl:list 'SetArmcConfigure-request
    (cl:cons ':configure (configure msg))
))
;//! \htmlinclude SetArmcConfigure-response.msg.html

(cl:defclass <SetArmcConfigure-response> (roslisp-msg-protocol:ros-message)
  ((result
    :reader result
    :initarg :result
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass SetArmcConfigure-response (<SetArmcConfigure-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <SetArmcConfigure-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'SetArmcConfigure-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name armc_controller-srv:<SetArmcConfigure-response> is deprecated: use armc_controller-srv:SetArmcConfigure-response instead.")))

(cl:ensure-generic-function 'result-val :lambda-list '(m))
(cl:defmethod result-val ((m <SetArmcConfigure-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader armc_controller-srv:result-val is deprecated.  Use armc_controller-srv:result instead.")
  (result m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <SetArmcConfigure-response>) ostream)
  "Serializes a message object of type '<SetArmcConfigure-response>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'result) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <SetArmcConfigure-response>) istream)
  "Deserializes a message object of type '<SetArmcConfigure-response>"
    (cl:setf (cl:slot-value msg 'result) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<SetArmcConfigure-response>)))
  "Returns string type for a service object of type '<SetArmcConfigure-response>"
  "armc_controller/SetArmcConfigureResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetArmcConfigure-response)))
  "Returns string type for a service object of type 'SetArmcConfigure-response"
  "armc_controller/SetArmcConfigureResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<SetArmcConfigure-response>)))
  "Returns md5sum for a message object of type '<SetArmcConfigure-response>"
  "e4d1173ab6495a30091da13a92937cf6")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'SetArmcConfigure-response)))
  "Returns md5sum for a message object of type 'SetArmcConfigure-response"
  "e4d1173ab6495a30091da13a92937cf6")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<SetArmcConfigure-response>)))
  "Returns full string definition for message of type '<SetArmcConfigure-response>"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'SetArmcConfigure-response)))
  "Returns full string definition for message of type 'SetArmcConfigure-response"
  (cl:format cl:nil "bool result~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <SetArmcConfigure-response>))
  (cl:+ 0
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <SetArmcConfigure-response>))
  "Converts a ROS message object to a list"
  (cl:list 'SetArmcConfigure-response
    (cl:cons ':result (result msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'SetArmcConfigure)))
  'SetArmcConfigure-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'SetArmcConfigure)))
  'SetArmcConfigure-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'SetArmcConfigure)))
  "Returns string type for a service object of type '<SetArmcConfigure>"
  "armc_controller/SetArmcConfigure")