; Auto-generated. Do not edit!


(cl:in-package robot_msg-srv)


;//! \htmlinclude VisualVar-request.msg.html

(cl:defclass <VisualVar-request> (roslisp-msg-protocol:ros-message)
  ((a
    :reader a
    :initarg :a
    :type cl:integer
    :initform 0))
)

(cl:defclass VisualVar-request (<VisualVar-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualVar-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualVar-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-srv:<VisualVar-request> is deprecated: use robot_msg-srv:VisualVar-request instead.")))

(cl:ensure-generic-function 'a-val :lambda-list '(m))
(cl:defmethod a-val ((m <VisualVar-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-srv:a-val is deprecated.  Use robot_msg-srv:a instead.")
  (a m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualVar-request>) ostream)
  "Serializes a message object of type '<VisualVar-request>"
  (cl:let* ((signed (cl:slot-value msg 'a)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 18446744073709551616) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualVar-request>) istream)
  "Deserializes a message object of type '<VisualVar-request>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'a) (cl:if (cl:< unsigned 9223372036854775808) unsigned (cl:- unsigned 18446744073709551616))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualVar-request>)))
  "Returns string type for a service object of type '<VisualVar-request>"
  "robot_msg/VisualVarRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualVar-request)))
  "Returns string type for a service object of type 'VisualVar-request"
  "robot_msg/VisualVarRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualVar-request>)))
  "Returns md5sum for a message object of type '<VisualVar-request>"
  "2cb2062d40f8dd0481d223e6c7570480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualVar-request)))
  "Returns md5sum for a message object of type 'VisualVar-request"
  "2cb2062d40f8dd0481d223e6c7570480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualVar-request>)))
  "Returns full string definition for message of type '<VisualVar-request>"
  (cl:format cl:nil "int64 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualVar-request)))
  "Returns full string definition for message of type 'VisualVar-request"
  (cl:format cl:nil "int64 a~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualVar-request>))
  (cl:+ 0
     8
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualVar-request>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualVar-request
    (cl:cons ':a (a msg))
))
;//! \htmlinclude VisualVar-response.msg.html

(cl:defclass <VisualVar-response> (roslisp-msg-protocol:ros-message)
  ((T
    :reader T
    :initarg :T
    :type (cl:vector cl:float)
   :initform (cl:make-array 12 :element-type 'cl:float :initial-element 0.0))
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass VisualVar-response (<VisualVar-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <VisualVar-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'VisualVar-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-srv:<VisualVar-response> is deprecated: use robot_msg-srv:VisualVar-response instead.")))

(cl:ensure-generic-function 'T-val :lambda-list '(m))
(cl:defmethod T-val ((m <VisualVar-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-srv:T-val is deprecated.  Use robot_msg-srv:T instead.")
  (T m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <VisualVar-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-srv:flag-val is deprecated.  Use robot_msg-srv:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <VisualVar-response>) ostream)
  "Serializes a message object of type '<VisualVar-response>"
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream)))
   (cl:slot-value msg 'T))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <VisualVar-response>) istream)
  "Deserializes a message object of type '<VisualVar-response>"
  (cl:setf (cl:slot-value msg 'T) (cl:make-array 12))
  (cl:let ((vals (cl:slot-value msg 'T)))
    (cl:dotimes (i 12)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:aref vals i) (roslisp-utils:decode-double-float-bits bits)))))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<VisualVar-response>)))
  "Returns string type for a service object of type '<VisualVar-response>"
  "robot_msg/VisualVarResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualVar-response)))
  "Returns string type for a service object of type 'VisualVar-response"
  "robot_msg/VisualVarResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<VisualVar-response>)))
  "Returns md5sum for a message object of type '<VisualVar-response>"
  "2cb2062d40f8dd0481d223e6c7570480")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'VisualVar-response)))
  "Returns md5sum for a message object of type 'VisualVar-response"
  "2cb2062d40f8dd0481d223e6c7570480")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<VisualVar-response>)))
  "Returns full string definition for message of type '<VisualVar-response>"
  (cl:format cl:nil "float64[12] T~%bool flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'VisualVar-response)))
  "Returns full string definition for message of type 'VisualVar-response"
  (cl:format cl:nil "float64[12] T~%bool flag~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <VisualVar-response>))
  (cl:+ 0
     0 (cl:reduce #'cl:+ (cl:slot-value msg 'T) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 8)))
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <VisualVar-response>))
  "Converts a ROS message object to a list"
  (cl:list 'VisualVar-response
    (cl:cons ':T (T msg))
    (cl:cons ':flag (flag msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'VisualVar)))
  'VisualVar-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'VisualVar)))
  'VisualVar-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'VisualVar)))
  "Returns string type for a service object of type '<VisualVar>"
  "robot_msg/VisualVar")