; Auto-generated. Do not edit!


(cl:in-package robot_msg-msg)


;//! \htmlinclude IpPos.msg.html

(cl:defclass <IpPos> (roslisp-msg-protocol:ros-message)
  ((pos
    :reader pos
    :initarg :pos
    :type cl:float
    :initform 0.0)
   (angle
    :reader angle
    :initarg :angle
    :type cl:float
    :initform 0.0)
   (id
    :reader id
    :initarg :id
    :type cl:fixnum
    :initform 0))
)

(cl:defclass IpPos (<IpPos>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <IpPos>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'IpPos)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name robot_msg-msg:<IpPos> is deprecated: use robot_msg-msg:IpPos instead.")))

(cl:ensure-generic-function 'pos-val :lambda-list '(m))
(cl:defmethod pos-val ((m <IpPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:pos-val is deprecated.  Use robot_msg-msg:pos instead.")
  (pos m))

(cl:ensure-generic-function 'angle-val :lambda-list '(m))
(cl:defmethod angle-val ((m <IpPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:angle-val is deprecated.  Use robot_msg-msg:angle instead.")
  (angle m))

(cl:ensure-generic-function 'id-val :lambda-list '(m))
(cl:defmethod id-val ((m <IpPos>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader robot_msg-msg:id-val is deprecated.  Use robot_msg-msg:id instead.")
  (id m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <IpPos>) ostream)
  "Serializes a message object of type '<IpPos>"
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'pos))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'angle))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let* ((signed (cl:slot-value msg 'id)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    )
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <IpPos>) istream)
  "Deserializes a message object of type '<IpPos>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'pos) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'angle) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'id) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<IpPos>)))
  "Returns string type for a message object of type '<IpPos>"
  "robot_msg/IpPos")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'IpPos)))
  "Returns string type for a message object of type 'IpPos"
  "robot_msg/IpPos")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<IpPos>)))
  "Returns md5sum for a message object of type '<IpPos>"
  "4e1eb587452ef16e6a9148c35801e972")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'IpPos)))
  "Returns md5sum for a message object of type 'IpPos"
  "4e1eb587452ef16e6a9148c35801e972")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<IpPos>)))
  "Returns full string definition for message of type '<IpPos>"
  (cl:format cl:nil "float64 pos~%float64 angle~%int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'IpPos)))
  "Returns full string definition for message of type 'IpPos"
  (cl:format cl:nil "float64 pos~%float64 angle~%int8 id~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <IpPos>))
  (cl:+ 0
     8
     8
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <IpPos>))
  "Converts a ROS message object to a list"
  (cl:list 'IpPos
    (cl:cons ':pos (pos msg))
    (cl:cons ':angle (angle msg))
    (cl:cons ':id (id msg))
))
