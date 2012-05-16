; Auto-generated. Do not edit!


(cl:in-package GoPro_Stereo-msg)


;//! \htmlinclude pos2d.msg.html

(cl:defclass <pos2d> (roslisp-msg-protocol:ros-message)
  ((x1
    :reader x1
    :initarg :x1
    :type cl:float
    :initform 0.0)
   (y1
    :reader y1
    :initarg :y1
    :type cl:float
    :initform 0.0)
   (x2
    :reader x2
    :initarg :x2
    :type cl:float
    :initform 0.0)
   (y2
    :reader y2
    :initarg :y2
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass pos2d (<pos2d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos2d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos2d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name GoPro_Stereo-msg:<pos2d> is deprecated: use GoPro_Stereo-msg:pos2d instead.")))

(cl:ensure-generic-function 'x1-val :lambda-list '(m))
(cl:defmethod x1-val ((m <pos2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:x1-val is deprecated.  Use GoPro_Stereo-msg:x1 instead.")
  (x1 m))

(cl:ensure-generic-function 'y1-val :lambda-list '(m))
(cl:defmethod y1-val ((m <pos2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:y1-val is deprecated.  Use GoPro_Stereo-msg:y1 instead.")
  (y1 m))

(cl:ensure-generic-function 'x2-val :lambda-list '(m))
(cl:defmethod x2-val ((m <pos2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:x2-val is deprecated.  Use GoPro_Stereo-msg:x2 instead.")
  (x2 m))

(cl:ensure-generic-function 'y2-val :lambda-list '(m))
(cl:defmethod y2-val ((m <pos2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:y2-val is deprecated.  Use GoPro_Stereo-msg:y2 instead.")
  (y2 m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <pos2d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:flag-val is deprecated.  Use GoPro_Stereo-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos2d>) ostream)
  "Serializes a message object of type '<pos2d>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y1))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'x2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'y2))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos2d>) istream)
  "Deserializes a message object of type '<pos2d>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y1) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'x2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'y2) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos2d>)))
  "Returns string type for a message object of type '<pos2d>"
  "GoPro_Stereo/pos2d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos2d)))
  "Returns string type for a message object of type 'pos2d"
  "GoPro_Stereo/pos2d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos2d>)))
  "Returns md5sum for a message object of type '<pos2d>"
  "b26924efcba5b88ad62c7ad3ea0376f4")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos2d)))
  "Returns md5sum for a message object of type 'pos2d"
  "b26924efcba5b88ad62c7ad3ea0376f4")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos2d>)))
  "Returns full string definition for message of type '<pos2d>"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos2d)))
  "Returns full string definition for message of type 'pos2d"
  (cl:format cl:nil "float32 x1~%float32 y1~%float32 x2~%float32 y2~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos2d>))
  (cl:+ 0
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos2d>))
  "Converts a ROS message object to a list"
  (cl:list 'pos2d
    (cl:cons ':x1 (x1 msg))
    (cl:cons ':y1 (y1 msg))
    (cl:cons ':x2 (x2 msg))
    (cl:cons ':y2 (y2 msg))
    (cl:cons ':flag (flag msg))
))
