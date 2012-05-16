; Auto-generated. Do not edit!


(cl:in-package GoPro_Stereo-msg)


;//! \htmlinclude pos3d.msg.html

(cl:defclass <pos3d> (roslisp-msg-protocol:ros-message)
  ((X
    :reader X
    :initarg :X
    :type cl:float
    :initform 0.0)
   (Y
    :reader Y
    :initarg :Y
    :type cl:float
    :initform 0.0)
   (Z
    :reader Z
    :initarg :Z
    :type cl:float
    :initform 0.0)
   (distance
    :reader distance
    :initarg :distance
    :type cl:float
    :initform 0.0)
   (bearing
    :reader bearing
    :initarg :bearing
    :type cl:float
    :initform 0.0)
   (flag
    :reader flag
    :initarg :flag
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass pos3d (<pos3d>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <pos3d>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'pos3d)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name GoPro_Stereo-msg:<pos3d> is deprecated: use GoPro_Stereo-msg:pos3d instead.")))

(cl:ensure-generic-function 'X-val :lambda-list '(m))
(cl:defmethod X-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:X-val is deprecated.  Use GoPro_Stereo-msg:X instead.")
  (X m))

(cl:ensure-generic-function 'Y-val :lambda-list '(m))
(cl:defmethod Y-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:Y-val is deprecated.  Use GoPro_Stereo-msg:Y instead.")
  (Y m))

(cl:ensure-generic-function 'Z-val :lambda-list '(m))
(cl:defmethod Z-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:Z-val is deprecated.  Use GoPro_Stereo-msg:Z instead.")
  (Z m))

(cl:ensure-generic-function 'distance-val :lambda-list '(m))
(cl:defmethod distance-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:distance-val is deprecated.  Use GoPro_Stereo-msg:distance instead.")
  (distance m))

(cl:ensure-generic-function 'bearing-val :lambda-list '(m))
(cl:defmethod bearing-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:bearing-val is deprecated.  Use GoPro_Stereo-msg:bearing instead.")
  (bearing m))

(cl:ensure-generic-function 'flag-val :lambda-list '(m))
(cl:defmethod flag-val ((m <pos3d>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader GoPro_Stereo-msg:flag-val is deprecated.  Use GoPro_Stereo-msg:flag instead.")
  (flag m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <pos3d>) ostream)
  "Serializes a message object of type '<pos3d>"
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'X))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Y))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'Z))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'distance))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'bearing))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'flag) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <pos3d>) istream)
  "Deserializes a message object of type '<pos3d>"
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'X) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Y) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'Z) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'distance) (roslisp-utils:decode-single-float-bits bits)))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'bearing) (roslisp-utils:decode-single-float-bits bits)))
    (cl:setf (cl:slot-value msg 'flag) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<pos3d>)))
  "Returns string type for a message object of type '<pos3d>"
  "GoPro_Stereo/pos3d")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'pos3d)))
  "Returns string type for a message object of type 'pos3d"
  "GoPro_Stereo/pos3d")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<pos3d>)))
  "Returns md5sum for a message object of type '<pos3d>"
  "d4b1ecf7db42ce21fc31494bb091e422")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'pos3d)))
  "Returns md5sum for a message object of type 'pos3d"
  "d4b1ecf7db42ce21fc31494bb091e422")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<pos3d>)))
  "Returns full string definition for message of type '<pos3d>"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 Z~%float32 distance~%float32 bearing~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'pos3d)))
  "Returns full string definition for message of type 'pos3d"
  (cl:format cl:nil "float32 X~%float32 Y~%float32 Z~%float32 distance~%float32 bearing~%bool flag~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <pos3d>))
  (cl:+ 0
     4
     4
     4
     4
     4
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <pos3d>))
  "Converts a ROS message object to a list"
  (cl:list 'pos3d
    (cl:cons ':X (X msg))
    (cl:cons ':Y (Y msg))
    (cl:cons ':Z (Z msg))
    (cl:cons ':distance (distance msg))
    (cl:cons ':bearing (bearing msg))
    (cl:cons ':flag (flag msg))
))
