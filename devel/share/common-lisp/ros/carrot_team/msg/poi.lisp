; Auto-generated. Do not edit!


(cl:in-package carrot_team-msg)


;//! \htmlinclude poi.msg.html

(cl:defclass <poi> (roslisp-msg-protocol:ros-message)
  ((poi
    :reader poi
    :initarg :poi
    :type (cl:vector geometry_msgs-msg:Point)
   :initform (cl:make-array 0 :element-type 'geometry_msgs-msg:Point :initial-element (cl:make-instance 'geometry_msgs-msg:Point))))
)

(cl:defclass poi (<poi>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <poi>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'poi)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name carrot_team-msg:<poi> is deprecated: use carrot_team-msg:poi instead.")))

(cl:ensure-generic-function 'poi-val :lambda-list '(m))
(cl:defmethod poi-val ((m <poi>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader carrot_team-msg:poi-val is deprecated.  Use carrot_team-msg:poi instead.")
  (poi m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <poi>) ostream)
  "Serializes a message object of type '<poi>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'poi))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'poi))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <poi>) istream)
  "Deserializes a message object of type '<poi>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'poi) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'poi)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'geometry_msgs-msg:Point))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<poi>)))
  "Returns string type for a message object of type '<poi>"
  "carrot_team/poi")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'poi)))
  "Returns string type for a message object of type 'poi"
  "carrot_team/poi")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<poi>)))
  "Returns md5sum for a message object of type '<poi>"
  "d4ec992df6b6add1e81dc8da9c38c0ab")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'poi)))
  "Returns md5sum for a message object of type 'poi"
  "d4ec992df6b6add1e81dc8da9c38c0ab")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<poi>)))
  "Returns full string definition for message of type '<poi>"
  (cl:format cl:nil "geometry_msgs/Point[] poi~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'poi)))
  "Returns full string definition for message of type 'poi"
  (cl:format cl:nil "geometry_msgs/Point[] poi~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <poi>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'poi) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <poi>))
  "Converts a ROS message object to a list"
  (cl:list 'poi
    (cl:cons ':poi (poi msg))
))
