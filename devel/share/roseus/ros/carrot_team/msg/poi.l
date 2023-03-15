;; Auto-generated. Do not edit!


(when (boundp 'carrot_team::poi)
  (if (not (find-package "CARROT_TEAM"))
    (make-package "CARROT_TEAM"))
  (shadow 'poi (find-package "CARROT_TEAM")))
(unless (find-package "CARROT_TEAM::POI")
  (make-package "CARROT_TEAM::POI"))

(in-package "ROS")
;;//! \htmlinclude poi.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(defclass carrot_team::poi
  :super ros::object
  :slots (_poi ))

(defmethod carrot_team::poi
  (:init
   (&key
    ((:poi __poi) ())
    )
   (send-super :init)
   (setq _poi __poi)
   self)
  (:poi
   (&rest __poi)
   (if (keywordp (car __poi))
       (send* _poi __poi)
     (progn
       (if __poi (setq _poi (car __poi)))
       _poi)))
  (:serialization-length
   ()
   (+
    ;; geometry_msgs/Point[] _poi
    (apply #'+ (send-all _poi :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; geometry_msgs/Point[] _poi
     (write-long (length _poi) s)
     (dolist (elem _poi)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; geometry_msgs/Point[] _poi
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _poi (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _poi)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get carrot_team::poi :md5sum-) "d4ec992df6b6add1e81dc8da9c38c0ab")
(setf (get carrot_team::poi :datatype-) "carrot_team/poi")
(setf (get carrot_team::poi :definition-)
      "geometry_msgs/Point[] poi
================================================================================
MSG: geometry_msgs/Point
# This contains the position of a point in free space
float64 x
float64 y
float64 z

")



(provide :carrot_team/poi "d4ec992df6b6add1e81dc8da9c38c0ab")


