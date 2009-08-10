(roslisp:ros-load-message-types "geometry_msgs/Point" "sensor_msgs/PointCloud" "geometry_msgs/Point32" "visualization_msgs/VisualizationMarker")

(defpackage :fake-person
  (:use :roslisp :cl)
  (:export :main)
  (:import-from 
   :geometry_msgs
   :<Point>
   :<Point32>
   :<PointCloud>
   :<VisualizationMarker>
   :pts-val
   :header-val
   :z-val
   :x-val
   :y-val)
  (:import-from
   :roslib
   :frame_id-val))

(in-package :fake-person)


(defvar *person-pos* nil)


(defun publisher ()
  (loop-at-most-every .1
     (when *person-pos*
       (publish-on-topic "person_position" *person-pos*))))

(defun main ()
  (with-ros-node ("fake-person-sender")
    (advertise "person_position" "geometry_msgs/Point")
    (sb-thread:make-thread #'publisher :name "fake-person-publisher")
    (loop
       (let ((x (read)) (y (read)))
	 (setq *person-pos* (make-instance '<Point> :x x :y y :z 0))))))

