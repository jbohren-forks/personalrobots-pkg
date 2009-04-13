(roslisp:ros-load-message-types "robot_msgs/Point" "robot_msgs/PointCloud" "robot_msgs/Point32" "robot_msgs/VisualizationMarker")

(defpackage :fake-person
  (:use :roslisp :cl)
  (:export :main)
  (:import-from 
   :robot_msgs
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
    (advertise "person_position" "robot_msgs/Point")
    (sb-thread:make-thread #'publisher :name "fake-person-publisher")
    (loop
       (let ((x (read)) (y (read)))
	 (setq *person-pos* (make-instance '<Point> :x x :y y :z 0))))))

