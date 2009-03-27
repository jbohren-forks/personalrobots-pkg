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

(defun main ()
  (with-ros-node ("fake-person-sender")
    (advertise "person_position" "robot_msgs/Point")
    ;;(advertise "cloud_ground_filtered" "robot_msgs/PointCloud")
    (advertise "visualizationMarker" "robot_msgs/VisualizationMarker")
    (let ((global_frame (get-param "global_frame_id")))
      (loop 
	 (let* ((x (read)) (y (read)))
	   (if (and (typep x 'real) (typep y 'real))
	       
	       (let ((pos-message (make-instance '<Point>))
		     (cloud-message (make-instance '<PointCloud>)))
		       
		 (setf (x-val pos-message) x
		       (y-val pos-message) y
		       (z-val pos-message) 0)
		 (publish-on-topic "person_position" pos-message)


		 (setf (pts-val cloud-message) (make-array 100)
		       (frame_id-val (header-val cloud-message)) global_frame)
		 (dotimes (i 10)
		   (dotimes (j 10)
		     (setf (aref (pts-val cloud-message) (+ i (* 10 j)))
			   (let ((p (make-instance '<Point32>)))
			     (setf (x-val p) (+ x (* i .1))
				   (y-val p) (+ y (* j .1))
				   (z-val p) 0.1)
			     p))))
		 ;;(publish-on-topic "cloud_ground_filtered" cloud-message)
		 )
	       
	     
	       (princ "Not a legal position")))))))

	   
       
	 
      
