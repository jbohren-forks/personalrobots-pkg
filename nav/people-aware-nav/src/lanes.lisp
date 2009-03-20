(defpackage :lane-following
  (:nicknames :lanes)
  (:use :roslisp :cl :transform-2d)
  (:export :main))

(in-package :lane-following)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Params
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *robot-radius* .32 "Circumscribed radius of robot in metres")
(defparameter *wall-buffer* .2 "Additional buffer distance that we'd like to keep from wall")
(defvar *original-goal*)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun main ()
  (with-ros-node ("lane-changer")

    ;; 1. setup
    (subscribe "corridor")
    (subscribe "person")

    ;; 2. executive
    (with-highlevel-controller (move-base "state" "robot_msgs/Planner2DState" "goal" "robot_msgs/Planner2DGoal")
      (ecase (controller-result move-base (goal-message *original-goal*))
	(succeeded (print "done!"))
	(aborted 
	 (ecase (controller-result move-base (goal-message (waiting-pose current-pose corridor-width)))
	   (aborted (print "Could not move to waiting position --- failing!"))
	   (succeeded (wait-for-person-to-move)
		      (ecase (controller-result move-base (goal-message *original-goal*))
			(succeeded (print "done!"))
			(aborted (print "Failed even after lane-change."))))))))))

      

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun corridor-info (p1 p2 p3)
  "Return 1) the transform from map to corridor frames 2) the corridor width, given that p1 and p2 are points on the left wall, and p3 is on the right wall"
  (let ((d1 (a- p2 p1))
	(d2 (a- p3 p1)))
    (values (transform-between (make-pose p1 (vector-angle d1)) (make-pose (vector 0.0 0.0) (/ pi 2)))
	    (inner-product d2 (unit-vector (transform-point (rotation-matrix (/ pi -2)) d1))))))

(defun waiting-pose (current-pose corridor-width)
  "Return the pose corresponding to 'shifting to the right lane' (all in the corridor frame)"
  (let ((pos (pose-position current-pose))
	(target-wall-offset (- corridor-width *robot-radius* *wall-buffer*)))
    (make-pose (vector target-wall-offset (aref pos 1)) (/ pi 2))))
    
    
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Communication with highlevel controllers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro with-highlevel-controller ((name state-topic state-type goal-topic goal-type) &body body)
  `(let ((,state-message-queue (make-queue)))
     (subscribe state-topic state-type (add-to-queue ,state-message-queue))
     (advertise goal-topic goal-type)
     ;; not quite right
     (flet ((controller-result (name message)
	      (controller-result message goal-topic state-message-queue)))
       ,@body)))


(defun controller-result (goal-pos topic response-queue)
  (publish topic (make-instance '<Planner2DGoal> :position goal-pos))
  (dequeue-wait response-queue))
    
  
