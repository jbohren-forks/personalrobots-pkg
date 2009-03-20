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
    
    
    