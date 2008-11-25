;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; math/geometry/conf-space/simple-cs.lisp.lisp
;; Represents 2d configuration spaces with obstacles and a robot
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package motion-planning)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Class def
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <simple-cspace> (<set>)
  ((bounds :accessor bounds :initarg :bounds :initform '(#(-infty -infty) #(infty infty)))
   (robot :accessor robot :initarg :robot)
   (obstacles :accessor obstacles :initarg :obstacles)
   (sampler :accessor sampler))
  (:documentation "Class <simple-cspace>

Represents the set of configurations of a robot in a bounded 2d world with obstacles.  The configurations are of type geometry:rigid-2d (x,y,theta).  The distance metric, local planner, and sampler are all based on the embedding into R^2 x S^1

Initargs
:bounds - if this is left out, the world is unbounded.  Otherwise it must look like (#(a b) #(c d)) indicating that the entire robot must be strictly in between the bounds a <= x <= c, b <= y <= d
:robot - point-set
:obstacles - list of point-sets
:cell-resolution - For efficient collision-checking, the portion of the space covered by the obstacles is partitioned into square cells whose intersections with the obstacles are precomputed.  Cell-resolution is the length of the side of each cell.  Defaults to 1. TODO: Currently not implemented.

"))

(defmethod initialize-instance :after ((cs <simple-cspace>) &rest args &key bounds)
  (declare (ignore args))
  (assert bounds nil "For now, simple cspaces must be bounded.")
  (when bounds
    (dbind ((a b) (c d)) bounds
      (setf (obstacles cs)
	(concatenate 'list
		     (obstacles cs)
		     (list (make-instance '<half-space> :a #(1 0) :b c)
			   (make-instance '<half-space> :a #(-1 0) :b (- a))
			   (make-instance '<half-space> :a #(0 1) :b d)
			   (make-instance '<half-space> :a #(0 -1) :b (- b))))
	
	(sampler cs)
	#'(lambda ()
	    (make-rigid-2d 
	     :center #(0 0)
	     :theta (random (* 2 pi))
	     :v `#(,(+ a (* (- c a) (random 1.0)))
		   ,(+ b (* (- d b) (random 1.0))))))))))

      

  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Cspace operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod is-free ((cs <simple-cspace>) (config rigid-2d))
  (and (within-bounds config cs)
       (let ((transformed-robot (transform config (robot cs))))
	 (every #'(lambda (o) (not (intersects o transformed-robot))) (obstacles cs)))))

(defmethod get-path ((cs <simple-cspace>) (conf1 rigid-2d) (conf2 rigid-2d))
  (let ((conf (make-rigid-2d))
	(theta1 (rigid-2d-theta conf1))
	(theta2 (rigid-2d-theta conf2))
	(v1 (rigid-2d-centered-v conf1))
	(v2 (rigid-2d-centered-v conf2)))
    (when (> (abs-diff theta1 theta2) pi)
      (if (> theta1 theta2)
	  (decf theta1 *2pi*)
	(incf theta1 *2pi*)))

    #'(lambda (x)
	(reinitialize-instance conf :center #(0 0) :theta (mod (convex-combination theta2 theta1 x) *2pi*) :v (convex-combination v2 v1 x))
	conf)))

(defmethod distance ((cs <simple-cspace>) (conf1 rigid-2d) (conf2 rigid-2d))
  (+ (l2-dist (rigid-2d-centered-v conf1) (rigid-2d-centered-v conf2))
     (let ((d (abs-diff (rigid-2d-theta conf1) (rigid-2d-theta conf2))))
       (min d (- *2pi* d)))))

(defmethod min-conf-distance ((cs <simple-cspace>) (cs1 <rigid-2d-motions>) (cs2 <rigid-2d-motions>))
  (min-distance (offsets cs1) (offsets cs2)))
  

(defmethod all-confs ((cs <simple-cspace>))
  (objects-satisfying #'(lambda (x) (and (typep x 'rigid-2d) (within-bounds x cs)))))     



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun within-bounds (config cs)
  (every #'< (first (bounds cs)) (rigid-2d-centered-v config) (second (bounds cs))))
