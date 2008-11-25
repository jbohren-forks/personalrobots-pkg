(in-package geometry)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; (factored sets of) rigid 2d motions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <rigid-2d-motions> (<direct-product-set>)
  ((angles :accessor angles :initarg :angles)
   (offsets :accessor offsets :initarg :offsets)
   (center :accessor center :initarg :center)
   (init-conf :initarg :conf :reader init-conf :initform nil))
  (:documentation "Class <rigid-2d-motions> (<direct-product-set>)

Initargs

Either
:angles - defaults to [0,2pi)
:offsets - required.  Point-set in R^2.
:center - 2d vector.  Defaults to #(0 0)

Or
:conf - a rigid-2d configuration.  Generates the corresponding singleton set.

Represents the set of rigid motions consisting of a rotation about center by an angle in angles, followed by a translation in offsets.")
  (:default-initargs :angles *all-angles* :center #(0 0)))


(defmethod initialize-instance :around ((s <rigid-2d-motions>) &rest args 
					&key angles offsets center conf)
  (assert (xor conf (and angles offsets)))
  (when conf
    (let ((v (rigid-2d-centered-v conf))
	  (theta (rigid-2d-theta conf)))
      (setf (angles s) (setf angles (list theta))
	    (offsets s) (setf offsets (list v))
	    (center s) (setf center (center conf)))))
	   
  (let ((acc (inst-vars:make-inst-var-accessors
	      :creator #'(lambda () (make-instance 'rigid-2d :center center))
	      :readers (vector #'rigid-2d-theta #'rigid-2d-v)
	      :writers (vector #'set-rigid-2d-theta #'set-rigid-2d-v)))
	(sets (list angles offsets)))
    (apply #'call-next-method s ':sets sets ':inst-acc acc args)))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Set operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod is-empty ((s <rigid-2d-motions>))
  (or (is-empty (angles s)) (is-empty (offsets s))))

(defmethod intersects ((s <rigid-2d-motions>) (s2 <rigid-2d-motions>))
  (cond ((init-conf s) (member? (init-conf s) s2))
	((init-conf s2) (member? (init-conf s2) s))
	(t
	 (assert (and (subset *all-angles* (angles s)) (subset *all-angles* (angles s2))) nil
		 "intersection of non-singleton rigid-2d-motion sets only implemented when sets include all rotations")
	 (let ((p1 (offsets s))
	       (p2 (offsets s2))
	       (r (a- (center s) (center s2))))
	   (assert (and (is-convex p1) (is-convex p2)) nil
		   "intersections of rigid-2d-motion sets only implemented when offset sets are polygonal.")
	   (let ((p1 (transform  (make-instance 'rigid-2d :theta 0 :v r) p1)))
	     (between (norm r) (min-distance p1 p2) (max-distance p1 p2)) ;; using convexity
	     )))))

(defmethod vdc-sequence ((s <rigid-2d-motions>) &optional (constant-space nil) (base 0))
  (mvbind (f b) (product-vdc-sequence 'vector
				      (list (angles s) (offsets s))
				      base constant-space)
    (let ((c (center s)))
      (values
       #'(lambda ()
	   (dbind (theta v) (funcall f)
	     (make-instance 'rigid-2d :theta theta :v v :center c)))
       b))))

(defmethod sample-uniformly ((s <rigid-2d-motions>))
  (or (init-conf s)
      (make-instance 'rigid-2d 
		     :center (center s) 
		     :theta (sample-uniformly (angles s))
		     :v (sample-uniformly (offsets s)))))


(defmethod member? ((tr rigid-2d) (s <rigid-2d-motions>))
  (aif (init-conf s)
       (equal-transformations tr it)

       (and (member? (rigid-2d-theta tr) (angles s))
	    (member? (new-v tr (rigid-2d-v tr) (center tr) (center s)) (offsets s)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Conf set ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod compose-transformation-sets ((phi rigid-2d) (s <rigid-2d-motions>))
  (with-slots (angles offsets center) s
    (assert (subset *all-angles* angles))
    (make-instance '<rigid-2d-motions>
      :center center :angles *all-angles*
      :offsets (transform (make-rigid-2d :center (a- (center phi) center)
					 :theta (rigid-2d-theta phi)
					 :v (rigid-2d-v phi))
			  offsets))))


(defmethod transformations-into ((s vector) (p <polygon>) (type (eql 'rigid-2d)) &optional bound)
  (declare (ignore bound))
  (let ((vertex-diffs (map 'vector #'(lambda (v) (a- v s)) (vertices p))))
    (make-instance '<rigid-2d-motions> :center s :offsets (make-instance '<polygon> :vertices vertex-diffs))))

(defmethod transformations-intersecting ((s vector) (p <polygon>) (type (eql 'rigid-2d)))
  (transformations-into s p 'rigid-2d))


(defmethod transformations-into-inner-bound ((s <point-set>) (p <polygon>) (type (eql 'rigid-2d)))
  (with-slots (centre radius) (bounding-sphere s)
    (transformations-into centre (shrink-polygon p radius) 'rigid-2d)))

(defmethod transformations-into-outer-bound ((s <polygon>) (p <polygon>) (type (eql 'rigid-2d)))
  (transformations-into (centroid s) p 'rigid-2d))

(defmethod transformations-intersecting ((s <point-set>) (p <polygon>) (type (eql 'rigid-2d)))
  (with-slots (centre radius) (bounding-sphere s)
    (transformations-into centre (expand-polygon p radius) 'rigid-2d)))


(defmethod swept-region ((s <rigid-2d-motions>) (p <polygon>))
  (if (init-conf s)

      ;; For a single transformation, just apply it
      (transform (init-conf s) p)

      (with-slots (offsets center) s
	(assert (member? center p) nil "swept-region only currently implemented for a single rigid-2d motion, or when the rotations are centered within the polygon.")
    
	;; For sets of transformations, assume all angles are possible (outer bound)
	(expand-polygon
	 (transform (make-rigid-2d :v center :theta 0) offsets)
	 (reduce-set #'mymax (vertices p) :key #'(lambda (v) (l2-dist v center)))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((s <rigid-2d-motions>) str)
  (if (init-conf s)
      (format str "{~a}" (init-conf s))
      (print-unreadable-object (s str :type t :identity nil)
	(format str "~:[centre ~a~;~*~]. Angle in ~a. Offset in ~a" 
		(equalp (center s) #(0.0 0.0)) (center s) (angles s) (offsets s)))))