(in-package geometry)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; point-sets in R^n
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <point-set> (<set>)
  ((n :accessor n :initarg :n)
   (equality-test :initform #'equalp)))

(deftype [point-set] ()
  "[point-set] type includes <point-set> class and vectors"
  '(or <point-set> vector))


(defmethod member? :around (x (s <point-set>))
  (if (vectorp x)
      (if (= (n s) (length x))
	  (call-next-method)
	(values nil ':incorrect-length))
    (values nil ':not-vector)))

(defmethod print-object ((s <point-set>) str)
  (print-unreadable-object (s str :type t :identity nil)
    (format str "in R^~a" (n s))))

(defmethod n ((v vector))
  (length v))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Generic operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric same-point-set (s1 s2 &optional tol)
  (:documentation "same-point-set S1 S2 &optional (TOL *TOL*).  Are these equal point sets?  Unlike set-eq, has a tolerance factor, whose precise meaning depends on the type of set.  May make one-sided errors (i.e. fail to recognize equality) depending on the types.")
  (:method ((s1 <point-set>) (s2 <point-set>) &optional tol) (declare (ignore tol)) (eq s1 s2)))

(defgeneric closest-points (s1 s2)
  (:documentation "closest-points S1 S2.  Let D be the inf distance between S1 and S2, and P1 and P2 two points that achieve this distance (consequences undefined if no such pair exists).  Return 1) P1 2) P2 3) D.")
  (:method ((s1 vector) (s2 vector))
    (values s1 s2 (l2-dist s1 s2))))


(defgeneric is-convex (s)
  (:documentation "Is S known to be convex?")
  (:method ((s vector)) t))


(defgeneric min-distance (s1 s2)
  (:documentation "Return the infimum of the L2 distances between two point sets.")
  (:method (s1 s2) (nth-value 2 (closest-points s1 s2)))
  (:method ((s1 <interval>) (s2 <interval>))
	   (when (my> (left-bound s1) (left-bound s2))
	     (rotatef s1 s2))
	   (mymax 0 (my- (left-bound s2) (right-bound s1))))
  (:method (s1 (s2 null)) (declare (ignore s1)) 'infty)
  (:method ((s1 null) s2) (declare (ignore s2)) 'infty))

(defgeneric max-distance (s1 s2)
  (:documentation "Return the supremum of the L2 distances between two point sets."))


(defmethod intersects ((s1 <point-set>) (s2 <point-set>))
  (zerop (min-distance s1 s2)))

(defgeneric diameter (s)
  (:documentation "Diameter POINT-SET.  Max_{x1,x2 \in S} L2-dist(x1,x2)")
  
  ;; Method for unions: max over component-pair distances
  (:method ((s <implicit-union>))
	   (let ((sets (union-sets s)))
	     (reduce-set 
	      #'mymax sets
	      :key #'(lambda (c) (reduce-set #'mymax sets :key #'(lambda (c2) (max-distance c c2))))))))


(defgeneric area (s)
  (:documentation "Return the 2-dimensional area of point-set S.")
  (:method ((s sequence)) 0.0))

(defgeneric centroid (s)
  (:documentation "Return the center of mass of S.")
  (:method ((s vector)) s))

(defgeneric bounding-box (s)
  (:documentation "bounding-box S.
Returns an <axis-aligned-box> that contains S.  

Methods will generally try to make the box as tight as possible, but need not guarantee this."))

(defgeneric bounding-sphere (s)
  (:documentation "bounding-sphere S.
Returns a <sphere> that contains S.
Methods will generally try to make the sphere tight as possible, but need not guarantee this."))

(defgeneric minkowski-reverse-sum (s1 s2)
  (:documentation "minkowski-reverse-sum S1 S2.  Return the set of all points of the form x-y where x \in S1 and y \in S2.")
  )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Boxes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <axis-aligned-box> (<point-set>)
  ((intervals :accessor intervals :initarg :intervals))
  (:documentation "Class <axis-aligned-box> (<set>)

Initargs
:intervals - vector of intervals, one for each dimension."))


(defun make-axis-aligned-box (bounds)
  "make-axis-aligned-box BOUNDS

BOUNDS is of the form ((A1 B1) ... (An Bn)) where the Ai and Bi are reals with Ai <= Bi.  Returns the corresponding (closed) axis-aligned-box in R^n."
  (make-instance '<axis-aligned-box> 
    :intervals (map 'vector 
		 #'(lambda (x) (apply #'make-closed-interval x))
		 bounds)
    :n (length bounds)))


(defmethod member? (x (s <axis-aligned-box>))
  (every #'member? x (intervals s)))

(defmethod is-convex ((s <axis-aligned-box>))
  t)

(defmethod intersects ((s <axis-aligned-box>) (s2 <axis-aligned-box>))
  (every #'intersects (intervals s) (intervals s2)))

(defun lower-left-corner (box)
  "lower-left-corner BOX. Return the lower left corner (or its analog in higher dimensions) of the box."
  (map 'vector #'left-bound (intervals box)))

(defun upper-right-corner (box)
  "upper-right-corner BOX. Return the upper right corner (or its analog in higher dimensions) of the box."
  (map 'vector #'right-bound (intervals box)))

(defun side-lengths (box)
  (map 'vector #'(lambda (i) (- (right-bound i) (left-bound i))) (intervals box)))


(defmethod vdc-sequence ((s <axis-aligned-box>) &optional (constant-space nil) (base 0))
  (with-slots (intervals) s
    (product-vdc-sequence 'vector intervals base constant-space)))


(defmethod vdc-sequence ((s <point-set>) &optional (constant-space nil) (base 0))
  (mvbind (f b) (vdc-sequence (bounding-box s) constant-space base)
    (values
     #'(lambda ()
	 (loop
	   (let ((x (funcall f)))
	     (when (member? x s) (return x)))))
     b)))

(defmethod sample-uniformly ((s <axis-aligned-box>))
  (map 'vector #'(lambda (int)
		   (with-accessors ((a left-bound) (b right-bound)) int
		       (let ((u (random 1.0)))
			 (convex-combination a b u))))
       (intervals s)))

    
  


(defun pprint-box (str b)
  (pprint-logical-block (str nil)
    (print-unreadable-object (b str :type t :identity nil)
      (format str "with bounds ~a, ~a" (mapset 'list #'(lambda (x) (round-decimal (left-bound x) 2)) (intervals b))
	      (mapset 'list #'(lambda (x) (round-decimal (right-bound x) 2)) (intervals b))))))

(set-pprint-dispatch '<axis-aligned-box>  #'pprint-box)




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Half-spaces
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <half-space> (<point-set>)
  ((b :accessor b :initarg :b)
   (a :accessor a :initarg :a))
  (:documentation "<half-space> 

Initargs
:b - real number 
:a - n-dim vector

Represents the set of y in R^n satisfying A*y >= b"))

(defmethod initialize-instance :after ((s <half-space>) &rest args &key a)
  (declare (ignore args))
  (setf (n s) (length a))
  (assert (> (norm a) 0)))

(defmethod member? (x (s <half-space>))
  (>= (inner-product x (a s)) (b s)))

(defmethod is-convex ((s <half-space>))
  t)
  
(defmethod transform (trans (hs <half-space>))
  (inverse-transform (invert trans) hs))

(defmethod inverse-transform ((trans rigid-2d) (hs <half-space>))
  (let* ((theta (rigid-2d-theta trans))
	 (v (centered-v trans))
	 (new-a (2d-rotate (- theta) (a hs))))
    (make-instance '<half-space> :a new-a :b (- (b hs) (inner-product (a hs) v)))))
       

(defun pprint-hs (str s)
  (print-unreadable-object (s str :type t :identity nil)
    (format str "in R^~a.  " (n s))
    (loop 
	with print-sign = nil
	for a across (a s)
	for i below (n s)
	for positive = (> a 0)
	do (format str "~:[~:[~*~;~:[-~;+~]~]~:[~a*~;~*~]x~a~;~]" (zerop a) (or print-sign (not positive))
		   positive (= 1 (abs a)) (abs a) i)
	   (setf print-sign (or print-sign (not (zerop a)))))
    (format str " >= ~a" (b s))))

(set-pprint-dispatch '<half-space> #'pprint-hs)
		     


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Line segments, represented in terms of endpoints
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <line-segment> (<point-set>)
  ((a :accessor a :initarg :a :reader endpoint1)
   (v :accessor v)
   (b :accessor b :initarg :b :reader endpoint2))
      
  (:documentation "<line-segment>

Initargs
:a - endpoint 1
:b - endpoint 2"))

(defmethod initialize-instance :after ((s <line-segment>) &rest args &key a b)
  (declare (ignore args))
  (setf (n s) (length a)
	(v s) (a- b a))
  ;;(setf (ind s) (position-if (fn (not zerop)) v))
  )


(defmethod closest-points ((p vector) (s <line-segment>))
  (with-slots (a b v) s
    (let ((n (inner-product v v)))
      (assert (> n (expt *tol* 2)) nil)
      (let ((t0 (/ (inner-product v (a- p a)) n)))
	(if (between t0 0 1)

	    ;; Case 1: point is above line segment
	    (let ((q (a+ a (a* t0 v))))
	      (values p q (l2-dist p q)))

	    ;; Case 2: use closest end point
	    (let ((d1 (l2-dist p a))
		  (d2 (l2-dist p b)))
	      (if (< d1 d2) 
		  (values p a d1)
		  (values p b d2))))))))

(defmethod closest-points ((s <line-segment>) (p vector))
  (mvbind (a b c) (closest-points p s)
	  (values b a c)))

(defmethod is-convex ((s <line-segment>))
  t)

(defmethod is-empty ((s <line-segment>))
  nil)

(defmethod intersect ((s1 <line-segment>) (s2 <line-segment>))
  (let ((a1 (a s1))
	(a2 (a s2))
	(v1 (v s1))
	(v2 (v s2)))
    (dbind (v11 v12) v1
      (dbind (v21 v22) v2
	(dbind (a11 a12) a1
	  (dbind (a21 a22) a2
	    (let ((det (- (* v11 v22) (* v12 v21))))
	      (if (>= (abs det) 1e-8)
		  
		  ;; Nonparallel
		  (let ((t1 (/ (- (* v12 (- a21 a11)) (* v11 (- a22 a12))) det)))
		    (when (<= 0 t1 1)
		      (when (<= 0 (/ (- (* v22 (- a21 a11)) (* v21 (- a22 a12))) det) 1)
			(a+ a2 (a* t1 v2)))))
		
		;; Parallel
		(let (t1 t2)
		  (cond
		   ((> (abs v11) 1e-8)
		    (setf t1 (/ (- a21 a11) v11)
			  t2 (/ (- (sfirst (b s2)) a11) v11))
		    (unless (< (abs-diff (+ a12 (* t1 v12)) a22) 1e-8)
		      (return-from intersect nil)))
		   (t
		    (setf t1 (/ (- a22 a12) v12)
			  t2 (/ (- (ssecond (b s2)) a12) v12))
		    (unless (< (abs-diff (+ a11 (* t1 v11)) a21) 1e-8)
		      (return-from intersect nil))))
		  
		  (when (> t1 t2) (rotatef t1 t2))
		  (unless (or (< t2 -1e-8) (> t1 (1+ 1e-8)))
		    (maxf t1 0)
		    (minf t2 1)
		    (make-instance '<line-segment>
		      :a (a+ a1 (a* t1 v1)) :b (a+ a1 (a* t2 v1)))))))))))))


(defmethod same-point-set ((s1 <line-segment>) (s2 <line-segment>) &optional (tol *tol*))
  (let ((a1 (a s1))
	(b1 (b s1))
	(a2 (a s2))
	(b2 (b s2)))
    (or (and (close-to a1 a2 tol) (close-to b1 b2 tol))
	(and (close-to a1 b2 tol) (close-to a2 b1 tol)))))

(defmethod print-object ((s <line-segment>) str)
  (print-unreadable-object (s str :type nil :identity nil)
    (format str "Line segment from ~a to ~a" (a s) (b s))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; neighbourhoods
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <neighbourhood> (<point-set>)
  ((base-set :type <point-set> :initarg :base-set :reader base-set)
   (radius :type (real 0) :initarg :radius :reader radius)
   (closed :type boolean :initarg :closed :reader closed :initform t))
  (:documentation "Initargs
:base-set - a <point-set>
:radius - a positive extended real number
:closed - optional, defaults to t.

Implicit representation of the set of points within distance radius of base-set."))

(defmethod initialize-instance :after ((s <neighbourhood>) &rest args &key base-set)
  (declare (ignore args))
  (setf (n s) (n base-set)))

(defun closed-neighbourhood (s r)
  (make-instance '<neighbourhood> :base-set s :radius r :closed t))

(defmethod is-convex ((s <neighbourhood>))
  t)

(defmethod member? (x (s <neighbourhood>))
  (funcall (if (closed s) #'<= #'<)
	   (min-distance x (base-set s)) (radius s)))

(defmethod bounding-box ((s <neighbourhood>))
  (let ((r (radius s)))
    (make-instance '<axis-aligned-box>
      :intervals (map 'vector #'(lambda (i) (make-closed-interval (my- (left-bound i) r) (my+ (right-bound i) r)))
		      (intervals (bounding-box (base-set s)))))))