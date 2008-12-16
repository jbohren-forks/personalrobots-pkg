(in-package geom)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Class
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <polygon> (<point-set>)
  ((half-spaces :accessor half-spaces :initarg :half-spaces :type sequence)
   (n :initform 2)
   (vertices :accessor vertices :initarg :vertices :type sequence)
   (bounding-box :reader bbox :writer set-bbox :type (or symbol <axis-aligned-box>) :initform ':unassigned)
   (bounding-sphere :reader bsphere :writer set-bsphere :type (or <sphere> symbol) :initform ':unassigned)
   (area :reader polygon-area :writer set-polygon-area :initform ':unassigned)
   (centroid :reader polygon-centroid :writer set-polygon-centroid :initform ':unassigned)
   (diameter :reader cached-diameter :writer set-diameter :initform ':unassigned))
  (:documentation "Class <polygon>.  Represents a closed, *convex* polygonal region in R^2.

Initargs
:vertices - sequence of vertices, in clockwise order.
OR
:half-spaces
OR
:unordered-vertices - sequence of vertices in any order.  The passed-in vector may be modified to put in order.

Properties of polygons
- Vertex ordering preserved under rigid transformations
"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Initialization
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *verify-polygon-convexity* t)

(defmethod initialize-instance :after ((p <polygon>) &rest args &key vertices half-spaces unordered-vertices)
  (declare (ignore args))
  
  ;; Convert from list if necessary
  (macrolet ((coerce-to-array (place)
		 (let ((x (gensym)))
		   `(let ((,x ,place))
		      (unless (arrayp ,x)
			(setf ,place (coerce ,x 'vector))))))
	     (convert (v)
	       `(progn (coerce-to-array ,v)
		       (dotimes (i (length ,v) ,v)
			 (coerce-to-array (aref ,v i))))))
    
    (when unordered-vertices
      (convert unordered-vertices)
      ;; order if necessary
      (let ((center (vertex-center unordered-vertices)))
	(setf (vertices p) 
	      (setf vertices (sort unordered-vertices #'< :key #'(lambda (v) (angle (a- v center) #(1 0))))))))
    
    (when vertices
      (setf (vertices p) (convert vertices))
      ))
  
  (assert (or vertices half-spaces))
  (when (and vertices *verify-polygon-convexity*)
    (mvbind (in-order v1 v2 v3) (in-clockwise-order vertices)
	    (assert in-order nil "Vertices ~a, ~a, and ~a not in clockwise order" v1 v2 v3)))
  
  ;; Compute half spaces from vertices if necessary
  (unless half-spaces
    (setf (half-spaces p) (vertices-to-half-spaces vertices)))
  
  ;; Compute vertices from half-spaces if necessary
  (unless vertices
    (setf (vertices p) (half-spaces-to-vertices half-spaces))))
  

(defun half-spaces-to-vertices (hs)
  (let ((k (length hs)))
    (mapset 'vector
      #'(lambda (i)
	  (let ((hs1 (aref hs i))
		(hs2 (aref hs (if (> i 0) (1- i) (1- k)))))
	    (intersect-lines (aref (a hs1) 0) (aref (a hs1) 1) (b hs1)
			     (aref (a hs2) 0) (aref (a hs2) 1) (b hs2))))
      k)))


(defun vertices-to-half-spaces (vertices)
  (let ((k (length vertices)))
    (mapset 
     'vector
     #'(lambda (i)
	 (let* ((v1 (aref vertices i))
		(v2 (aref vertices (if (= i (1- k)) 0 (1+ i))))
		(x1 (aref v1 0))
		(y1 (aref v1 1))
		(n (make-array 2 :element-type 'float :initial-element 0.0)))
	   (setf (aref n 0) (- (aref v2 1) y1)
		 (aref n 1) (- x1 (aref v2 0)))
	   (make-instance '<half-space> :b (inner-product n v1) :a n)))
     k)))

(defun box->polygon (b)
  (dbind (i1 i2) (intervals b)
    (let ((x0 (left-bound i1))
	  (x1 (right-bound i1))
	  (y0 (left-bound i2))
	  (y1 (right-bound i2)))
      (make-instance '<polygon>
	:vertices (list (list x0 y0) (list x0 y1) (list x1 y1) (list x1 y0))))))


(defun square (&key center angle radius)
  "Return a <polygon> representing a square.  Provide one of the following key combinations

1. CENTER is a 2-vector, ANGLE is the angle rotation in clockwise radians from being axis parallel, RADIUS is the distance from the center to vertices."
  
  (let* ((v1 (2d-rotate (+ angle (/ pi 4)) (vector radius 0)))
	 (v2 (2d-normal v1)))
    (make-instance '<polygon> 
		   :vertices (list (a+ center v1) (a- center v2) (a- center v1) (a+ center v2)))))
		   

  




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (x (p <polygon>))
  (every #'(lambda (h) (member? x h)) (half-spaces p)))

(defmethod is-convex ((p <polygon>))
  t)

(defmethod is-empty ((p <polygon>))
  (not (member? (vertex-center p) p)))

(defmethod transform ((trans rigid-2d) (p <polygon>))
  (flet ((apply-transform (s) (transform trans s)))
    (make-instance '<polygon>
      :half-spaces (map 'vector #'apply-transform (half-spaces p))
      :vertices (map 'vector #'apply-transform (vertices p)))))

(defmethod intersects ((p <polygon>) (p2 <polygon>))
  ;; Makes use of the geometric fact that given two nonintersecting convex polygons,
  ;; one of their sides strictly separates them
  (and (intersects (bounding-box p) (bounding-box p2))
       (every #'(lambda (h) (intersects h p2)) (half-spaces p))
       (every #'(lambda (h) (intersects h p)) (half-spaces p2))))

(defmethod intersects ((h <half-space>) (p2 <polygon>))
  (some #'(lambda (v) (member? v h)) (vertices p2)))

(defmethod intersects ((p <polygon>) (h <half-space>))
  (intersects h p))

(def-symmetric-method intersect ((p <polygon>) (l <line-segment>))
  (let ((a (a l))
	(b (b l))
	(side-intersections nil))
    (cond
     ((and (member? a p) (member? b p)) l)
     (t
      (do-elements (side (sides p))
	(let ((int (intersect side l)))
	  (typecase int
	    (vector (pushnew int side-intersections :test #'close-to)
		    (when (length-exceeds side-intersections 1) (return)))
	    (<line-segment> (return-from intersect int)))))
      (when side-intersections
	(if (cdr side-intersections)
	    (make-instance '<line-segment> :a (first side-intersections) :b (second side-intersections))
	  (let ((i (car side-intersections)))
	    (cond ((member? a p) (make-instance '<line-segment> :a a :b i))
		  ((member? b p) (make-instance '<line-segment> :a b :b i))
		  (t i)))))))))
	    
	
      
    
    


(defmethod subset ((p <polygon>) (p2 <polygon>))
  (every #'(lambda (v) (member? v p2)) (vertices p)))

(defmethod same-point-set ((p1 <polygon>) (p2 <polygon>) &optional (tol *tol*))
  "Check closeness of vertices."
  (let* ((v1 (vertices p1))
	 (v2 (vertices p2))
	 (l (length v1)))
    (and (= l (length v2))
	 (any l
	      #'(lambda (offset)
		  (each l #'(lambda (k) (close-to (aref v1 k) (aref v2 (mod+ l k offset)) tol))))))))


(def-symmetric-method same-point-set ((p <polygon>) (b <axis-aligned-box>) &optional (tol *tol*))
  (same-point-set p (box->polygon b) tol))


(defun compute-bounding-sphere (p)
  (let* ((v (vertices p))
	 (c (vertex-center v)))
    (make-instance '<sphere>
      :centre c
      :radius (reduce-set #'mymax v :key #'(lambda (vertex) (l2-dist vertex c))))))
    

(defun compute-bounding-box (p)
  (with-slots (vertices) p
    (make-axis-aligned-box
     (mapset 'list
	     #'(lambda (i)
		 (list (reduce-set #'mymin vertices :key #'(lambda (x) (aref x i)))
		       (reduce-set #'mymax vertices :key #'(lambda (x) (aref x i)))))
	     2))))

(defmethod bounding-box ((p <polygon>))
  (compute-memoized (bbox set-bbox compute-bounding-box) p))

(defmethod bounding-sphere ((p <polygon>))
  (compute-memoized (bsphere set-bsphere compute-bounding-sphere) p))

(defun compute-polygon-area (p)
  (let ((vert (vertices p)))
    (* .5
       (sum-over 
	(length vert)
	#'(lambda (i)
	    (let ((v1 (aref-mod vert i))
		  (v2 (aref-mod vert (1+ i))))
	      (dbind (x1 y1) v1
		(dbind (x2 y2) v2
		  (- (* x2 y1) (* x1 y2))))))))))


(defmethod area ((p <polygon>))
  (compute-memoized (polygon-area) p))

(defmethod centroid ((p <polygon>))
  (compute-memoized (polygon-centroid) p))

(defun compute-polygon-centroid (p)
  (let ((vert (vertices p))
	(a (* 6 (area p)))
	(sx 0)
	(sy 0))
    
    (do-elements (p vert (vector (/ sx a) (/ sy a)) i)
      (dbind (x1 y1) p
	(dbind (x2 y2) (aref-mod vert (1+ i))
	  (let ((d (- (* x2 y1) (* x1 y2))))
	    (incf sx (* d (+ x1 x2)))
	    (incf sy (* d (+ y1 y2)))))))))
	      


(defmethod min-distance ((poly1 <polygon>) (poly2 <polygon>))
  (iunless (intersects poly1 poly2)
    (flet ((one-way-dist (p1 p2)
	     (let ((v1 (vertices p1))
		   (sides (sides p2)))
	       (reduce-set 
		#'mymin v1 
		:key #'(lambda (v)
			 (reduce-set 
			  #'mymin sides
			  :key #'(lambda (s) (min-distance v s))))))))
      
      (mymin (one-way-dist poly1 poly2) (one-way-dist poly2 poly1)))))


(defmethod closest-points ((p vector) (poly <polygon>))
  (if (member? p poly)
      (values p p 0)
      (let ((best 'infty) q)
	(do-elements (s (sides poly) (values p q best))
	  (mvbind (p1 p2 d) (closest-points p s)
		  (declare (ignore p1))
		  (when (my< d best)
		    (setf best d q p2)))))))

(defmethod closest-points ((poly <polygon>) (p vector))
  (mvbind (a b c) (closest-points p poly) (values b a c)))

(def-symmetric-method max-distance ((p vector) (s <polygon>))
  (reduce-set #'mymax (vertices s) :key #'(lambda (v) (l2-dist p v))))

(defmethod max-distance ((p1 <polygon>) (p2 <polygon>))
  (reduce-set #'mymax (vertices p1)
	      :key #'(lambda (v) (max-distance v p2))))

(defmethod diameter ((p <polygon>))
  (polygon-diameter p))

(define-memoized-reader polygon-diameter
    #'cached-diameter #'set-diameter
    #'(lambda (p) (max-distance p p)))

(defgeneric vertex-center (p)
  (:documentation "center of a polygon or vertex set.")
  (:method ((p <polygon>))
	   (let ((vertices (vertices p)))
	     (a/ (apply #'a+ (coerce vertices 'list)) (length vertices))))
  (:method ((v vector))
	   (vertex-center (coerce v 'list)))
  (:method ((v list))
	   (a/ (apply #'a+ v) (length v))))


(defun shrink-polygon (p r)
  "shrink-polygon POLYGON R.  Returns a new point set P2, either a <polygon> or nil, which consists of all points X such that a circle of radius R centered at X is contained in P.   P2 is formed by moving the sides of P towards the center by R."
  (let* ((hspaces (map 'vector
		    #'(lambda (hs)
			(with-accessors ((a a) (b b)) hs
			  (make-instance '<half-space> :a a :b (+ b (* r (norm a))))))
		    (half-spaces p)))
	 (vertices (half-spaces-to-vertices hspaces))
	 (center (vertex-center vertices)))
    
    ;; To check that the polygon is nonempty, we compute what the vertices would be and see if their center actually lies in all the half-spaces
    ;; Note we're using that everything's convex
    (if (every #'(lambda (hs) (member? center hs)) hspaces)
	(make-instance '<polygon> :half-spaces hspaces :vertices vertices)
      nil)))

(defun expand-polygon (p r)
  "expand-polygon POLYGON RADIUS.  Return a new polygon P2, obtained by moving each side of P outward by RADIUS."
  (let ((hspaces (map 'vector
		   #'(lambda (hs)
		       (with-accessors ((a a) (b b)) hs
			 (make-instance '<half-space> :a a :b (- b (* r (norm a))))))
		   (half-spaces p))))
    (make-instance '<polygon> :half-spaces hspaces)))

(defmethod sample-uniformly ((p <polygon>))
  (let ((b (bounding-box p)))
    (rejection-sampling #'(lambda () (sample-uniformly b)) p)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun intersect-lines (a b c d e f)
  (macrolet ((trunc (x) `(when (< (abs ,x) *tol*) (setf ,x 0))))
    (trunc b)
    (trunc d)
    (trunc a)
    (cond ((< (abs a) (abs d))
	   (intersect-lines d e f a b c))
	  ((zerop a)
	   (assert (not (zerop b)))
	   `#(,(/ (- f (/ (* e c) b)) d) ,(/ c b)))
	  (t (let ((y (/ (- f (/ (* d c) a)) (- e (/ (* d b) a)))))
	       `#(,(/ (- c (* b y)) a) ,y))))))


(defun in-clockwise-order (vertices &aux (n (length vertices)))
  (dotimes (i (1- n) t)
    (let ((v1 (aref-mod vertices (1- i)))
	  (v2 (aref vertices i))
	  (v3 (aref vertices (1+ i))))
      (unless (clockwise v1 v2 v3)
	(return (values nil v1 v2 v3))))))

(defun clockwise (p1 p2 p3)
  (dbind (a b) p1
    (dbind (c d) p2
      (dbind (e f) p3
	(<= (* (- f b) (- c a)) (* (- d b) (- e a)))))))

(defun sides (p)
  "sides POLYGON.  Return set of line segments representing sides of polygon.  Side I goes from vertex I-1 to vertex I."
  (let* ((v (vertices p))
	 (m (length v)))
    (mapset 'vector
	    #'(lambda (i)
		(let ((j (mod-dec m i)))
		  (make-instance '<line-segment> :a (aref v i) :b (aref v j))))
	    m)))
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((p <polygon>) str)
  (print-unreadable-object (p str :type t :identity nil)
    (format str "with ~a vertices" (length (vertices p)))))

(defun pprint-polygon (str p)
  (pprint-logical-block (str nil)
    (print-unreadable-object (p str :type t :identity nil)
      (format str "with vertices ~w" (map 'vector #'round-array (vertices p))))))

(set-pprint-dispatch '<polygon> #'pprint-polygon)


(defun consistent-polygon? (p &optional (tol .0001))
  "consistent-polygon P &optional (TOL .0001).

Check that the half-space and vertex representations stored in P are consistent to within TOL.  If they are return t.  Otherwise, if they're different lengths, assert.  Otherwise, return nil, I, V, V', where V is the Ith vertex and V' is the Ith computed vertex from the half-spaces."
  
  (with-slots (vertices half-spaces) p
    (assert (= (length vertices) (length half-spaces)))
    (assert (in-clockwise-order (vertices p)))
    (let* ((computed-vertices (half-spaces-to-vertices half-spaces))
	   (i (find-element (length vertices)
			    #'(lambda (i) (not (close-to (aref vertices i) (aref computed-vertices i) tol))))))
      (or (not i)
	  (values nil i (aref vertices i) (aref computed-vertices i))))))
	
	
	  
  
  