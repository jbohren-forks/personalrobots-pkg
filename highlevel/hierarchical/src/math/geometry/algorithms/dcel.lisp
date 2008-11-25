(defpackage doubly-connected-edge-list
  (:export
   
   make-vertex
   make-edge-pair
   make-face-bounded-by
   make-face
   make-dcel-from-faces
   
   twin
   from-coords
   to-coords
   attach

   edge-cycle-iterator
   boundary-iterator
   half-edges-from

   overlay
   
   is-consistent-dcel
   pprint-dcel
   )
   
   
   
  (:use cl utils set skiplist geometry lin-alg graph)
  (:import-from mapping evaluate)
  (:nicknames dcel))


(in-package dcel)



(defstruct (dcel (:conc-name nil) (:constructor create-dcel (faces edges verts)))
  faces
  edges
  verts)

(defstruct (face (:conc-name nil))
  face-info
  bounding-edge
  inner-edges)

(defstruct (half-edge (:conc-name nil))
  from
  next
  prev
  face
  edge-info
  (line-seg ':unassigned)
  twin)

(defstruct (vertex (:conc-name nil))
  coords
  vertex-info 
  edge-from)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic accessors and ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun to (e)
  (from (twin e)))

(defun from-coords (e)
  (coords (from e)))

(defun to-coords (e)
  (coords (to e)))

(defun line-segment (e)
  (compute-memoized (line-seg) e))

(defun set-line-seg (v e)
  (setf (line-seg e) v))

(defun compute-line-seg (e)
  (make-instance '<line-segment> :a (from-coords e) :b (to-coords e)))

(defun attach (&rest half-edges)
  "Set the next and prev pointers to make these half edges form a chain.  To make a cycle, explicitly reinclude the first half-edge at the end."
  (mapcar #'(lambda (e1 e2) 
	      (setf (next e1) e2 
		    (prev e2) e1))
	  half-edges (rest half-edges)))

(defun edge-cycle-iterator (e0)
  "Return an iterator (see do-iterator) over the cycle of half-edges beginning at e."
  (let ((e e0))
    #'(lambda ()
	(if e
	    (values
	     (prog1 e
	       (setf e (next e))
	       (when (eq e e0) (setf e nil)))
	     nil)
	    (values nil t)))))
	    
(defun boundary-iterator (f)
  (edge-cycle-iterator (check-not-null (bounding-edge f) "bounding-edge of ~a" f)))


(defun half-edges-from (v)
  "iterator over the set of half-edges from vertex v, in clockwise order."
  (let* ((e0 (edge-from v))
	 (e e0))
    #'(lambda ()
	(if e
	    (values
	     (prog1 e
	       (setf e (next (twin e)))
	       (when (eq e e0) (setf e nil)))
	     nil)
	    (values nil t)))))



  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Constructors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-edge-pair (v1 v2)
  "Make a half edge pair between V1 and V2.  Return both the half edges.  Set the edge pointer of each vertex to point to the new edges."
  (let ((e12 (make-half-edge :from v1))
	(e21 (make-half-edge :from v2)))
    (setf (twin e12) e21
	  (twin e21) e12
	  (edge-from v1) e12
	  (edge-from v2) e21)
    (values e12 e21)))

(defun make-face-bounded-by (&rest edges)
  "Make a new face bounded by the edges.  Assumed that they are going counterclockwise.  Set the edge next, prev, and face pointers appropriately, but does not change the face pointers of the twin edges.  Return the new face."
  (let ((f (make-face :bounding-edge (first edges))))
    (apply #'attach (slast edges) edges)
    (mapcar #'(lambda (e) (setf (face e) f)) edges)
    f))


(defun make-dcel-from-faces (faces)
  "make a dcel with the given faces, and all resulting edges and vertices."
  (let ((edges (make-hash-table :test #'eq))
	(vertices (make-hash-table :test #'eq)))
    (do-elements (f faces (create-dcel faces edges vertices))
      (when (bounding-edge f)
	(do-elements (e (boundary-iterator f))
	  (addf edges e)
	  (addf edges (twin e))
	  (addf vertices (to e))
	  (addf vertices (from e)))))))


	  
      
(defun clone-dcel (d &key (face-info-copier #'identity) (edge-info-copier #'identity) (vertex-info-copier #'identity))
  "Return a copy of the DCEL.  The arrays that represent individual points are the same, but everything else is fresh, so any modification to the new DCEL that doesn't modify those arrays will not affect the original."
  (let ((copied-vertices (copy-to-table (verts d) #'copy-vertex))
	(copied-edges (copy-to-table (edges d) #'copy-half-edge))
	(copied-faces (copy-to-table (faces d) #'copy-face)))
    (flet ((vertex-copy (v) (gethash v copied-vertices))
	   (edge-copy (e) (gethash e copied-edges))
	   (face-copy (f) (gethash f copied-faces)))

      (maphash #'(lambda (v cv)
		   (setf (edge-from cv) (edge-copy (edge-from v))
			 (vertex-info cv) (funcall vertex-info-copier (vertex-info v))))
	       copied-vertices)

      (maphash #'(lambda (e ce)
		   (setf (from ce) (vertex-copy (from e))
			 (prev ce) (edge-copy (prev e))
			 (next ce) (edge-copy (next e))
			 (face ce) (face-copy (face e))
			 (twin ce) (edge-copy (twin e))
			 (edge-info ce) (funcall edge-info-copier (edge-info e))))
	       copied-edges)

      (maphash #'(lambda (f cf)
		   (setf (bounding-edge cf) (edge-copy (bounding-edge f))
			 (face-info cf) (funcall face-info-copier (face-info f))
			 (inner-edges cf) (mapcar #'edge-copy (inner-edges f))))
	       copied-faces)

      (create-dcel (copy-values copied-faces) (copy-values copied-edges) (copy-values copied-vertices)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun copy-to-table (s copier)
  (let ((h (make-hash-table :test #'eq)))
    (do-elements (x s h)
      (setf (gethash x h) (funcall copier x)))))

(defun copy-values (h)
  (let ((h2 (make-hash-table :test #'eq)))
    (maphash #'(lambda (k v)
		 (declare (ignore k))
		 (setf (gethash v h2) t))
	     h)
    h2))
		 

    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun pprint-dcel (&rest args)
  (bind-pprint-args (str d) args
    (let ((first t))
      (pprint-logical-block (str nil :prefix "[" :suffix "]")
	(do-elements (f (faces d) nil i)
	  (if first
	      (setf first nil)
	      (pprint-newline :mandatory str))
	  (format str "Face ~a: " i)
	  (if (bounding-edge f)
	      (do-iterator (boundary-iterator f) (e)
		(format str "~a " (coords (from e))))
	      (format str "Unbounded")))))))
	  
    
    
(defun is-consistent-dcel (dcel)
  "Check if 1) every edge is its own twin's twin 2) every edge is its previous edge's next edge 3) every face is its bounding edge's contained face 4) every vertex is the source of its corresponding edge"
  (flet ((inconsistent (&rest args)
	   (return-from is-consistent-dcel (values nil args))))
    (let ((edges (edges dcel))
	  (verts (verts dcel))
	  (faces (faces dcel))
	  (unbounded-faces nil))
      (awhen (find-element edges #'(lambda (e) (not (eq e (twin (twin e))))) )
	(inconsistent 'not-twins-twin it))
      (awhen (find-element edges #'(lambda (e) (not (eq e (next (prev e))))) )
	(inconsistent 'not-prevs-next it))
      (awhen (find-element edges #'(lambda (e) (not (eq e (prev (next e))))) )
	(inconsistent 'not-nexts-prev it))
      (awhen (find-element 
	      faces 
	      #'(lambda (f) 
		  (if (bounding-edge f)
		      (any (boundary-iterator f) #'(lambda (e) (not (eq (face e) f))))
		      (progn (push f unbounded-faces) nil))))
	(inconsistent 'not-bounding-edges-contained-face it))
      (awhen (find-element verts #'(lambda (v) (not (eq v (from (edge-from v))))))
	(inconsistent 'not-edges-source it))
      (unless (= (length unbounded-faces) 1)
	(inconsistent 'wrong-num-unbounded-faces unbounded-faces))
      t)))

    
    
(defmethod print-object ((e half-edge) str)
  (print-unreadable-object (e str :type nil :identity nil)
    (format str "Half-edge from ~a to ~a" (from-coords e) (to-coords e))))

(defmethod print-object ((v vertex) str)
  (print-unreadable-object (v str :type nil :identity t)
    (format str "Vertex ~a" (coords v))))