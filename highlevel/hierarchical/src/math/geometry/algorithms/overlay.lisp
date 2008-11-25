(in-package dcel)


(defstruct (event (:conc-name nil))
  point
  (top nil)
  (bottom nil)
  (internal nil))

(defun overlay (d1 d2 &key (nondestructive nil))
  "Return the overlay of two DCEL's, also represented as a DCEL.
Destructively modifies D1 and D2 unless nondestructive is set to t."
  (when nondestructive
    (setf d1 (clone-dcel d1)
	  d2 (clone-dcel d2)))
    
  (let ((event-queue (make-skiplist :comparator #'lex-comp))
	(sweep-status (make-skiplist))
	(edge-splits (make-hash-table :test #'eq)) ;; maps edges to any c
	(left-of (make-hash-table :test #'eq)) ;; maps vertex to half edge immediately to its left
	(vertices (make-hash-table :test #'eq))

	(vertex-face-angles (make-hash-table :test #'eq))
	;; Maps v to a pair of lists, one for each overlay, mapping edge angle to face info
	;; Used for checking which faces in sources correspond to each face in target
	(edges (make-hash-table :test #'eq)))
    (add-events d1 event-queue)
    (add-events d2 event-queue)

    ;; Carry out a top-to-bottom plane sweep where event-queue contains future potential intersection events
    ;; and sweep-status holds the current line segments in left-right order (at the current sweep location)
    (until (empty? event-queue)
      (let ((event (minimum event-queue t)))
	(let ((top (top event))
	      (bottom (bottom event))
	      (internal (internal event))
	      (point (point event)))
	  (format t "~&Event ~a" event)
	  
	  (let ((v (update-overlay d1 d2 edge-splits top bottom internal 
				   point edges vertices vertex-face-angles)))
	    ;; 2. Do bookkeeping necessary for plane sweep

	    ;; The comparison function infinitesemally above this y-coordinate
	    (setf (comparator sweep-status) (line-comparator event 1))

	    ;; Remove items for finished and internal edges
	    (do-elements (e (disjoint-union bottom internal))
	      (assert (del sweep-status e) nil "Unable to delete ~a from ~a" e sweep-status))
	
	    ;; The comparison function infinitesemally below this y-coordinate
	    (setf (comparator sweep-status) (line-comparator event -1))

	    ;; Add items for each new edge and internal edges
	    (let ((leftmost nil)
		  (rightmost nil)
		  (x (aref point 0))
		  (y (aref point 1)))
	      (flet ((left-of (x y)
		       (or (null y) (eq '< (funcall (comparator sweep-status) x y))))
		     (right-of (x y)
		       (or (null y) (eq '> (funcall (comparator sweep-status) x y)))))
		(do-elements (e (disjoint-union top internal))
		  (insert sweep-status e)
		  (when (left-of e leftmost)
		    (setf leftmost e))
		  (when (right-of e rightmost)
		    (setf rightmost e))))
	  
	      (format t "~2I~&Internal edges ~a~&New edges ~a~&" internal top)

	      ;; Compute potential new intersections
	      (if leftmost

		  ;; 1. If there are edges going downward from v
		  (let ((pred (predecessor sweep-status leftmost))
			(succ (successor sweep-status rightmost)))
		    (format t "~&Pred = ~a, succ = ~a" pred succ)
		    (add-possible-intersection event-queue y leftmost pred)
		    (add-possible-intersection event-queue y rightmost succ)
		    (setf (gethash v left-of) pred))

		  ;; 2. Otherwise
		  (let* ((comp (point-comparator event))
			 (pred (predecessor sweep-status x :comparator comp))
			 (succ (successor sweep-status x :comparator comp)))
		    (format t "~&Pred = ~a, succ = ~a" pred succ)
		    (add-possible-intersection event-queue y pred succ)
		    (setf (gethash v left-of) pred))))))))

    ;; After the plane sweep
    ;; 1. Update the left-of pointers correctly since some edges may have been split
    (maphash #'(lambda (v e)
		 (when e
		   (let ((y (ssecond (coords v))))
		     (dolist (e2 (gethash e edge-splits))
		       (when (<= (ssecond (to-coords e2)) y (ssecond (from-coords e2)))
			 (setf e e2)))
		     (assert (<= (ssecond (to-coords e)) y (ssecond (from-coords e)))))
		   (setf (gethash v left-of) e)))
	     left-of)



    ;; 2. Compute the boundary cycles
    (mvbind (cycles edge-cycles) (compute-boundary-cycles edges)
      (declare (ignore edge-cycles))
      (mvbind (cycle-types leftmost-vertices) (compute-cycle-types cycles)

	(terpri)
	(pprint-hash cycle-types)
	(terpri)
	(pprint-hash leftmost-vertices)

	;; 3. Compute an adjacency graph among the boundary cycles
	(let ((faces (create-overlay-faces left-of cycle-types leftmost-vertices)))
	  
	  ;; 4. Create and return the DCEL
	  (create-dcel faces edges vertices))))))


(defclass <overlay-graph> (<adjacency-list-graph> <union-find-graph>)
  ())

(defun create-overlay-faces (left-of cycle-types leftmost-vertices)
  (let ((g (make-instance '<overlay-graph>)))
    (add-node g 'dummy)
    (do-hash (cycle type cycle-types)
      (add-node g cycle)
      (when (eq type 'internal)
	(add-edge g cycle (or (evaluate left-of (evaluate leftmost-vertices cycle)) 'dummy))))

    ;; OK, now each connected component of G corresponds to a face
    (mapset 'vector
	    #'(lambda (comp)
		(let ((outer nil) (inner nil))
		  (do-elements (cycle comp)
		    (if (or (eq cycle 'dummy) 
			    (eq (evaluate cycle-types cycle) 'external))
			(push cycle outer)
			(push cycle inner)))
		  
		  ;; There should be exactly one outer boundary
		  (assert (and outer (not (cdr outer))))
		  (let* ((bounding-edge (first outer))
			 (f (make-face :face-info nil :bounding-edge (unless (eq bounding-edge 'dummy) bounding-edge) 
						      :inner-edges inner)))
		    (do-elements (e (unless (eq bounding-edge 'dummy) (edge-cycle-iterator bounding-edge)) f)
		      (setf (face e) f)))))
	    (connected-components g))))



      

(defun compute-cycle-types (cycles)
  (let ((h (make-hash-table :test #'eq))
	(left (make-hash-table :test #'eq)))

    (flet ((left-of (x y)
	     (or (< (sfirst x) (sfirst y))
		 (and (< (sfirst x) (+ (sfirst y) *tol*)) (< (ssecond x) (ssecond y))))))
      (do-hash (k v cycles)
	(let ((p nil))
	  (do-elements (e (edge-cycle-iterator k))
	    (when (or (null p)
		      (left-of (from-coords e) (from-coords p)))
	      (setf p e)))
	  ;; Now p holds the edge from the leftmost point of the cycle.
	  ;; Store leftmost point, and set the type of cycle according to the angle at p
	  (setf (gethash k left) (from p)

		(gethash k h)
		(let ((a (- (edge-angle p) (edge-angle (prev p)))))
		  (if (or (<= 0 a pi) (<= (* -2 pi) a (- pi)))
		      'external
		      'internal)))))
      (values h left))))


(defun update-overlay (d1 d2 edge-splits top bottom internal 
		       point edges vertices vertex-face-angles)
  "Update overlay during plane sweep"
  (let ((tb1 (or (contains top d1) (contains bottom d1)))
	(tb2 (or (contains top d2) (contains bottom d2)))
	(i1 (contains internal d1))
	(i2 (contains internal d2))
	(v (make-vertex :coords point)))
	      
    (assert (and (not (and tb1 i1)) (not (and tb2 i2))) nil
	    "Can't have the same point being internal and top/bottom for the same DCEL.")

    (unless (and (not i1) (not i2) (or (not tb1) (not tb2)))
      ;; Do this unless this is a vertex of a single dcel
      ;; 1. Create new edges
      (let ((new-edges
	     (map 'vector 
		  #'(lambda (e)
		      (let ((edge (make-edge-pair v (from e))))
			(if (gethash e edge-splits)
			    (push (twin edge) (gethash e edge-splits))
			    (setf (gethash e edge-splits) (list (twin edge))))
			(attach edge (next (twin e)))
			(attach (prev e) (twin edge))
			(setf (from e) v)
			(setf (edge-from v) e)
			(add-edge-pair edges edge)
			edge))
		  internal)))

	;; 2. Recompute prev and next pointers

	(let ((all-edges 
	       (sort (concatenate 'vector new-edges internal top (map 'vector #'twin bottom))
		     #'< :key #'edge-angle)))
	  (dotimes (i (length all-edges))
	    (attach (twin (aref-mod all-edges (1+ i))) (aref all-edges i))))))

    ;; General bookkeeping
    (orf (edge-from v) (or (first top) (when bottom (twin (first bottom)))))
    (dolist (e top) 
      (add-edge-pair edges e) 
      (setf (from e) v)) ;; to deal with the case when the two dcels have separate vertex objects for the same pt
	    
    (dolist (e bottom) (setf (from (twin e)) v))
    (addf vertices v)
    v))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun add-edge-pair (edges e)
  (addf edges e)
  (addf edges (twin e)))

(defun edge-angle (e)
  (dbind (f1 f2) (from-coords e)
    (dbind (t1 t2) (to-coords e)
      (atan (- t2 f2) (- t1 f1)))))

(defun contains (l d)
  (some #'(lambda (e) (eq (car (edge-info e)) d)) l))

(defun compute-boundary-cycles (edges)
  (let ((h (make-hash-table :test #'eq))
	(cycles (make-hash-table :test #'eq)))
    (do-elements (e edges (values cycles h))
      (unless (hash-table-has-key h e)
	(setf (gethash e cycles) t)
	(do-elements (e2 (edge-cycle-iterator e))
	  (setf (gethash e2 h) e))))))
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sweep status data structure
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun point-comparator (event)
  "Used for finding the lines immediately to the left and right of a given point."
  (let ((y (aref (point event) 1)))
    #'(lambda (e x)
	(dsbind (d a b) (edge-info e)
	  (declare (ignore d))
	  (default-comparator (+ b (* a y)) x *tol*)))))

(defun line-comparator (event d)
  "Used to compare line segments at a given y-location.  Ties are broken by looking upward/downward infinitesemally depending on whether d is positive or negative."
  (let ((y (aref (point event) 1)))
    #'(lambda (e1 e2)
	(dsbind (d1 a1 b1) (edge-info e1)
	  (declare (ignore d1))
	  (dsbind (d2 a2 b2) (edge-info e2)
	    (declare (ignore d2))
	    (flet ((compare-at (z) (default-comparator (+ b1 (* a1 z)) (+ b2 (* a2 z)) *tol*)))
	      (let ((v (compare-at y)))
		(if (eq v '=)
		    (compare-at (+ d y))
		    v))))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Lexical ordering used for event queue
;; higher < lower, and within the same row, left < right
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun lex-comp (e1 e2)
  (let ((c1 (point e1))
	(c2 (point e2)))
    (cond ((lex< c1 c2) '<)
	  ((lex< c2 c1) '>)
	  (t '=))))

(defun event-point-comp (e1 p2)
  (let ((p1 (point e1)))
    (cond ((lex< p1 p2) '<)
	  ((lex< p2 p1) '>)
	  (t '=))))

(defun lex< (v1 v2)
  (or (> (aref v1 1) (aref v2 1))
      (and (= (aref v1 1) (aref v2 1))
	   (< (aref v1 0) (aref v2 0)))))

(defun downward? (e)
  (lex< (coords (from e)) (coords (to e))))


(defun add-events (dcel q)
  "Add all initial top and bottom events."
  (do-elements (e (edges dcel))
    (when (downward? e)
      (let ((from (from-coords e))
	    (to (to-coords e)))

	;; First precompute some info about the line segment
	(dbind (a1 b1) from
	  (dbind (a2 b2) to
	    (let ((d (/ (- a2 a1) (- b2 b1))))
	      (setf (edge-info e) (list dcel d (- a1 (* b1 d)))))))
	
	(let ((top-item (find-item q from :comparator #'event-point-comp))
	      (bottom-item (find-item q to :comparator #'event-point-comp)))
	  (aif top-item
	       (push e (top it))
	       (insert q (make-event :point from :top (list e))))
	  (aif bottom-item
	       (push e (bottom it))
	       (insert q (make-event :point to :bottom (list e)))))))))
	

(defun add-possible-intersection (q y e1 e2)
  "Look for possible intersection between adjacent lines in sweep status, and add to the event queue if necessary."
  (when (and e1 e2)
    (let ((p (verify-type (intersect (line-segment e1) (line-segment e2)) (or null (vector * 2)))))
      (when (and p (< (aref p 1) y))
	(let ((item (find-item q p :comparator #'event-point-comp)))
	  ;; (format t "~&Valid intersection.  Event item ~a" item)
	  (cond
	    (item 
	     (unless (member e1 (bottom item))
	       (push e1 (internal item)))
	     (unless (member e2 (bottom item))
	       (push e2 (internal item))))
	    (t (insert q (make-event :point p :internal (list e1 e2))))))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

