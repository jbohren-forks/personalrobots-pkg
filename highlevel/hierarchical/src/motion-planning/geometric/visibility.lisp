(in-package mplan)

(defclass <visibility-graph> (<adjacency-list-graph> <union-find-graph>)
  ((polygons :initarg :polygons :reader polygons)))

(defun connect-using-visibility-graph (g c1 c2)
  "connect-using-visibility-graph VISIBILITY-GRAPH SOURCE DEST

C1 and C2 must be rigid-2d configurations with angle 0.  Returns a list of the form (C0 C1 ... Cn) where C0=SOURCE, Cn=DEST, and each consecutive pair is directly connected."

  (let ((v1 (rigid-2d-v c1))
	(v2 (rigid-2d-v c2)))

    (do-elements (comp (connected-components g))
      (do-elements (n1 comp)
	(unless (intersects-some-polygon g (make-instance '<line-segment> :a v1 :b n1))
	  (do-elements (n2 comp)
	    (unless (intersects-some-polygon g (make-instance '<line-segment> :a n2 :b v2))
	      ;; TODO: implement weighted shortest path for this
	      (return-from connect-using-visibility-graph (nconc (list v1) (unweighted-shortest-path g n1 n2) (list v2)))))
	  (return))))))

(defgeneric 2d-polygonal-visibility-graph (robot polygons bounds)
  (:documentation "2d-polygonal-visibility-graph ROBOT POLYGONS BOUNDS

ROBOT is a polygon or 'point
POLYGONS is a set of (possibly intersecting) <polygon>s.  POLYGONS assumed to not have any vertices in common.
BOUNDS is of the form ((Xmin Ymin) (Xmax Ymax)) (vectors may be used instead of lists)

Returns a <graph> whose nodes are labelled by configurations of the resulting free space, with edges between mutually visible vertices.  Note that it's ok for a visibility line to touch the boundary of an obstacle, i.e., the obstacle region is open.")
  
  (:method ((robot (eql 'point)) polygons bounds)

	   (let ((g (make-instance '<visibility-graph> :polygons polygons :node-test #'equalp)))
	     (dbind ((xmin ymin) (xmax ymax)) bounds
	       (flet ((within-bounds (v)
			(and (< xmin (sfirst v) xmax) (< ymin (ssecond v) ymax))))

    
		 ;; Add nodes
		 (do-elements (p polygons)
		   (do-elements (v (vertices p))
		     (when (and (within-bounds v)
				(every #'(lambda (p2) (or (eq p p2) (not (member? v p2)))) polygons))
		       (add-node g v))))

		 
		 ;; Add edges: loop over pairs of polygons
		 (do-elements (p1 polygons g i)
		   (do-elements (p2 polygons nil j)
	    
		     (cond
		      ((= i j)
		       ;; If polygons are same, add edges between successive vertices
		       (do-elements (s (sides p1))
			 (with-accessors ((e1 endpoint1) (e2 endpoint2)) s
			   (when (and (within-bounds e1) (within-bounds e2)
				      (set:each polygons #'(lambda (p) (or (eq p p1) (is-empty (intersect p s))))))
			     (add-edge g (endpoint1 s) (endpoint2 s))))))
	 
		      ((< i j)
		       ;; Else, loop over pairs of vertices
		       (do-elements (v1 (filter ':implicit (vertices p1) #'within-bounds))
			 (do-elements (v2 (filter ':implicit (vertices p2) #'within-bounds))

			   (unless (intersects-some-polygon g (make-instance '<line-segment> :a v1 :b v2))
			       (add-edge g v1 v2))))))))))))


  (:method ((robot <polygon>) polygons bounds &aux (verts (vertices robot)))
	   (dbind ((xmin ymin) (xmax ymax)) bounds
	     (let ((new-xmin (- xmin (set:reduce-set #'mymin verts :key #'sfirst)))
		   (new-xmax (- xmax (set:reduce-set #'mymax verts :key #'sfirst)))
		   (new-ymin (- ymin (set:reduce-set #'mymin verts :key #'ssecond)))
		   (new-ymax (- ymax (set:reduce-set #'mymax verts :key #'ssecond)))
		   (polygons (mapset 'list #'(lambda (p) (minkowski-reverse-sum p robot)) polygons)))
	       
	       (2d-polygonal-visibility-graph 'point polygons 
					      (list (list new-xmin new-ymin) (list new-xmax new-ymax)))))))


(defun intersects-some-polygon (g l)
    
    (any (polygons g)
	 #'(lambda (p)
	     (let ((int (intersect l p)))
	       (and (typep int '<line-segment>)
		    (> (l2-dist (endpoint1 int) (endpoint2 int)) 1e-8))))))
  






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

    
    



  

		  
		  