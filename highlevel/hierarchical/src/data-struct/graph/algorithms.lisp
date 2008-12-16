(in-package graph)


(defun adjacency-matrix (g)
  "adjacency-matrix GRAPH.  Returns an NxN boolean matrix M, where M(i,j) iff there is an edge from Node #I to Node #J."
  (let* ((nodes (nodes g))
	 (n (size nodes))
	 (m (make-array (list n n) :element-type 'boolean :initial-element nil)))
    (do-elements (x nodes m i)
      (do-elements (y nodes nil j)
	(when (get-edge g x y) (setf (aref m i j) t))))))

(defun labeled-adjacency-matrix (g)
  "labeled-adjacency-matrix GRAPH.  Return an NxN matrix M, where M(i,j) is the label of the edge from node #i to node #j if such an edge exists, or ':no-edge otherwise."
  (let* ((nodes (nodes g))
	 (n (size nodes))
	 (m (make-array (list n n))))
    (do-elements (x nodes m i)
      (do-elements (y nodes nil j)
	(setf (aref m i j)
	  (aif (get-edge g x y)
	      (label it)
	    ':no-edge))))))
	    
    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; shortest paths
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun unweighted-shortest-path (graph start end &key (directed nil))
  "unweighted-shortest-path GRAPH START END &key (DIRECTED nil)

Return the shortest path (assuming edge costs are constantly 1) from START to END, in the form of a list of nodes, or nil if the nodes are not connected."
  (assert (not directed) nil "unweighted-shortest-path only implemented for undirected case")
  (let ((q (pqueue:make-priority-queue))
	(node-table (make-hash-table :test #'eq))
	(path (list end)))
    
    ;; Make initial queue
    (do-elements (n (nodes graph))
      (let ((dist (if (eq n start) 0 'infty)))
	(pqueue:enqueue q n (my- dist))
	(setf (gethash n node-table) (list dist nil nil))))
    
    (until (zerop (pqueue:num-entries q))
      (let* ((n (pqueue:dequeue-highest q))
	     (entry (check-not-null (gethash n node-table))))
	(dbind (dist visited prev) entry
	  
	  ;; Stop when we reach unreachable nodes
	  (when (eql dist 'infty)
	    (return nil))
	 
	  (assert (or (eq n start) prev))
	  
	  ;; If it's already been visited, do nothing
	  (unless visited
	    (setf (second entry) t)
	    (do-elements (n2 (neighbours graph n))
	      (let ((entry2 (check-not-null (gethash n2 node-table))))
		(dbind (ndist nvisited) entry2
		  (let ((new-dist (1+ dist)))
		    (when (my> ndist new-dist)
		      (assert (not nvisited))
		      (setf (first entry2) new-dist
			    (third entry2) n)
		      (pqueue:enqueue q n2 (- new-dist)))))))))))
    (until (eq (car path) start)
      (let ((prev (third (gethash (car path) node-table))))
	(if prev
	    (push prev path)
	  (return-from unweighted-shortest-path nil))))
    
    path))
    
    
	  
  


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; max weight bipartite matching
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun max-weight-bipartite-matching (graph)
  "G must be a bipartite graph with positive weights.  Uses the Hungarian algorithm to find a max weight matching."
  (mvbind (g l r) (perfectly-matchable-graph graph)
    (let* ((m (size (nodes g)))
	   (k (size l))
	   (matching nil)
	   (free (make-array m :element-type 'boolean))
	   (labeling (make-array m :element-type 'extended-real :initial-element 0.0)))
      
      (labels ((equality-neighbour (i j)
		 (= (+ (aref labeling i) (aref labeling j)) (label (get-edge g i j))))
	       
	       (neighbours (s)
		 ;; neighbours in equality graph
		 (let ((neighbours nil))
		   (dolist (i s neighbours)
		     (for-loop (j k m)
		       (when (equality-neighbour i j)
			 (pushnew j neighbours)))))))
      
	;; Make the initial feasible labeling
	(dotimes (i k)
	  (setf (aref labeling i)
	    (reduce-set #'mymax (edges-from g i) :key #'label)))

	(until (= k (length matching))
	  (fill free t)
	  (dolist (e matching)
	    (setf (aref free (from e)) nil
		  (aref free (to e)) nil))
	  
	  (let* ((i (check-not-null (position t free)))
		 (s (list i))
		 (r nil)
		 (ns (neighbours s))
		 (bp (mapcar #'(lambda (x) (cons x i)) ns)))
	    (assert (< i k))
	    (loop
	      ;; If N(s) = r, then update labels to force N(s) \neq r
	      (when (set-eq ns r)
		(let ((alpha 
		       (reduce-set #'mymin (direct-product 'list k k)
				   :key #'(lambda (x)
					    (dbind (a b) x
					      (let ((c (+ b k)))
						(if (and (member? a s) (not (member? c r)))
						    (my+ (aref labeling a) (aref labeling c)
							 (- (label (get-edge g a c))))
						  'infty)))))))
		  (dolist (a s) (decf (aref labeling a) alpha))
		  (dolist (a r) (incf (aref labeling a) alpha)))
		(setf ns (neighbours s))
		)
	    
	      ;; Look for a member of ns - r
	      (let* ((j (find-element (int-range k m) #'(lambda (x) (and (member x ns :test #'eq) 
									 (not (member x r :test #'eq))))))
		     (nj (check-not-null (find-element k #'(lambda (i) (and (member i s :test #'eq) 
									    (equality-neighbour i j)))))))
		
		;; If it's free, incorporate the resulting augmenting path into the matching
		(if (aref free j)
		    (let ((path (list nj j)))
		      (until (equal (car path) i)
			(push (cdr (assoc (car path) bp :test #'eq)) path))
		      (loop
			  for a from 0
			  for b in path
			  for c in (rest path)
			  do (if (oddp a)
				 (deletef matching (get-edge g c b))
			       (push (get-edge g b c) matching)))
		      (return))
		  
		  ;; Otherwise, grow s and r and continue
		  (let ((n (from (check-not-null (find j matching :key #'to :test #'eq)))))
		    (push j r)
		    (push n s)
		    (do-elements (i (neighbours (list n)))
		      (pushnew i ns))
		    (push (cons j nj) bp)
		    (push (cons n j) bp)))))))
	
	;; Return those edges in the original graph that correspond to this matching
	(remove nil
		(mapset 'list
			#'(lambda (e) (get-edge-between graph (nth (from e) l) (nth (- (to e) k) r)))
			(filter ':implicit matching #'(lambda (e) (< (to e) (num-nodes graph))))))))))


(defun perfectly-matchable-graph (g)
  "Given a bipartite graph G, make a new graph in which any matching can be extended to a perfect matching of the same weight."
  (mvbind (l r) (bipartite-components g)
    (let ((n1 (length l))
	  (n2 (length r)))
      (when (> n2 n1)
	(rotatef n1 n2)
	(rotatef l r))
      (flet ((num (n)
	       (cond
		((member? n l) (item-number n l))
		(t (+ n1 (item-number n r))))))
	(let* ((k (* n1 2))
	       (g2 (make-instance '<adjacency-matrix-graph> :nodes k :m (make-array (list k k) :initial-element nil))))
	  (do-elements (x l nil i)
	    (do-elements (y r nil j)
	      (add-edge g2 i (+ j n1) :label (aif (get-edge-between g x y) (label it) 0.0)))
	    (for-loop (j (+ n2 n1) (* 2 n1))
	      (add-edge g2 i j :label 0.0)))
	  (values g2 l r))))))
    


(defun bipartite-components (g)
  "bipartite-components G.  Return two values: the left and right components of bipartite graph G (l/r selected arbitrarily).  Error if G is not bipartite."
  (let ((comps (list (cons t nil) (cons nil nil)))
	(scanned (make-array (num-nodes g) :element-type 'boolean :initial-element nil)))
    (labels ((helper (node side)
	       (let ((i (node-number node g)))
		 (unless (aref scanned i)
		   (setf (aref scanned i) t)
		   (push node (cdr (assoc side comps)))
		   (do-elements (e (incident-edges g node))
		     (helper (if (equal node (to e)) (from e) (to e)) (not side)))))))
      (do-elements (n (nodes g) (values-list (mapcar #'cdr comps)))
	(helper n t)))))

  