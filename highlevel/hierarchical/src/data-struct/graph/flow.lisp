(in-package graph)

(defun edmonds-karp (g source sink)
  "edmonds-karp G SOURCE SINK.  Returns the maximum flow for G, represented as a 2D array, computed using the Edmonds-Karp version of the Ford-Fulkerson algorithm (at each step, push flow along the shortest augmenting path)."
  (let* ((n (size (nodes g)))
	 (a (make-array (list n n) :initial-element 0.0 :element-type 'float)))
    (loop
      (let* ((rn (residual-network g a))
	     (p (shortest-augmenting-path rn source sink)))
	(if p
	    (let ((c 'infty))
	      (mapl #'(lambda (l) (awhen (cadr l) (setf c (mymin c (label (get-edge rn (car l) it)))))) p)
	      (map nil #'(lambda (i j) (_f my+ (aref a (node-number i g) (node-number j g)) c)) p (rest p)))
	  (return a))))))




(defun residual-network (g f &aux (nodes (nodes g)) (n (size nodes)))
  "residual-network G F.  Return the residual network of G w.r.t. flow F.  This is a new graph over the nodes of G.  For any edge in G with current flow f and capacity c, there is an edge in G with capacity f-c, and a reverse edge with capacity f.  The returned graph is stored as an adjacency list.  The capacities are stored as the edge labels."
  
  (let ((weights (make-array n :initial-element nil)))
    (do-elements (n nodes nil i)
      (do-elements (e (edges-from g n))
	(let ((flow (evaluate-flow g f (from e) (to e)))
	      (capacity (label e))
	      (n2 (to e)))
	  (when (my> flow 0)
	    (let ((j (node-number n2 g)))
	      (push (cons n flow) (aref weights j))))
	  (when (my< flow capacity)
	    (push (cons n2 (my- capacity flow)) (aref weights i))))))
    (make-instance '<adjacency-list-graph>
      :nodes nodes
      :labeled-adjacency-lists weights)))


(defun shortest-augmenting-path (rn source sink &aux (nodes (nodes rn)) (n (size nodes)))
  ;; TODO: implement general purpose graph shortest path algorithm and use for this
  (let ((scanned (make-array n :initial-element nil))
	(bp (make-array n :initial-element nil))
	(q (queue:make-queue (list (node-number source rn))))
	(l (node-number source rn))
	(k (node-number sink rn)))
    (setf (aref scanned l) t)
    (loop
      (if (queue:queue-empty q)
	  (return nil)
	(let ((i (queue:dequeue q)))
	  (do-elements (e (edges-from rn (item i nodes)))
	    (let ((j (node-number (to e) rn)))
	      (unless (aref scanned j)
		(queue:enqueue j q)
		(setf (aref scanned j) t
		      (aref bp j) i))
	      (when (= j k)
		  (return-from shortest-augmenting-path
		    (let ((path (list k)))
		      (until (= (car path) l)
			(push (aref bp (car path)) path))
		      (mapcar #'(lambda (i) (item i nodes)) path)))
		))))))))



(defun evaluate-flow (g f from to)
  (let ((i (node-number from g))
	(j (node-number to g)))
    (ecase (array-rank f)
      (1 (aif (assoc to (aref f i) :test #'eq) (cdr it) 0))
      (2 (aref f i j)))))
  