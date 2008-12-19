(in-package graph)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class def
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <adjacency-matrix-graph> ()
  ((nodes :accessor nodes :initarg :nodes)
   (m :accessor m))
  (:documentation "<adjacency-matrix-graph>

Initargs
:nodes - set of nodes
OR
:node-test - a test, in which case, nodes will be an empty <indexed-set> using equality test test.

:m - a 2d array representing the adjacencies.  M(i,j) = nil means no edge from i to j.  If it is an object of type edge, then that is the edge from i to j.  Otherwise, an edge object is created from i to j having M(i,j) as the label.  Can be left out, in which case an all-nil array will be used.  Even if M is provided, the graph doesn't keep a reference to it, so future modifications to M will not affect the graph."))

(defmethod initialize-instance :after ((g <adjacency-matrix-graph>) &rest args &key (m nil) node-test nodes)
  (declare (ignore args))
  (assert (xor nodes node-test))
  (unless (slot-boundp g 'nodes)
    (setf (nodes g) (make-instance '<indexed-set> :test node-test)))
  (let ((nodes (nodes g)))
  (let ((n (size nodes))
	(table (setf (m g) (make-inf-array :rank 2 :default-val nil))))
    ;; The table is stored as an inf-array data structure, to make adding nodes constant-time
    (when m
      (dotimes (i n)
	(dotimes (j n)
	  (let ((e (aref m i j)))
	    (when e
	      (setf (inf-aref table i j)
		(cond
		 ((edge-p e) (assert (and (eq (from e) (item i nodes)) (eq (to e) (item j nodes)))) e)
		 (t (make-edge (item i nodes) (item j nodes) :label e))))))))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; methods
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod edges-from ((g <adjacency-matrix-graph>) n)
  (let ((i (node-number n g))
	(m (m g)))
    (ndlet-fail ((j (num-nodes g)))
      (or (inf-aref m i j) 'fail))))

(defmethod edges-to ((g <adjacency-matrix-graph>) n)
  (let ((i (node-number n g))
	(m (m g)))
    (ndlet-fail ((j (num-nodes g)))
      (or (inf-aref m j i) 'fail))))

(defmethod get-edge ((g <adjacency-matrix-graph>) n1 n2)
  (inf-aref (m g) (node-number n1 g) (node-number n2 g)))

(defmethod add-existing-edge (g e)
  (let ((i (node-number (from e) g))
	(j (node-number (to e) g))
	(m (m g)))
    (assert (not (inf-aref m i j)))
    (setf (inf-aref m i j) e)))

(defmethod add-node (g l)
  (addf (nodes g) l t)
  (values l g))
  
    
  
(set-pprint-dispatch '<adjacency-matrix-graph> #'pprint-graph)  
