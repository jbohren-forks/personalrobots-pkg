(defpackage graph
  (:documentation "Package graph

Types
-----
<adjacency-list-graph>
<adjacency-matrix-graph>
<union-find-graph>

Accessors
---------
get-edge
nodes
edges-from
edges-to
source
dest
incident-edges
add-edge
add-node
edge-label
neighbours
parents
children

Operations
----------
topological-sort
descendant-matrix
unweighted-shortest-path
max-weight-bipartite-matching
edmonds-karp
connected-component
connected-components
track-connected-components

pprint-graph

Conditions
----------
unknown-node
")
  (:export
   <adjacency-list-graph>
   <adjacency-matrix-graph>
   <union-find-graph>

   get-edge
   nodes
   edges-from
   edges-to
   source
dest
   incident-edges
   add-edge
   add-node
   edge-label
   neighbours
   parents
   children
   
   topological-sort
   descendant-matrix
   unweighted-shortest-path
   max-weight-bipartite-matching
   edmonds-karp
   connected-component
   connected-components
   track-connected-components
   
   pprint-graph
   
   unknown-node
   )
  (:use
   cl
   union-find
   utils
   set)
  )

(in-package graph)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; edges
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (edge (:conc-name nil)
	    (:constructor make-edge (from to &key label directed?)))
  from
  to
  (label nil)
  (directed? t))

(defun source (e) (from e))
(defun dest (e) (to e))

;; Seems unnecessary... might as well just use the function #'label defined in the structure
(defun edge-label (e) (label e))
(defun set-edge-label (e v) (setf (label e) v))
(defsetf edge-label set-edge-label)

(defun reverse-edge (e)
  (let ((e2 (copy-edge e)))
    (rotatef (from e2) (to e2))
    e2))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; conditions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(define-condition unknown-node (error)
  ((id :reader id :initarg :id)
   (graph :reader graph :initarg :graph))
  (:report (lambda (c s) (format s "Graph ~a does not have a node called ~a"
				 (graph c) (id c)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic operations on graphs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric nodes (g)
  (:documentation "nodes GRAPH.  Return the set of nodes in the graph."))

(defun node-number (n g)
  (handler-case 
      (item-number n (nodes g))
    (unknown-node ()
      (error 'unknown-node :id n :graph g))))

(defgeneric get-edge (g from to)
  (:documentation "get-edge GRAPH FROM TO.  If there's at least one edge between FROM and TO, return one of them.  Otherwise, return nil.  Might signal an unknown-node error.")
  (:method (g from to)
	   (unless (member? to (nodes g)) (error 'unknown-node :id to :graph g))
	   (find-element (edges-from g from) #'(lambda (x) (eq (to x) to)))))


(defun get-edge-between (g n1 n2)
  "get-edge-between G N1 N2.  If there's at least one edge between N1 and N2 (in either direction), return one of them, else return nil.   Might signal an unknown-node error."
  (or (get-edge g n1 n2) (get-edge g n2 n1)))

(defgeneric edges-from (g from)
  (:documentation "edges-from GRAPH FROM.  Return the set of edges emanating from FROM.  Might signal an unknown-node error."))

(defgeneric edges-to (g to)
  (:documentation "edges-to GRAPH TO.  Return the set of edges coming into TO.  Might signal an unknown-node error."))

(defgeneric incident-edges (g n)
  (:documentation "incident-edges GRAPH NODE.  Return the set of edges whose from or to field is NODE.  This is the way to get edges connected to a node for undirected graphs.  Might signal an unknown error.")
  (:method (g n) (disjoint-union (edges-from g n) (edges-to g n))))

(defun neighbours (g n)
  "neighbours GRAPH NODE.  Return set of (undirected) neighbours of NODE."
  (disjoint-union 
   (ndlet ((e (edges-from g n))) (to e))
   (ndlet ((e (edges-to g n))) (from e))))

(defun parents (g n)
  "parents GRAPH NODE.  Return implicitly specified set of nodes that are sources of edges whose destination is NODE."
  (ndlet ((e (edges-to g n))) (from e)))

(defun children (g n)
  "children GRAPH NODE.  Return implicitly specified set of nodes that are destinations of edges whose source is NODE."
  (ndlet ((e (edges-from g n))) (to e)))

(defun num-nodes (g)
  "num-nodes GRAPH.  Return the number of nodes in the graph."
  (size (nodes g)))

(defgeneric add-edge (g from to &key label directed?); (label nil) (directed? t))
  (:documentation "add-edge G FROM TO &key (LABEL nil) (DIRECTED? t).  Returns a graph having a new edge from FROM to TO.  If G is of a type that was created using defclass (as opposed to, say, some potential future use of integers or symbols to denote particular graphs), then that graph is guaranteed to be modified, so you can then do (add-edge g 'foo 'bar) instead of (setf g (add-edge f 'foo 'bar))")
  (:method (g from to &key (label nil) (directed? t))
	   (assert directed?)
	   (add-existing-edge g (make-edge from to :label label :directed? directed?))))



(defgeneric add-existing-edge (g e)
  (:documentation "add-existing-edge G EDGE.  See add-edge."))

(defgeneric add-node (g label)
  (:documentation "add-node G LABEL.  Returns 1) the new node 2) a graph which is like G except that there's a new node with the given LABEL (which must not already label a node in G).  If G is in a class, then it is destructively modified."))
  
  
     
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric pprint-graph (str g)
  (:documentation "pprint-graph STREAM GRAPH.  Pretty printing function for graphs.")
  (:method (str g)
	   (pprint-logical-block (str nil :prefix "[" :suffix "]")
	     (format str "Graph~2I")
	     (set:do-elements (n (nodes g) nil i)
	       (pprint-pop)
	       (format str "~:@_~w -> " n)
	       (pprint-logical-block (str nil)
		 (do-elements (e (edges-from g n) nil j)
		   (pprint-pop)
		   (format str "~:[~:@_~;~]~w~:[~; with label ~:*~w~]"
			   (zerop j) (to e) (label e))))))))

