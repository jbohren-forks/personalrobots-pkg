(in-package graph)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Class def
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <adjacency-list-graph> ()
  ((nodes :accessor nodes :initarg :nodes)
   (adjacency-lists :accessor adjacency-lists)
   (reverse-adjacency-lists :accessor rev-adj-lists))
  (:documentation "Stores a graph using adjacency lists.

Initargs
:nodes - [numbered-set] of node ids (must be unique).  Optional, and defaults to using an empty indexed set, with elements compared using :node-test if provided, or #'eq otherwise.

:adjacency-lists - a vector, where the Ith element is the adjacency list for the ith node.  Each adjacency list is a list of node ids OR
:labeled-adjacency-lists - a vector, where the ITH element is a weighted adjacency list of elements of the form (NODE . LABEL)
If neither is provided, assume no edges.
"))



(defmethod initialize-instance :after ((g <adjacency-list-graph>) &rest args 
				       &key adjacency-lists labeled-adjacency-lists (node-test #'eq)
				       &aux (n (length (or adjacency-lists labeled-adjacency-lists))))
  (declare (ignore args))
  (set-if-unbound 'nodes g (make-instance '<indexed-set> :test node-test))
  (assert (not (and adjacency-lists labeled-adjacency-lists)))
  
  (when (not labeled-adjacency-lists)
    (orf adjacency-lists (vector)))
  (set-if-unbound 'nodes g n)
  (let ((v (setf (adjacency-lists g) (make-array n :fill-pointer 0 :adjustable t)))
	(rv (setf (rev-adj-lists g) (make-array n :fill-pointer n :adjustable t :initial-element nil)))
	(nodes (nodes g)))
    (if adjacency-lists
	(dotimes (i n)
	  (let ((source (item i (nodes g)))
		(l (aref adjacency-lists i)))
	    (vector-push (mapcar #'(lambda (dest) (make-edge source dest)) l) v)))
      (dotimes (i n)
	(let ((source (item i (nodes g)))
	      (l (aref labeled-adjacency-lists i)))
	  (vector-push 
	   (mapcar #'(lambda (item) (destructuring-bind (dest . label) item (make-edge source dest :label label))) l) v))))
    (map nil 
      #'(lambda (l)
	  (dolist (e l)
	    (let ((j (item-number (to e) nodes)))
	      (push e (aref rv j)))))
      (adjacency-lists g))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; accessors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod edges-from ((g <adjacency-list-graph>) from-id)
  (handler-case 
      (aref (adjacency-lists g) (node-number from-id g))
    (item-not-in-set ()
      (error 'unknown-node :id from-id :graph g))))

(defmethod edges-to ((g <adjacency-list-graph>) to)
  (handler-case
      (aref (rev-adj-lists g) (node-number to g))
    (item-not-in-set ()
      (error 'unknown-node :id to :graph g))))


(defmethod add-existing-edge ((g <adjacency-list-graph>) e)
  (push e (aref (adjacency-lists g) (node-number (from e) g)))
  (push e (aref (rev-adj-lists g) (node-number (to e) g))))

(defmethod add-node ((g <adjacency-list-graph>) l)
  (addf (nodes g) l t)
  (vector-push-extend nil (adjacency-lists g))
  (vector-push-extend nil (rev-adj-lists g))
  (values l g))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; printing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(set-pprint-dispatch '<adjacency-list-graph> #'pprint-graph)
  
