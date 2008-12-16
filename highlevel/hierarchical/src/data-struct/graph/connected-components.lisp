(in-package graph)

(defclass <union-find-graph> ()
  ((union-find-components :accessor union-find-components :initform nil))
  (:documentation "A graph that keeps track of its (weakly) connected components."))

(defmethod initialize-instance :around ((g <union-find-graph>) &rest args)
  (declare (ignore args))
  (call-next-method)
  (recompute-comps g))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Exported ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun track-connected-components (g)
  "track-connected-components G.  Compute the connected components of G and, from now on, transparently keep track of them after each add-node and add-edge operation.  The connected-component operation can be used to get the components, each of which can be treated as a set of nodes.

Works fine so long as G is an instance of standard-class, and G's original class does not have a slot or method called graph::union-find-components."
  (unless (typep g '<union-find-graph>)
    (change-class g (get-mixin-class (class-name (class-of g)) '<union-find-graph>))))

(defun connected-component (n g)
  "connected-component N G.  Return the connected component corresponding to node label N in G.  The returned object is a <set>, and iteration and membership queries can be called on it."
  (recompute-comps g)
  (get-comp n (union-find-components g)))

(defun connected-components (g)
  "connected-components G.  Return list of connected components."
  (all-comps (union-find-components g)))
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun recompute-comps (g)
  "If components of G are null, recompute them from scratch and return t.  Otherwise return nil."
  (check-type g <union-find-graph>)
  (unless (union-find-components g)
    (let ((comps (setf (union-find-components g) (make-union-find :test (equality-test (nodes g))))))
      (do-elements (n (nodes g))
	(make-set n comps))
      (do-elements (n (nodes g) t)
	(do-elements (e (edges-from g n))
	  (combine n (to e) comps))))))

(defmethod add-node :after ((g <union-find-graph>) l)
  (or (recompute-comps g) (make-set l (union-find-components g))))

(defmethod add-existing-edge :after ((g <union-find-graph>) e)
  (or (recompute-comps g) (combine (from e) (to e) (union-find-components g))))

  
  


