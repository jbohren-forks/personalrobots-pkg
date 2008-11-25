(defpackage union-find
  (:documentation "Package union-find

Code for the union-find data structure.

Operations
----------
make-union-find
make-set
combine
get-comp
all-comps

Components returned by make-set, combine, all-comps, and get-comp are sets, and the operations member? and iteration (mapset, do-elements) are defined on them.")


  (:export
   make-union-find
   make-set
   combine
   get-comp
   all-comps)
   
  (:use 
   cl
   tree
   set
   utils)
  )

(in-package union-find)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Data structure for each node in the union-find tree
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <uf-node> (<node>)
  ((rank :accessor rank :initarg :rank)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code for allowing such a tree to be conveniently
;; interpreted as a set of items
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun items (tree)
  (ndlet ((n (nodes tree)))
    (node-label n)))

(defmethod iterator ((s <uf-node>))
  (iterator (items s)))

(defmethod member? (x (s <uf-node>))
  (member? x (items s)))

(defmethod is-empty ((s <uf-node>))
  nil)

(defmethod size ((s <uf-node>) &optional (constant-time nil))
  (if constant-time
      ':unknown
    (num-nodes s)))

(defmethod equality-test ((s <uf-node>))
  (error "equality test for union find set ~a unknown" s))

(defun pprint-component (&rest args)
  (bind-pprint-args (str comp) args
    (pprint-set str comp)))

(set-pprint-dispatch '<uf-node> #'pprint-component)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The main union find type
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 
(defstruct (union-find (:conc-name nil) (:constructor create-union-find))
  test
  sets
  table)

(defun make-union-find (&key (test #'equal))
  "make-union-find &key (TEST #'equal).
Return an empty union-find.  TEST is used to compare item labels."
  (create-union-find :test test :sets nil :table (make-hash-table :test test)))

(defun make-set (l uf)
  "make-set ITEM UNION-FIND.  Add a new set to the union-find, containing a single element ITEM (which must not already exist in the union-find).  Return the new component"
  (let ((tree (make-instance '<uf-node>
	       :child-edges nil
	       :node-label l
	       :rank 0)))
    (push tree (sets uf))
    (setf (gethash l (table uf)) tree)
    tree))


(defun combine (l1 l2 uf)
  "combine ITEM1 ITEM2 UNION-FIND.  Merge the components containing the ITEM1 and ITEM2 and return the new component.  If they're already in the same component, do nothing."
  (let ((comp1 (get-comp l1 uf))
	(comp2 (get-comp l2 uf)))
    (if (eq comp1 comp2)
	comp1
      (let ((rank1 (rank comp1))
	    (rank2 (rank comp2))
	    new-comp comp-to-delete)
	(cond
	 ((> rank1 rank2) 
	  (add-subtree comp1 nil comp2)
	  (setf new-comp comp1
		comp-to-delete comp2))
	 ((< rank1 rank2) 
	  (add-subtree comp2 nil comp1)
	  (setf new-comp comp2
		comp-to-delete comp1))
	 (t
	  (add-subtree comp1 nil comp2)
	  (incf (rank comp1))
	  (setf new-comp comp1
		comp-to-delete comp2)))
	(setf (sets uf) (delete comp-to-delete (sets uf)))
	new-comp))))

(defun get-comp (item uf)
  "get-comp ITEM UNION-FIND.  Return the component containing ITEM."
  (let* ((path (get-path-from-root (item-node item uf)))
	 (root (car path)))
    ;; Also flatten the tree by making all nodes on the path to this item children of the root
    (dolist (n (cddr path) root)
      (add-subtree root nil (remove-subtree n)))))

(defun all-comps (uf)
  "Return list of components.  This operation is O(num-items)."
  (let ((comps nil))
    (maphash
     #'(lambda (k v)
	 (declare (ignore v))
	 (adjoinf comps (get-comp k uf)))
     (table uf))
    comps))

(defun item-node (item uf)
  "Look up node corresponding to this item in the union-find"
  (check-not-null (gethash item (table uf)) "Union find node for ~a" item))
