(defpackage tree
  (:documentation "Package containing a simple implementation of trees in which nodes and edges can be labelled.

The way it works is that nodes and edges are objects.  Nodes contain links to their incoming and outgoing edges, and edges contain links to their head and tail nodes.  Nodes are also used to refer to the subtree they head.  In particular, operations on a tree are given the root node of the tree.


Node constructor and accessors
------------------------------
<node>
child-edges
parent-edge
node-label
parent-node
root?

Edge constructor and accessors
------------------------------
<edge>
head
tail
edge-label

Consistency checking
--------------------
is-inconsistent-tree
not-root
cycle
incorrect-parent-edge
incorrect-child-edge

Basic Operations
----------------
get-root
get-child
children
get-parent
get-child-edge-by-label
get-descendant
depth
get-path-from-root
add-new-child
add-subtree
remove-subtree
remove-subtree-below
leaf?
num-nodes
get-highest-ancestor

Iteration
---------
preorder-iterator
do-preorder
map-preorder
postorder-iterator
do-postorder
map-postorder

Sets
----
leaves
nodes

Debugging
---------
pprint-tree
move-child
move-par
*current-node*
*edge-label-pprint-key*
*subtree-pprint*
")

  (:use cl
	set
	utils)
  (:export
   <node>
   child-edges
   parent-edge
   parent-node
   node-label
   node
   root?
   
   <edge>
   head
   tail
   edge-label
   edge
   
   is-inconsistent-tree
   is-consistent-tree
   cycle
   incorrect-child-edge
   incorrect-parent-edge
   not-root
   
   get-root
   get-child
   children
   get-parent
   get-child-edge-by-label
   get-descendant
   depth
   get-path-from-root
   
   add-new-child
   add-subtree
   remove-subtree
   remove-subtree-below
   leaf?
   num-nodes
   get-highest-ancestor
   
   copy-subtree
   
   preorder-iterator
   do-preorder
   map-preorder
   postorder-iterator
   do-postorder
   map-postorder
   
   leaves
   nodes
   
   move-child
   move-par
   *current-node*
   pprint-tree
   *edge-label-pprint-key*
   *subtree-pprint*
   ))

(in-package tree)
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; data structures
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <node> ()
  ((child-edges :accessor child-edges :initarg :child-edges :initform nil)
   (parent-edge :accessor parent-edge :initarg :parent-edge :initform nil)
   (label :accessor node-label :initarg :node-label :initform nil))
  (:documentation "Class <node>

Represents a node in a tree.
Initargs
:child-edges (default nil)
:parent-edge (default nil)
:node-label (default nil)"))

(defmethod initialize-instance :after ((n <node>) &rest args &key parent)
  (declare (ignore args parent)))

(defclass <edge> ()
  ((edge-label :accessor edge-label :initarg :edge-label :initform nil)
   (head :initarg :head :accessor head)
   (tail :initarg :tail :accessor tail))
  (:documentation "Class <edge>

Represents an edge in a tree.
Initargs
:head
:tail
:edge-label (default nil)"))

(defun parent-node (node)
  (tail (check-not-null (parent-edge node) "parent edge of ~a" node)))

(defvar *current-node* nil)
(defvar *max-print-depth* nil)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; conditions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-condition nonexistent-child ()
  ((node :initarg :node :reader node)
   (num :initarg :num :reader num))
  (:report (lambda (c s) (format s "Node ~a does not have child #~a"
				 (node c) (num c)))))
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun is-consistent-tree (node &optional (is-root t))
  "is-consistent-tree TREE &optional (IS-ROOT t) ==> boolean
Returns the negation of is-inconsistent-tree."
  (not (is-inconsistent-tree node is-root)))

(defun is-inconsistent-tree (node &optional (is-root t))
  "is-inconsistent-tree TREE &optional (IS-ROOT t) ==> generalized-boolean
Is TREE an inconsistent tree.  Checks
1. does every child edge of a node have that node as their tail?
2. Does the parent edge of a node have the node's parent as head
3. Are there any undirected cycles?
4. If is-root, then is TREE the root node (i.e. not having a parent?)

If the tree is consistent, returns nil.  Otherwise, returns one of 'cycle, 'not-root, 'incorrect-parent-edge, 'incorrect-child-edge"

  (let ((nodes-seen nil))

    (labels
	((make-sure-that (c problem)
	   (unless c (return-from is-inconsistent-tree problem)))
	   
	 (check-recursively (n p)
	   
	   ;; make sure node hasn't been seen already (cycle)
	   (make-sure-that (not (member n nodes-seen)) 'cycle)
	   (push n nodes-seen)
	     
	   ;; make sure parent is as expected
	   (let ((p-edge (parent-edge n)))
	     (when p
	       (make-sure-that
		(and p-edge
		     (member p-edge (child-edges p)))
		'incorrect-parent-edge
		)))
	     
	   (let ((children (child-edges n)))
	       
	     ;; make sure child edges have n as tail
	     (make-sure-that 
	      (every (lambda (e) (eq n (tail e))) children)
	      'incorrect-child-edge)
	     
	     ;; call recursively on children
	     (dolist (e (child-edges n))
		(check-recursively (head e) n)))))
    
      ;; if required, ensure that node is root, i.e. par is nil
      (when is-root (make-sure-that (root? node) 'not-root))

      ;; initiate recursion
      (check-recursively node (awhen (parent-edge node) (tail it)))
      
      ;; return nil if no problems encountered
      nil)))	     


	     

(defun add-new-child (node edge-label child-label &key (node-initargs nil))
  "add-new-child NODE EDGE-LABEL CHILD-LABEL &key NODE-INITARGS  Create a new edge whose tail is NODE and whose head is a newly created node (which is returned).  

The new child will of the same subclass of <node> as NODE, and passed the :parent initarg.  Any additional initialization arguments can be passed using NODE-INITARGS."


  (let* ((child (apply #'make-instance (class-of node) :node-label child-label :parent node node-initargs))
	 (edge (make-instance '<edge> :tail node :head child :edge-label edge-label)))
    
    (setf (parent-edge child) edge)
    (setf (child-edges node) (insert-at-position (child-edges node) edge nil))
    child))

(defun get-child (node i)
  "get-child NODE I.  Get the Ith child of NODE."
  (let ((e (nth i (child-edges node))))
    (unless e
      (error 'nonexistent-child :node node :num i))
    (head e)))

(defun children (node)
  "children NODE.  Return list of children of NODE."
  (map 'list #'head (child-edges node)))


(defun get-ancestor (node i)
  "get-ancestor NODE I.  Return the Ith ancestor of NODE (where the 0th ancestor is NODE, the first ancestor is its parent, and so on).    Might throw a 'nonexistent-ancestor exception if not enough ancestors exist."
  (do ((current node (get-parent current))
       (l 0 (incf l)))
      ((>= l i) current)
    (unless current
      (error 'nonexistent-ancestor :node node :i i :depth l))))

(defun get-parent (node)
  "get-parent NODE.  Return the parent of NODE or nil if it doesn't have one."
  (awhen (parent-edge node)
	 (tail it)))

(defun get-descendant (node label-seq &key (key #'identity))
  "get-descendant NODE LABEL-SEQ &key (KEY #'identity).  Starting from NODE, follow a path towards the leaves with labels LABEL-SEQ and return the resulting node.  Return nil if there's no such path.  KEY is applied to the edge labels before comparison."
  (loop
      with current = node
      for l in label-seq
      for e = (get-child-edge-by-label current l :key key)
	      
      unless e 
      do (return nil)
	 
      do (setf current (head e))
      finally (return current)))

(defun depth (node)
  "depth NODE.  How many generations of ancestors does this node have
  (0 for root)."
  (do ((current node (get-parent current))
       (i -1 (incf i)))
      ((root? current) (1+ i))))

(defun get-path-from-root (node)
  "get-path-from-root NODE.  Return a list of the form (NODE_1 NODE_2 ... NODE_k) where NODE_k = NODE, NODE_1 is the root, and each NODE_{i+1} is a child of NODE_i."
  (do ((l (list node))
       (current node))
      ((root? current) l)
    (push (setf current (get-parent current)) l)))
      
  
	       

(defun get-child-edge-by-label (node label &key (key #'identity))
  "get-child-edge-by-label NODE LABEL &key KEY.  Return child edge with this label (tested using #'equal), or nil if there isn't one.  KEY is applied to edge labels before comparison."
  (find label (child-edges node) :key (lambda (x) (funcall key (edge-label x))) :test #'equal))

(defun get-root (tree)
  "get-root TREE.  Return the root node of TREE."
  ;; right now trees are represented by their root, so...
  tree)

(defun add-subtree (node edge-label tree &optional (position nil))
  "add-subtree NODE EDGE-LABEL TREE (POSITION nil).  Create a new outgoing edge from NODE with the given label.  Make the head of this edge be the root of TREE, and make the parent edge of the root be NODE.  POSITION is a nonnegative integer or nil - if it is nil it defaults to the current number of children of NODE.  The new child will be the POSITIONth (starting from the left) child of NODE."
  (let* ((root (get-root tree))
	 (edge (make-instance '<edge> :tail node :head root :edge-label edge-label))
	 (p-edge (parent-edge root)))
    (assert (null p-edge) ()
      "Can't add a subtree with root ~a that already has a parent ~a"
      root (tail p-edge))
    (setf (child-edges node) (insert-at-position (child-edges node) edge position))
    (setf (parent-edge root) edge))
  (values))


(defun remove-subtree-below (node edge)
  "remove-subtree-below NODE OUTGOING-EDGE.  Has the effect of 'cutting' OUTGOING-EDGE.  Remove OUTGOING-EDGE from the list of child edges of NODE.  Also, set the PARENT-EDGE of the head of OUTGOING-EDGE to nil.  Finally, return the now disconnected subtree."
  (deletef (child-edges node) edge)
  (let ((subtree-root (head edge)))
    (setf (parent-edge subtree-root) nil)
    subtree-root))

(defun remove-subtree (node)
  "remove-subtree NODE.  Remove the subtree headed by NODE from its containing tree and return it."
  (let ((e (parent-edge node)))
    (remove-subtree-below (tail e) e)))

(defun leaf? (node)
  "leaf? NODE.  True if it has no children."
  (null (child-edges node)))

(defun root? (node)
  "root? NODE.  True if iff its parent is null."
  (null (parent-edge node)))

(defun num-nodes (tree)
  "num-nodes TREE.  Number of nodes."
  (1+ (loop for e in (child-edges tree) summing (num-nodes (head e)))))

(defun get-highest-ancestor (node)
  "get-highest-ancestor NODE.  Return the root of the tree containing NODE by following parent edges repeatedly."
  (if (root? node)
      node
    (get-highest-ancestor (get-parent node))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; iteration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun postorder-iterator (tree)
  "postorder-iterator TREE. Return an iterator over the nodes of TREE, i.e. a function of no arguments which returns two values, NEXT-NODE and DONE."
  (let ((stack1 (list (list (get-root tree))))
	(stack2 nil))
    (lambda ()
      (while (car stack1)
	(let ((n (pop (car stack1))))
	  (push n stack2)
	  (push (children n) stack1)))
      (pop stack1)
      (if stack2
	  (values (pop stack2) nil)
	(values nil t)))))
	

(defmacro do-postorder ((var tree &optional result-form num-var) &rest body)
  "do-postorder (VAR TREE &optional RESULT-FORM NUM-VAR) &rest BODY.
Repeatedly execute BODY with VAR bound to the successive nodes of tree in postorder.  Finally return RET-VAL."
  `(do-iterator (postorder-iterator ,tree) (,var ,result-form ,num-var) ,@body))

(defgeneric map-postorder (result-type fn tree)
  (:documentation "map-postorder RESULT-TYPE FN TREE.  Iterate over the nodes of TREE in postorder and apply FN to each one, collecting the results into an object of type RESULT-TYPE (which is either 'list or 'vector).")
  (:method ((result-type (eql 'list)) fn tree)
	   (map-iterator-to-list fn (postorder-iterator tree)))
  (:method ((result-type (eql 'vector)) fn tree)
	   (let ((v (make-array (num-nodes tree))))
	     (do-postorder (x tree v i)
	       (setf (aref v i) (funcall fn x))))))



(defun preorder-iterator (tree)
  "preorder-iterator TREE.  Return an iterator over the nodes of TREE, i.e. a function of no arguments which returns two values, NEXT-NODE and DONE."
  (let ((stack nil))
    (push (list (get-root tree)) stack)
    (lambda ()
      (if stack
	  (let ((next (pop (car stack))))
	    (when (null (car stack))
	      (pop stack))
	    (awhen (children next)
		   (push it stack))
	    (values next nil))
	(values nil t)))))

(defmacro do-preorder ((var tree &optional result-form num-var) &rest body)
  "do-preorder (VAR TREE &optional RESULT-FORM NUM-VAR) &rest BODY.
Repeatedly execute BODY with VAR bound to the successive nodes of tree in preorder.  Finally return RET-VAL."
  `(do-iterator (preorder-iterator ,tree) (,var ,result-form ,num-var) ,@body))

(defgeneric map-preorder (result-type fn tree)
  (:documentation "map-preorder RESULT-TYPE FN TREE.  Iterate over the nodes of TREE in preorder and apply FN to each one, collecting the results into an object of type RESULT-TYPE (which is either 'list or 'vector).")
  (:method ((result-type (eql 'list)) fn tree)
	   (map-iterator-to-list fn (preorder-iterator tree)))
  (:method ((result-type (eql 'vector)) fn tree)
	   (let ((v (make-array (num-nodes tree))))
	     (do-preorder (x tree v i)
	       (setf (aref v i) (funcall fn x))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; copying
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun copy-subtree (n)
  "copy-subtree N.  Return a copy of N that shares no structure with it.  The node and edge labels are copied using clone. The only difference is that the node N's copy will not have a parent, even if N has one. Note that this function won't work if the nodes and edges are not actually of classes <node> and <edge>."
  (labels ((copy-subtree-rec (node par-edge)
	     (let ((new-node (make-instance '<node>
			       :node-label (clone (node-label node)) :parent-edge par-edge )))
	       (setf (child-edges new-node)
		 (mapcar
		  #'(lambda (e)
		      (let ((new-e (make-instance '<edge> :edge-label (clone (edge-label e)) :tail new-node :head nil)))
			(setf (head new-e) (copy-subtree-rec (head e) new-e))
			new-e))
		  (child-edges node)))
	       new-node)))
    (copy-subtree-rec n nil)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sets of nodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun leaves (tree)
  "leaves TREE.  Implicit representation of set of leaves of tree."
  (if (leaf? tree)
      (list tree)
    (set:ndunion ((e (child-edges tree)))
      (leaves (head e)))))

(defun nodes (tree)
  "nodes TREE.  Implicit representation of set of nodes of tree."
  (set:disjoint-union 
   (list tree)
   (set:ndunion ((e (child-edges tree)))
     (nodes (head e)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; printing, debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun pprint-edge (str e)
  (pprint-logical-block (str nil)
    (format str "(Edge ~a " (edge-label e))
    (pprint-newline :fill str)
    (format str "from ~a " (node-label (tail e)))
    (pprint-newline :fill str)
    (format str "to ~a" (node-label (head e)))))


(defun pprint-node (str n)
  (pprint-logical-block (str nil)
    (format str "<<Node ~a " (node-label n))
    (pprint-newline :fill str)
    (let ((p (parent-edge n)))
      (when p
	(format str "Parent ~a " (node-label (tail p)))
	(pprint-newline :fill str)))
    (format str " with child labels ")
    (pprint-fill str (map 'list #'edge-label (child-edges n)))))




(defun move-par ()
  "move-par 
Move current node up tree."
  (if *current-node*
      (aif (get-parent *current-node*)
	   (setf *current-node* it)
	   (warn "Node ~a does not have a parent." (node-label *current-node*)))
    (warn "*current-node* is nil."))
  *current-node*)

(defun move-child (i)
  "move-child I
Move to child I of current node."
  (if *current-node*
      (handler-case 
	  (setf *current-node*
	    (get-child *current-node* i))
	(nonexistent-child ()
	  (warn "Child ~a does not exist" i)))
    (warn "*current-node* is nil"))
  *current-node*)



(defvar *edge-label-pprint-key* #'identity)
(defvar *subtree-pprint* (constantly t))

(defun pprint-tree (&rest args)
  "pprint-tree TREE.  Can customize the behaviour of this using the variables *pprint-tree-edge-label-key* and *pprint-tree-node-label-key*"
  (bind-pprint-args (str node) args
    (let ((prob (is-inconsistent-tree node nil)))
      (assert (not prob) ()
	"Cannot print tree rooted at ~a as it is not consistent for reason ~a"
	node prob))
    (labels ((pprint-tree-rec (str node)
	       (pprint-logical-block (str nil :prefix "[" :suffix "]")
		 (format str "~a~:[~;~:@_Parent edge: ~:*~a~]" 
			 node
			 (when (parent-edge node) (funcall *edge-label-pprint-key* (edge-label (parent-edge node)))))
		 (if (funcall *subtree-pprint* (node-label node))
		 (set:do-elements (e (child-edges node) nil i)
		   (pprint-pop)
		   (format str "~:@_~:[~:@_~;~]Child ~a: " (zerop i) i)
		   (pprint-tree-rec str (head e)))
; 		   (let ((child (head e)))
; 		     (if (funcall *subtree-pprint* (node-label child))
; 			 (pprint-tree-rec str (head e))
; 		       (format str "omitted"))))
		 (format str "~:@_Children omitted.")))))
      (pprint-tree-rec str node))))

(defmethod print-object ((n <node>) str)
  (print-unreadable-object (n str :type t :identity nil)
    (format str "~a" (node-label n))))