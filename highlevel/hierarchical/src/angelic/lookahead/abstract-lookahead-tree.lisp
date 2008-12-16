;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Contains operations for abstract lookahead trees.  The operations assume
;; that the hierarchy is preprocessed to have dummy Act and Finish 
;; actions (this can be done by using the preprocess flag when creating the
;; <abstract-planning-problem>).
;;
;; Debug levels
;; 3 - print tree add/delete ops
;; 4 - print subsumption checker state at various points, note updates
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Special variables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *prim-then-act* nil)
(defvar *known-values* nil)
(defvar *num-extensions* 0
  "Number of node extensions.  Incremented each time extend-node is called.  Should be initialized to 0 by any external algorithm that wants to count this.")
(defvar *num-progressions* 0
  "Like *num-extensions*, except only incremented when extend-node has to actually create a new node as opposed to activating an existing one.")
(defvar *num-refinements* 0
  "Number of refinements.  Incremented each time refine is called.  Should be initialized to 0 by any external algorithm that wants to count this.")
(defvar *upward-prop* nil) ;; Turning this off for now, because it results in occasional bugs without helping performance too much
(defvar *tree* nil
  "Lookahead algorithms may set this to the tree they're using to simplify debugging.")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Tree nodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <alt-node> (<node>)
  ((sound-valuation :accessor sound-valuation :initarg :sound-valuation :initform nil)
   (complete-valuation :accessor complete-valuation :initarg :complete-valuation :initform nil)
   (root :accessor root)
   (state :accessor state :initarg :state :initform nil)
   (subsumed :accessor subsumed :initarg :subsumed :initform nil)
   (status :accessor status :initarg :status :initform 'active)
   (action :accessor action :initarg :action :initform nil)
   (upward-parent :accessor upward-parent :initform nil)
   (downward-spans :accessor downward-spans :initform nil)
   (hpp :accessor hpp :initarg :hpp)
   (changed :accessor changed) 
   ;; Used only for internal communication of info between update methods
   ;; The right way to do this would be to define a new kind of method combination...
   
   (out-of-date-forward-flag :accessor out-of-date-forward-flag :initform t)
   ;; This is true iff the node is out-of-date w.r.t its parent (accomplished by calling
   ;; update-forward).  The reason we have a flag for this is that after we do, say, upward
   ;; propagation, making all the descendant nodes up to date immediately could be expensive
   ;; so we do it lazily using this flag.  
   
   (out-of-date-backward-flag :accessor out-of-date-backward-flag :initform t)
   ;; Backwards updates are not done lazily, so this will only be true during the add/remove path
   ;; operations
   
   (out-of-date-upward-flag :accessor out-of-date-upward-flag :initform nil)
   ;; Is the node up-to-date with respect to any refined versions?
   
   (known-value :accessor known-value :initarg :known-value :initform nil)))

(defmethod initialize-instance :after ((n <alt-node>) &rest args &key parent)
  (declare (ignore args))
  (setf (root n) (if parent (root parent) n))
  (when parent 
    (setf (hpp n) (hpp parent))
    (update-forward n parent)))


(defun edge-action (e)
  (action (head e)))

(defun sound-set (n)
  (reachable-set (sound-valuation n)))

(defun complete-set (n)
  (reachable-set (complete-valuation n)))

(defun sound-reward-upto (n)
  (max-achievable-value (sound-valuation n)))

(defun complete-reward-upto (n)
  (max-achievable-value (complete-valuation n)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Top-level tree operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun initial-tree (hpp &key (init-state (init-state hpp)) (prim-then-act nil) (known-values nil)
			      (initial-tree nil) (subsumption-checker (subsumption-checker hpp)) 
			      (node-type '<alt-node>) (node-initargs nil))
  "initial-tree HIERARCHICAL-PLANNING-PROBLEM &key (INIT-STATE (INIT-STATE HPP)) (PRIM-THEN-ACT nil) (KNOWN-VALUES nil) (INITIAL-TREE nil) (SUBSUMPTION-CHECKER (subsumption-checker HPP)) (NODE-TYPE nil) (NODE-INITARGS nil)
Returns the initial lookahead tree for this hierarchical planning problem.  The tree consists of a root node corresponding to the initial state, with a single outgoing edge corresponding to the dummy top-level HLA ACT.

NODE-TYPE, if left out, will default to the standard node type.  It can be specified here by algorithms that want to include additional information in the nodes.  It should be a subclass of <node>.  NODE-INITARGS are any additional initargs for the node-type."

  (let* ((s (make-state-set hpp init-state))
	 (val (make-simple-valuation s 0))
	 (root (or initial-tree
		   (apply 
		    #'make-instance
		    node-type :sound-valuation val :complete-valuation val :state init-state :hpp hpp node-initargs))))
    (setf (out-of-date-forward-flag root) nil)
    (if prim-then-act
	(do-elements (a (avail-actions hpp init-state) root)
	  (add-path-to-tree (list a 'act) root :known-values known-values :subsumption-checker subsumption-checker))
      (progn (add-path-to-tree '(act) root :known-values known-values) root))))
  

(defun refine (tree plan ind &key (known-values nil) (subsumption-checker (subsumption-checker (hpp tree))) 
	       &aux (path (get-plan-path tree plan)) (hpp (hpp tree)))
  "refine TREE PLAN IND &key (KNOWN-VALUES nil) (SUBSUMPTION-CHECKER (subsumption-checker (hpp tree)))
Add to the tree all valid immediate refinements at position IND of the action-sequence PLAN (refinements may be cut short if they reach a known state).  Return list of nonsubsumed final nodes of added plans.  Maintains well-formedness of tree.  If the action is primitive, is a noop.
SUBSUMPTION-CHECKER is any object for which add-entry and is-subsumed are implemented.  The default null value means no subsumption checking is done."
  (incf *num-refinements*)
  (if (is-primitive (aref plan ind) hpp)
      (debug-print 1 "Not refining ~a at position ~a as action is primitive." plan ind)
    (let ((hset (complete-set (aref path ind)))
	  (n (length path))
	  (ref-end-node (elt path (1+ ind)))
	  (new-leaf-nodes nil))
      (debug-print 1 "Refining ~a at pos ~a" plan ind)
      (debug-print 3 "~&Subsumption checker is ~a" subsumption-checker)
      (with-debug-indent 

	  (do-elements (r (applicable-refinements hpp (aref plan ind) hset))
	    (let ((new-plan (substitute-ref 'list plan ind r)))
	      (when (valid-plan? hpp new-plan)
	    (let ((node (add-path-to-tree (substitute-ref 'list plan ind r) tree 
					  :known-values known-values :subsumption-checker subsumption-checker
					  :update-backward nil)))
	  
	      ;; Unless new leaf node is subsumed or known, add vertical link between node at end of refinement and 
	      ;; node at end of original action
	      (unless (or (subsumed node) (known-value node))
		(let ((new-path (get-path-from-root node)))
		  (add-ref-link (elt new-path ind) (elt new-path (+ ind 1 (- (length new-path) n))) ref-end-node)))

	      (push node new-leaf-nodes)))))
      
	(setf (out-of-date-upward-flag ref-end-node) t)
      
	;; Do backward propagation 
	(let ((removed-node (remove-path-from-tree plan tree known-values :update-backward nil)))
	  (update-nodes-backward new-leaf-nodes known-values (elt path ind))
	  (when removed-node
	    (assert (not (root? removed-node)))
	    (update-nodes-backward (parent-node removed-node) known-values))

	  ;; We only upward propagate if 1) The original node is still active 2) None of the newly added nodes were subsumed or known
	  (unless (or (some (fn (not active)) (get-path-from-root ref-end-node))
		      (some (fn (or subsumed known-value)) new-leaf-nodes))
	    (upward-propagate ref-end-node known-values))
	
	  ;; Return unsubsumed new leaf nodes
	  (filter ':list new-leaf-nodes (fn (not subsumed))))))))
      
	     


(defun get-best-plan (tree plan-type &key (q-fn nil) (reward-fn nil))
  "get-best-plan TREE PLAN-TYPE &key (Q-FN nil) (REWARD-FN nil)

Get the best active plan in tree of type PLAN-TYPE (which must be one of ':primitive, ':begins-with-primitive, or ':any).  If Q-FN is provided (currently the only option), then the plan is found by following the nodes with the max value of Q-FN.  If there is at least one path, returns 1) the plan 2) its value 3) the path.  Otherwise, returns nil.

If REWARD-FN is provided, it is used to sum up the value of the best plan.  If not, the Q-value at the root is used.  But note that in the :primitive case, the above will often be wrong, because it will be based on a maximization over all plans."
  (assert q-fn)
  (assert (or reward-fn (not (eq plan-type ':primitive))))
  ;; Reason: there's a potential inconsistency otherwise, in the case where at each step,
  ;; the q-fn was defined by maximization over all (not necessarily primitive) children, 
  ;; so the returned primitive plan doesn't achieve the returned value
  ;; So, to get the true value, you need to have some way of summing rewards as you go.
  
  (let ((best-val 0.0)
	(node tree)
	(plan (make-array 0 :adjustable t :fill-pointer 0))
	(path (make-array 0 :adjustable t :fill-pointer 0)))
    (loop
      (let ((children
	     (if (or (eq plan-type ':primitive) 
		     (and (eq plan-type ':begins-with-primitive) (root? node)))
		 (filter ':implicit (active-child-nodes node) 
			 #'(lambda (n) (is-primitive (action n) (hpp n))))
	       (active-child-nodes node))))
	
	(vector-push-extend node path)
	(assert (not (out-of-date-backward-flag node)) nil
	  "In get-best-plan, reached an out-of-date-backward node at end of ~a" 
	  (get-path-from-root node))
	(unless (eq node tree) 
	  (vector-push-extend (action node) plan)
	  (when reward-fn (incf best-val (funcall reward-fn node))))
	
	(if (is-empty children)
	    (progn
	      (debug-print 3 "Best plan is ~a with value ~a" plan best-val)
	      (return (and (> (length path) 0) (values plan best-val path))))
	  (mvbind (ind val best) (argmax children :key q-fn)
	    (declare (ignore ind))
	    (when (and (eq node tree) (not reward-fn))
	      (setf best-val val))
	    (setf node best)))))))






;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Midlevel tree operations
;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun add-path-to-tree (actions root &key (known-values nil) (subsumption-checker nil) 
					   (update-backward t) &aux (actions (coerce actions 'list)))
  "add-path-to-tree ACTIONS ROOT &key (KNOWN-VALUES nil) (SUBSUMPTION-CHECKER nil) (UPDATE-BACKWARD t)
Causes a path corresponding to prefix A' of ACTIONS to be in the tree such that either A' = ACTIONS, or the last node in the path has known state or is subsumed.  Returns the new terminal node.  Maintains well-formedness and forward-up-to-dateness of tree.  When UPDATE-BACKWARD is true, also maintains backward-up-to-date-ness."

  (debug-print 2 "Adding path ~a to tree" actions)
  (with-debug-indent 
      (let ((node root))
	(while actions
	  (let ((child-edge (get-action-edge node (pop actions) t)) )
	    (setf node (head child-edge))
	
	    (cond
	     ;; 1. If child is subsumed, mark it subsumed, and stop adding further nodes
	     ;; TODO: This subsumption check only needs to be done if the node was newly added!
	     ((is-subsumed subsumption-checker (complete-valuation node))
	      (debug-print 3 "Subsumed")
	      (setf (subsumed node) t)
	      (return nil))

	 
	     ;; 2. If child is known, mark it known, and stop adding further nodes
	     ((setf (known-value node) 
		(awhen (state node)
		  (lookup-known-value 'state it 'known known-values)))
	      (debug-print 3 "Known")
	      (return nil))

	 
	     ;; 3. Otherwise, add it to the subsumption checker and added node list
	     (t (add-entry subsumption-checker (sound-valuation node))))))
    
	;; Make the tree up-to-date
	(when update-backward
	  (update-nodes-backward node known-values))
    
	;; Return the added node
	node)))

(defun remove-path-from-tree (actions root known-values &key (update-backward t)) 
  "remove-path-from-tree ACTIONS ROOT KNOWN-VALUES &key (UPDATE-BACKWARD t)
Find the highest node whose descendants are all members of this path, and make it inactive.  Return this node, or nil if there was no such node.
Maintains well-formedness and forward-up-to-dateness of tree.  Also maintains backward-up-to-dateness if update-backward is true."
  
  (debug-print 2 "Removing ~a from tree" actions)
  (with-debug-indent
      ;; Get nodes along this path in leaf-to-root order
      (let ((nodes (list root)))
	(do-elements (a actions)
	  (push (head (get-action-edge (first nodes) a)) nodes))
    
	;; Special case: if the lowest node on the path is actually not a leaf, do nothing
	(if (some #'(lambda (e) (eq (status (head e)) 'active)) (child-edges (first nodes)))
	    (progn
	      (debug-print 2 "Not removing any nodes of path ~a as it has active children" actions)
	      nil)
	      
    
	  ;; Figure out the highest node on the path that has more than one child
	  ;; If none do, we start at the root
	  (let ((i (or (position-if 
			#'(lambda (n) 
			    (>
			     (sum-over (child-edges n) #'(lambda (e) (indicator (eq (status (head e)) 'active))))
			     1))
			nodes)
		       (1- (length nodes)))))

	    ;; Remove the subtree by marking its top edge inactive
	    (let ((top-removed-node (elt nodes (1- i))))
	      (make-subtree-inactive top-removed-node :known-values known-values :update-backward update-backward)
	      top-removed-node))))))



(defun make-subtree-inactive (node &key (known-values nil) (update-backward t))
  "make-subtree-inactive NODE &key (UPDATE-BACKWARD t) (KNOWN-VALUES nil).  Make the subtree below NODE inactive.  Also do backward updates as necessary."
  (let ((unremoved-node (parent-node node)))
    (setf (status node) 'inactive
	  (out-of-date-backward-flag unremoved-node) t)
    (when update-backward
      (update-nodes-backward unremoved-node known-values))))

  


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Low-level tree operations
;;
;; A tree is well-formed if:
;; 1. If N2 is a child of N1, and has parent edge E  
;;    corresponding to action A, then:
;;    a) N2's complete valuation is the complete result of A from 
;;       N1's complete valuation, and likewise for the sound valuation
;;    b) If N1.state is non-nil and A is primitive, N2.state 
;;       is the successor state.  Otherwise, it's nil.
;; 2. The root node has non-nil state, and corresponding
;;    sound/complete valuations
;; 
;; Note that these operations don't keep the nodes
;; up-to-date (i.e. by doing backup operations).  
;; That is done by add and remove-path.
;; 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun get-action-edge (node action &optional (add-if-necessary nil) (follow-inactive nil))
  "get-action-edge NODE ACTION &optional (ADD-IF-NECESSARY nil) (FOLLOW-INACTIVE nil)

If there is an outgoing active edge from NODE whose action is #'equal to ACTION, return that edge.
Otherwise, if ADD-IF-NECESSARY is nil, return nil.
Otherwise, call extend-node to add new edge and child node, then return the edge.
Preserves well-formedness."
  (assert (not (and add-if-necessary follow-inactive)))
  (do-elements  (e (if follow-inactive (child-edges node) (active-children node)) (when add-if-necessary (parent-edge (extend-node node action))))
    (when (equal (edge-action e) action)
      (return e))))


(defun extend-node (node a)
  "extend-node NODE ACTION 
NODE - a node in an ALT
ACTION - the name of an action, such that there is not already an active edge from NODE corresponding to ACTION

If there is an inactive edge from NODE for ACTION, just make that edge active.  
Otherwise, add a new (active) child from NODE for ACTION.
In either case, return the new node.
Maintains the well-formedness and forward-updateness of the tree, and marks the parent node out-of-date backwards."
  
  (incf *num-extensions*)
  (setf (out-of-date-backward-flag node) t)
  (let ((e (find-element (child-edges node) #'(lambda (e) (equal (edge-action e) a)))))
    (if e
	;; If edge exists, make it active 
	(let ((child (head e)))
	  (assert (eq (status child) 'inactive) nil 
	    "Attempted to add ~a to ~a, but there is already an active edge ~a for this action."
	    a node e)
	  (setf (status child) 'active
		(subsumed child) nil
		(out-of-date-backward-flag child) t)
	  
	  ;; Make children of child inactive 
	  (do-elements (grandchild (children child) child)
	    (setf (status grandchild) 'inactive)))
      
      ;; Otherwise, create a new edge and child node.  Note that this will cause update-forward to get called.
      (progn 
	(incf *num-progressions*)
	(add-new-child node nil nil :node-initargs `(:parent ,node :action ,a))))))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Ops to keep tree up-to-date
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defgeneric update-forward (n parent)
  (:documentation "update-forward NODE PARENT. Ensure that info in NODE is up-to-date relative to its parent PARENT (note that at the point that this is called, NODE's parent edge field may not have been set yet).  A top-level :around method 1) checks if the out-of-date-forward-flag is true 2) If not, returns at once 3) Otherwise, calls the next method(s) 4) Sets the out-of-date flag to nil 5) If changes happened, make children out-of-date-forward and this node out-of-date-backward 6) Returns the set of changes that were noted by the child methods.  Can add :after methods for specific subclasses, which should push any changes onto (changed n)")
  (:method :around ((n <alt-node>) parent)
	   (declare (ignore parent))
	   (setf (changed n) nil)
	   (debug-print 3 "~:[Not updating forward as already up-to-date~;Updating forward~] ~a"
			(out-of-date-forward-flag n) n)
	   (when (out-of-date-forward-flag n)
	     (call-next-method)
	     (setf (out-of-date-forward-flag n) nil)
	     (when (changed n)
	       (with-debug-indent
		   (debug-print 3 "New value is ~a.~&Changes make children ~/set:pprint-set/ out of date" 
				n (ndlet ((c (children n))) (action c))))
	       (setf (out-of-date-backward-flag n) t)
	       ;; Note assymetry: forward-updating a node makes it backward-out-of-date, but not vice versa
	       (do-elements (c (children n))
		 (setf (out-of-date-forward-flag c) t))))
	   (prog1 (changed n)
	     (slot-makunbound n 'changed)))
  (:method ((n <alt-node>) parent &aux (a (action n)))
	   (with-slots (hpp state sound-valuation complete-valuation) parent
	     (let ((new-state (aand (is-primitive a hpp) state (succ-state it (primitive-action-description hpp a))))
		   (new-sv (progress-sound a sound-valuation hpp))
		   (new-cv (progress-complete a complete-valuation hpp)))
	       (when (change-if-necessary (state n) new-state #'same-state)
		 (push 'state (changed n)))
	       (when (change-if-necessary (sound-valuation n) new-sv #'equal-valuations)
		 (push 'sound-valuation (changed n)))
	       (when (change-if-necessary (complete-valuation n) new-cv #'equal-valuations)
		 (push 'complete-valuation (changed n)))))))
	   

(defgeneric update-backward (node known-values)
  (:documentation "update-backward NODE KNOWN-VALUES.  Ensure that info in NODE is up-to-date relative to its active children in the ALT, and wrt any information stored in KNOWN-VALUES.  A top-level around method 1) checks the out-of-date-backward flag 2) does nothing if its nil 3) otherwise, calls child methods 4) sets the backwards out-of-date flag to nil and sets parent out-of-date flag to t if changes happened 5) returns list of changes.  Can add :after methods for specific subclasses, which should push any changes onto (changed n).  Should be able to handle the following cases 1) root node 2) subsumed 3) known-value 4) leaf node corresponding to act or finish 5) internal nodes")
  (:method :around ((n <alt-node>) known-values)
	   (declare (ignore known-values))
	   (setf (changed n) nil)
	   (debug-print 3 "~:[Not updating backward as already up-to-date~;Updating backward~] ~a" 
			(out-of-date-backward-flag n) n)
	   (when (out-of-date-backward-flag n)
	     (call-next-method)
	     (setf (out-of-date-backward-flag n) nil)
	     (when (changed n)
	       (with-debug-indent
		   (debug-print 3 "New value is ~a.~&Changes make parent out of date" n))
	       (unless (root? n)
		 (setf (out-of-date-backward-flag (parent-node n)) t))))
	   (prog1 (changed n) (slot-makunbound n 'changed)))
  (:method ((n <alt-node>) known-values) (declare (ignore known-values)) nil))


(defgeneric update-upward (node)
  (:documentation "update-upward NODE.  Ensure that info in NODE is up-to-date relative to its downward children.  A top-level around method 1) checks the out-of-date-upward flag 2) does nothing if it's nil 3) Otherwise calls child methods 4) sets the out-of-date upward flag to nil and the forward and backwards out-of-date flags, and upward parent's upward out-of-date flags, to t if necessary. 5) Return list of changes.")
  (:method :around ((n <alt-node>))
	   (setf (changed n) nil)
	   (debug-print 3 "~:[~Not updating u~;Updating u~]pward ~a" (out-of-date-upward-flag n) n)
	   (with-debug-indent
	       (when (out-of-date-upward-flag n)
		 (call-next-method)
		 (setf (out-of-date-upward-flag n) t)
		 (when (changed n)
		   (awhen (upward-parent n)
		     (setf (out-of-date-upward-flag it) t))
		   (setf (out-of-date-backward-flag n) t)
		   (do-elements (c (children n))
		     (setf (out-of-date-forward-flag c) t))
		   (debug-print 3 "New value is ~a. ~:[~;Propagating upward~]."
				n (upward-parent n))))
		   
	     (prog1 (changed n) (slot-makunbound n 'changed))))
  (:method ((n <alt-node>)) nil))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; known value data structure
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun lookup-known-value (type key field known-values)
  "lookup-known-value TYPE KEY FIELD KNOWN-VALUES.  The idea is that KNOWN-VALUES is an association list from types to tables.  Examples include STATE, STATE-ACTION, and so on.  KEY identifies the record within the table, e.g. a particular state. FIELD is the information needed about the record, e.g., q-value, num-visits, etc.  The record must be an assoc list from field name to value.  Returns nil if no such record exists, or doesn't have the particular field asked for, the value otherwise, and signals a mapping-undefined error if the table doesn't exist."
  (mapping:evaluate-mv (known-value-mapping type key known-values) field))

(defun known-value-mapping (type key known-values)
  "known-value-mapping TYPE KEY KNOWN-VALUES.  Return the object corresponding to known values for this KEY in table TYPE in KNOWN-VALUES (see lookup-known-value for more).  Return nil if no such key exists, and error if table TYPE doesn't exist."
  (mapping:evaluate-mv (mapping:evaluate known-values type) key))

(defun modify-known-value-rec (rec field new-value mod)
  (setf mod
    (case mod
      (replace #'(lambda (x y) (declare (ignore x)) y))
      (max #'(lambda (x y) (mymax (or x '-infty) y)))
      (min #'(lambda (x y) (mymin (or x 'infty) y)))
      (otherwise mod)))
  (mapping:set-value rec field (funcall mod (mapping:evaluate-mv rec field) new-value)))

(defun modify-known-value (type key field new-value mod known-values)
  "modify-known-value TYPE KEY FIELD NEW-VAL MODIFICATION KNOWN-VALUES.  See lookup-known-value.  The new stored value equals the modification function applied to the old value (or nil if it doesn't exist) and new-value.  MODIFICATION can be 'replace, 'max, 'min, or an arbitrary two argument function.  

Returns the modified record (which can then be modified further)."
  
  (let* ((table (mapping:evaluate known-values type))
	 (rec (mapping:evaluate-mv table key))
	 (new-rec (modify-known-value-rec rec field new-value mod)))
    
    ;; If there wasn't already a record for key, add it to the table
    ;; Pushing #'eq first is to deal with the annoying setf issues that arise with null lists
    ;; Note that mappings can also be association lists with an equality test cons-ed on to the beginning
    (unless rec
      (push #'eq new-rec)
      (mapping:set-value table key new-rec))
    new-rec))

(defun make-known-values (&rest args)
  (cons #'eq (mapcar #'(lambda (pair) (list (car pair) (cdr pair))) (apply #'p2alist args))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Misc tree ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun update-nodes-backward (nodes known-values &optional (common-parent nil))
  "update-nodes-backward NODES KNOWN-VALUES &optional (COMMON-PARENT)
Ensure that nodes along each path from the root to NODES (assumed in the same tree) are backwards-up-to-date.  COMMON-PARENT, if provided, must be a common ancestor of the nodes (defaults to the root)."
  
  ;; Backup each node till the common parent
  (do-elements (n (typecase nodes (<node> (list nodes)) (t nodes)))
    (let ((node n))
      (until (eq node common-parent)
	(update-backward node known-values)
	(if (root? node) 
	    (return nil)
	  (setf node (parent-node node))))))
  
  ;; Backup beyond the common parent
  (when common-parent
    (let ((node common-parent))
      (loop
	(update-backward node known-values)
	(if (root? node)
	    (return nil)
	  (setf node (parent-node node)))))))

(defun upward-propagate (node known-values)
  "upward-propagate NODE KNOWN-VALUES.  For now, what this does is: call update-upward on NODE and its downward children.  Then update backward starting at node."
  (when *upward-prop*
    (update-upward node)
    (update-nodes-backward node known-values)))
  

(defun active-children (n)
  "Return list of active child edges of NODE"
  (filter ':implicit (child-edges n) #'(lambda (e) (eq 'active (status (head e))))))

(defun active-leaves (tree &optional (non-subsumed-only nil))
  "active-leaves NODE &optional (NON-SUBSUMED-ONLY nil).  Return the set of leaves of the active subtree headed by NODE (note that a node can be a leaf in the active subtree even if not a leaf of the original tree).  The set is represented implicitly."
  (labels ((helper (node)
	     (unless (and non-subsumed-only (subsumed node))
	       (let ((c (active-children node)))
		 (if (is-empty c)
		     (list node)
		   (ndunion ((e c))
		     (helper (head e))))))))
    (helper tree)))

(defun active-child-nodes (n)
  (filter ':implicit (children n) #'(lambda (n2) (eq 'active (status n2)))))

(defun active-descendants (n)
  (when (active n)
    (disjoint-union (list n) (ndunion ((c (active-child-nodes n))) (active-descendants c)))))


(defun get-plan-path (tree plan &optional (allow-inactive nil))
  "get-plan-path TREE PLAN 
TREE is an ALT.
PLAN is an array of action names.

Return an array #(N0 ... Nk) where N0 is the root, and the N form a path in the tree whose actions are equal to the PLAN_i.  If no such path exists, an assertion will happen.

As a secondary value, returns an array #(E1 ... Ek) where Ei is the parent of Ni"
  (let ((path (make-array (1+ (length plan))))
	(edge-path (make-array (length plan))))
    (setf (aref path 0) tree)
    (dotimes (i (length plan) (values path edge-path))
      (let ((e (check-not-null (get-action-edge (aref path i) (aref plan i) nil allow-inactive)
			       "Edge ~a of plan ~a in tree" i plan)))
	(setf (aref path (1+ i)) (head e)
	      (aref edge-path i) e)))))

(defun get-node-plan (node)
  "get-node-plan NODE.
Return an array #(A1 ... An) such that, starting at the root of the tree that contains NODE, if we follow edges corresponding to the actions Ai, we will end up at NODE."
  (map 'vector #'action (cdr (get-path-from-root node))))

(defun plan-final-node (tree plan)
  (slast (get-plan-path tree plan t)))



(defun max-valuation-change (v1 v2)
  (let ((m1 (max-achievable-value v1)))
    (if (eql m1 '-infty)
	'-infty
      (my- (max-achievable-value v2) m1))))

(defun add-ref-link (start end par)
  (setf (upward-parent end) par)
  (adjoinf (downward-spans par) (cons start end) :test #'equal))

(defun active (n)
  (eq (status n) 'active))

(defun is-primitive-node (n)
  (is-primitive (action n) (hpp n)))

(defun is-primitive-plan (plan hpp)
  (every #'(lambda (a) (is-primitive a hpp)) plan))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod print-object ((l <alt-node>) str)
  (print-unreadable-object (l str :type nil :identity nil)
    (format str "~a (~:[in~;~]active).  ~:[~;Out-of-date forward.  ~]~:[~;Out-of-date backward.~]"
	    (action l) (eq (status l) 'active) (out-of-date-forward-flag l)
	    (out-of-date-backward-flag l))))

(defun pprint-plans (&rest args)
  (labels ((helper (node str)
	     (pprint-logical-block (str nil)
	       (if (leaf? node)
		   (format str "Sound: ~$  Complete: ~$"
			   (sound-reward-upto node) 
			   (complete-reward-upto node))
		 (do-elements (e (active-children node) nil i)
		   (format str "~:[~0I~:@_~;~]~a~2I~:@_" (zerop i) (edge-action e))
		   (helper (head e) str))))))
    (bind-pprint-args (str tree) args
      (helper tree str))))


(defun is-well-formed (tree hpp)
  (labels ((helper (n2 active)
	     (let ((e (parent-edge n2)))
	       (aif
		   (if e
		       (let* ((n1l (tail e))
			      (el (head e))
			      (n2l n2)
			      (a (action el))
			      (ss (sound-set n1l))
			      (sv (sound-valuation n1l))
			      (cv (complete-valuation n1l))
			      (cs (complete-set n1l)))
			 (cond ((not (set-eq (sound-set n2l) (sound-reachable-set a ss hpp))) 'sound-reachable)
			       ((not (set-eq (complete-set n2l) (complete-reachable-set a cs hpp))) 'complete-reachable)
			       ((not (equal-valuations (sound-valuation n2l) (progress-sound a sv hpp))) 'sound-valuation)
			       ((not (equal-valuations (complete-valuation n2l) (progress-complete a cv hpp))) 
				'complete-valuation)
			       ((and (state n1l) (is-primitive a hpp)
				     (not (same-state (state n2l) (succ-state (state n1l) (sound-desc hpp a)))))
				'state)))
		     (cond ((not (my= (sound-reward-upto n2) 0)) 'sound-reward-upto)
			   ((not (my= (complete-reward-upto n2) 0)) 'complete-reward-upto)
			   ((not (state n2)) 'state)))
		   (values it n2)
		 (do-elements (e (child-edges n2))
		   (mvbind (v n) (helper (head e) (and active (eq (status (head e)) 'active)))
		     (when v (return (values v n)))))))))
    
    (mvbind (v n) (helper tree t)
      (if v (values nil v n) t))))
			
		    
		    
	       

