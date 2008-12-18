(in-package :valuation-bound-node)

(defclass <node> (<dependency-graph>)
  ((action :initarg :action :reader action)
   (descs :initarg :descs)
   (hierarchy :initarg :hierarchy)
   (root-node :writer set-root-node :reader root-node)
   (children :reader children :initform (make-adjustable-array))
   
   (optimistic-progressor :reader optimistic-progressor)
   (pessimistic-progressor :reader pessimistic-progressor)
   (optimistic-regressor :reader optimistic-regressor)
   (pessimistic-regressor :reader pessimistic-regressor))
  
  (:documentation "A valuation-bound node implements the following protocol:
The parents pass in four variables {initial|final}-{optimistic|pessimistic}.  The node in turn is responsible for passing back four corresponding variables {progressed|regressed}-{optimistic|pessimistic}.  The children are assumed to follow the same protocol.  The node is responsible for setting the inputs of its children.  Each child's outputs become external variables of this node, named using the child's unique ID.  For example, child I's progressed-optimistic variable gets tied to this node's variable (child-progressed-optimistic . I).  "))


(defmethod initialize-instance :after ((n <node>) &key parent)
  "Creates the basic input and output variables to the parent, and the node's own progressed/regressed valuations.  Subtypes extend this to add additional variables and set up dependencies within the node."
  (set-root-node (if parent (root-node parent) n) n)
  (unless (eq n (root-node n))
    (assert (not (or (slot-boundp n 'hierarchy) (slot-boundp n 'descs))) nil
	    "Hierarchy and descs arguments should be nil for a non-root node"))

  ;; Inputs from parents
  (add-variable n 'initial-optimistic :external)
  (add-variable n 'initial-pessimistic :external)
  (add-variable n 'final-optimistic :external)
  (add-variable n 'final-pessimistic :external)

  ;; Progressed/regressed valuations
  (with-slots (descs action) n
    (add-variable n 'my-progressed-optimistic :internal :dependees '(initial-optimistic)
		  :simple-update-fn (make-alist-function (initial-optimistic) (progress-optimistic descs action initial-optimistic)))
    (add-variable n 'my-progressed-pessimistic :internal :dependees '(initial-pessimistic)
		  :simple-update-fn (make-alist-function (initial-pessimistic) (progress-pessimistic descs action initial-pessimistic)))
    (add-variable n 'my-regressed-optimistic :internal :dependees '(initial-optimistic)
		  :simple-update-fn (make-alist-function (initial-optimistic) (regress-optimistic descs action initial-optimistic)))
    (add-variable n 'my-regressed-pessimistic :internal :dependees '(initial-pessimistic)
		  :simple-update-fn (make-alist-function (initial-pessimistic) (regress-pessimistic descs action initial-pessimistic))))

  ;; Outputs to parents.  Subclass will define the update functions (progressor/regressor).  Subclass :after method can add additional dependees of these variables.
  (add-variable n 'progressed-optimistic :internal :update-fn (optimistic-progressor n) :dependees '(my-progressed-optimistic))
  (add-variable n 'progressed-pessimistic :internal :update-fn (pessimistic-progressor n) :dependees '(my-progressed-pessimistic))
  (add-variable n 'regressed-optimistic :internal :update-fn (optimistic-regressor n) :dependees '(my-regressed-optimistic))
  (add-variable n 'regressed-pessimistic :internal :update-fn (pessimistic-regressor n) :dependees '(my-regressed-pessimistic)))

  
		


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subtypes with children should override
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric child-progressed-optimistic-dependants (n child-id))
(defgeneric child-progressed-pessimistic-dependants (n child-id))
(defgeneric child-regressed-optimistic-dependants (n child-id))
(defgeneric child-regressed-pessimistic-dependants (n child-id))
(defgeneric child-initial-optimistic-tied-to (n child-id) 
  (:method ((n <node>) child-id) (declare (ignore child-id)) nil))
(defgeneric child-initial-pessimistic-tied-to (n child-id)
  (:method ((n <node>) child-id) (declare (ignore child-id)) nil))
(defgeneric child-final-optimistic-tied-to (n child-id)
  (:method ((n <node>) child-id) (declare (ignore child-id)) nil))
(defgeneric child-final-pessimistic-tied-to (n child-id)
  (:method ((n <node>) child-id) (declare (ignore child-id)) nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subtypes must implement action-node-type, compute-cycle, 
;; and may implement :after methods for add-child to tie the
;; input variables of child to variables of node
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric action-node-type (action-class)
  (:documentation "Maps from the action type (e.g :or) to the class name of corresponding nodes (<or-node>)."))
(defgeneric add-child (n child-id new-node-type &rest args))
(defgeneric compute-cycle (n))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Things implemented here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod add-child ((n <node>) child-id new-node-type &rest args)
  "Add a child node, and set up communication from and to the child.  The dependencies within the child will be set by its initialize-instance :after method."
  (add-variable n (cons 'child-progressed-optimistic child-id) :external :dependants (child-progressed-optimistic-dependants n child-id))
  (add-variable n (cons 'child-progressed-pessimistic child-id) :external :dependants (child-progressed-pessimistic-dependants n child-id))
  (add-variable n (cons 'child-regressed-optimistic child-id) :external :dependants (child-regressed-optimistic-dependants n child-id))
  (add-variable n (cons 'child-regressed-pessimistic child-id) :external :dependants (child-regressed-pessimistic-dependants n child-id))
  (let ((child (apply #'make-instance new-node-type :parent n args)))
    (setf (evaluate (children n) child-id) child)
    
    ;; Set up upward communication
    (tie-variables child 'progressed-optimistic  n (cons 'child-progressed-optimistic child-id))
    (tie-variables child 'progressed-pessimistic (cons 'child-progressed-pessimistic child-id))
    (tie-variables child 'regressed-optimistic (cons 'child-regressed-optimistic child-id))
    (tie-variables child 'regressed-pessimistic (cons 'child-regressed-pessimistic child-id))

    ;; Set up downward communication if possible (otherwise, subtype should ensure that this happens before use)
    (awhen (child-initial-optimistic-tied-to n child-id)
      (tie-variables n it child 'initial-optimistic))
    (awhen (child-initial-pessimistic-tied-to n child-id)
      (tie-variables n it child 'initial-pessimistic))
    (awhen (child-final-optimistic-tied-to n child-id)
      (tie-variables n it child 'final-optimistic))
    (awhen (child-final-pessimistic-tied-to n child-id)
      (tie-variables n it child 'final-pessimistic))))


(defun create-child-for-action (hierarchy node id a)
  (add-child node id (action-node-type (action-class a hierarchy)) :action a))

(defun child-ids (n)
  (domain (children n)))

(defun child (i n)
  (evaluate (children n) i))

(defun hierarchy (n)
  (slot-value (root-node n) 'hierarchy))

(defun descs (n)
  (slot-value (root-node n) 'descs))