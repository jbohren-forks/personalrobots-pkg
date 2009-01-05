(in-package :valuation-bound-node)

(define-debug-topic :node :vb-node)

(defclass <node> (<dependency-graph>)
  ((id :initarg :action :reader action)
   (descs :initarg :descs)
   (equality-test :initform #'equal-valuations)
   (root-node :writer set-root-node :reader root-node)
   (cycle-number :initform 0 :accessor cycle-number)
   (children :reader children :initform (make-hash-table :test #'eql)) 
   

   (status :accessor status))
  
  (:documentation "A valuation-bound node implements the following protocol:
The parents pass in four variables {initial|final}-{optimistic|pessimistic}.  The node in turn is responsible for passing back four corresponding variables {progressed|regressed}-{optimistic|pessimistic}.  The children are assumed to follow the same protocol.  The node is responsible for setting the inputs of its children.  Each child's outputs become external variables of this node, named using the child's unique ID.  For example, child I's progressed-optimistic variable gets tied to this node's variable (child-progressed-optimistic . I).  "))


(defmethod initialize-instance :after ((n <node>) &key parent)
  "Creates the basic input and output variables to the parent, and the node's own progressed/regressed valuations.  Subtypes extend this to add additional variables and set up dependencies within the node."
  (set-root-node (if parent (root-node parent) n) n)
  (unless (eq n (root-node n))
    (assert (not (slot-boundp n 'descs)) nil "descs arguments should be nil for a non-root node"))

  ;; Inputs from parents
  (add-variable n 'initial-optimistic :external)
  (add-variable n 'initial-pessimistic :external)
  (add-variable n 'final-optimistic :external)
  (add-variable n 'final-pessimistic :external)

  ;; Progressed/regressed valuations
  ;; Assymetric in that progressed valuations depend on just the initial ones, but regressed valuations depend on
  ;; not just the final ones but also the initial ones.  This is because otherwise, esp. for higher level actions,
  ;; theree would be a huge number of irrelevant states in the regressed set
  (with-accessors ((descs descs) (action action)) n
    (add-variable n 'my-progressed-optimistic :internal :dependees '(initial-optimistic)
		  :simple-update-fn (make-alist-function (initial-optimistic) (progress-optimistic descs action initial-optimistic)))
    (add-variable n 'my-progressed-pessimistic :internal :dependees '(initial-pessimistic)
		  :simple-update-fn (make-alist-function (initial-pessimistic) (progress-pessimistic descs action initial-pessimistic)))
    (add-variable n 'my-regressed-optimistic :internal :dependees '(initial-optimistic final-optimistic)
		  :simple-update-fn (make-alist-function (initial-optimistic final-optimistic) (regress-optimistic descs action initial-optimistic final-optimistic)))
    (add-variable n 'my-regressed-pessimistic :internal :dependees '(initial-pessimistic final-pessimistic)
		  :simple-update-fn (make-alist-function (initial-pessimistic final-pessimistic) (regress-pessimistic descs action initial-pessimistic final-pessimistic))))

  ;; Outputs to parents.  Subclass :after method can add additional dependees of these variables.  For now, the update function always works the same way - e.g.,
  ;; we always compute progressed-optimistic by taking a pointwise-min of the parent variables.  Maybe allow child to change this if situation comes up where that's needed.
  (add-variable n 'progressed-optimistic :internal :update-fn (make-simple-update-fn #'node-optimistic-progression) :dependees '(my-progressed-optimistic))
  (add-variable n 'progressed-pessimistic :internal :update-fn (make-simple-update-fn #'node-pessimistic-progression) :dependees '(my-progressed-pessimistic))
  (add-variable n 'regressed-optimistic :internal :update-fn (make-simple-update-fn #'node-optimistic-regression) :dependees '(my-regressed-optimistic))
  (add-variable n 'regressed-pessimistic :internal :update-fn (make-simple-update-fn #'node-pessimistic-regression) :dependees '(my-regressed-pessimistic))

  (add-variable n 'node-optimistic-value-regressed :internal :update-fn (make-simple-update-fn #'maximize-sum-valuation) :dependees '(regressed-optimistic initial-optimistic))
  (add-variable n 'node-pessimistic-value-regressed :internal :update-fn (make-simple-update-fn #'maximize-sum-valuation) :dependees '(regressed-pessimistic initial-pessimistic)))

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subtypes must implement action-node-type, compute-cycle, 
;; and :after methods for add-child to tie the
;; variables of child to variables of node
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric action-node-type (action-class)
  (:documentation "Maps from the action type (e.g :or) to the class name of corresponding nodes (<or-node>)."))


(defgeneric add-child (n child-id new-node-type &rest args)
  (:documentation "Add a child node and set up communication with it.")
  (:method ((n <node>) child-id new-node-type &rest args)
    ;; The top method just creates and adds the child node object (which will set up the variables and communication within the child)
    ;; Subclasses should define :after methods for add-child that set up communication between n and the child
    (let ((child (apply #'make-instance new-node-type :parent n args)))
      (setf (evaluate (children n) child-id) child)
      (debug-out :node 1 t "~&Added child ~a to node ~a" (action child) (action n)))))

(defgeneric compute-cycle (n)
  (:documentation "This is where it all happens.  Improves the node's estimate of output vars given input vars.  May recursively call compute-cycle on children.  Takes time linear in depth of tree rooted at n.")
  (:method :after (n) (incf (cycle-number n))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Things implemented here
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun create-child-for-action (hierarchy node id a)
  (add-child node id (action-node-type (action-class a hierarchy)) :action a))

(defun node-optimistic-progression (l)
  (reduce #'binary-pointwise-min-upper-bound l :key #'cdr))

(defun node-pessimistic-progression (l)
  (reduce #'binary-pointwise-max-lower-bound l :key #'cdr))

(defun node-optimistic-regression (l)
  (reduce #'binary-pointwise-min-upper-bound l :key #'cdr))

(defun node-pessimistic-regression (l)
  (reduce #'binary-pointwise-max-lower-bound l :key #'cdr))

(defun maximize-sum-valuation (l)
  (assert (= (length l) 2))
  (max-achievable-value (make-sum-valuation (cdar l) (cdadr l))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Getting plans from a node
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun node-optimistic-value-regressed (n)
  (handler-case
      (current-value n 'node-optimistic-value-regressed)
    (uninitialized-variable () 'infty)))

(defun node-pessimistic-value-regressed (n)
  (handler-case
      (current-value n 'node-pessimistic-value-regressed)
    (uninitialized-variable () '-infty)))

(defgeneric primitive-plan-with-pessimistic-future-value-above (n init-state v)
  (:documentation "If N has a primitive subplan whose future value (within this node and after) starting at init-state exceeds V, return one (as a vector), and the successor state and reward within the node.  Else return nil"))
  


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Accessors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun child-ids (n)
  (domain (children n)))

(defun child (i n)
  (evaluate (children n) i))

(defmethod hierarchy ((n <node>))
  (hierarchy (descs n)))

(defun descs (n)
  (slot-value (root-node n) 'descs))

(defmethod planning-domain ((n <node>))
  (planning-domain (hierarchy (descs n))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun print-node (&rest args)
  (bind-pprint-args (str n) args
    (pprint-logical-block 
     (str nil :prefix "[" :suffix "]")
     (format str "Node ~a" (action n))
     (format str "~:@_ Children: ~a" (map 'list #'action (range (children n))))
     (handler-bind
	 ((uninitialized-variable #'(lambda (c) (declare (ignore c)) (use-value :uninitialized))))
       (format str "~:@_ Inputs:")
       (dolist (v '(initial-optimistic initial-pessimistic final-optimistic final-pessimistic))
	 (format str "~:@_  ~a: ~a" v (current-value n v)))
       (format str "~:@_ Outputs:")
       (dolist (v '(progressed-optimistic progressed-pessimistic regressed-optimistic regressed-pessimistic))
	 (format str "~:@_  ~a: ~a" v (current-value n v)))))))

(set-pprint-dispatch '<node> #'print-node 1)
     
(defun print-children (n)
  (let ((h (make-hash-table)))
    (maphash #'(lambda (k v) (setf (gethash k h) (action v))) (children n))
    (pprint-hash h)))

(defun node-inputs (n)
  (current-values n '(initial-optimistic initial-pessimistic final-optimistic final-pessimistic)))

(defun node-outputs (n)
  (current-values n '(progressed-optimistic progressed-pessimistic regressed-optimistic regressed-pessimistic)))