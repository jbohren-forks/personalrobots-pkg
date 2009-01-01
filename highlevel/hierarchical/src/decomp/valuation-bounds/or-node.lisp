(in-package :vb-node)

(define-debug-topic :or-node :vb-node)

(defclass <or-node> (<node>)
  ((status :initform :initial)
   (next-child :initform 0 :accessor next-child)))

(defmethod initialize-instance :after ((n <or-node>) &rest args)
  (declare (ignorable args))
  ;; Aggregated inputs from children
  (add-variable n 'children-progressed-optimistic :internal :simple-update-fn (or-node-progressed-optimistic-aggregator n) :dependants '(progressed-optimistic))
  (add-variable n 'children-progressed-pessimistic :internal :simple-update-fn (or-node-progressed-pessimistic-aggregator n) :dependants '(progressed-pessimistic))
  (add-variable n 'children-regressed-optimistic :internal :simple-update-fn (or-node-regressed-optimistic-aggregator n) :dependants '(regressed-optimistic))
  (add-variable n 'children-regressed-pessimistic :internal :simple-update-fn (or-node-regressed-pessimistic-aggregator n) :dependants '(regressed-pessimistic)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Adding child nodes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod add-child :after ((n <or-node>) i new-node-type &rest args)
  (declare (ignore new-node-type args))
  (let ((child (child i n)))

    ;; Child inputs are tied to node's own inputs
    (dolist (v '(initial-optimistic initial-pessimistic final-optimistic final-pessimistic))
      (tie-variables n v child v))

    ;; Add output variables from child and tie them 
    (add-variable n (cons 'child-progressed-optimistic i) :external :dependants '(children-progressed-optimistic) 
		  :initial-value (make-simple-valuation (universal-set (planning-domain n)) 'infty))
    (tie-variables child 'progressed-optimistic  n (cons 'child-progressed-optimistic i))

    (add-variable n (cons 'child-progressed-pessimistic i) :external :dependants '(children-progressed-pessimistic) 
		  :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))
    (tie-variables child 'progressed-pessimistic n (cons 'child-progressed-pessimistic i))

    (add-variable n (cons 'child-regressed-optimistic i) :external :dependants '(children-regressed-optimistic) 
		  :initial-value (make-simple-valuation (universal-set (planning-domain n)) 'infty))
    (tie-variables child 'regressed-optimistic n (cons 'child-regressed-optimistic i))

    (add-variable n (cons 'child-regressed-pessimistic i) :external :dependants '(children-regressed-pessimistic) 
		  :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))
    (tie-variables child 'regressed-pessimistic n (cons 'child-regressed-pessimistic i))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Children outputs are aggregated together
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun or-node-progressed-optimistic-aggregator (n)
  #'(lambda (l)
      ;; Note that we're treating the case with no children differently
      ;; in optimistic progression and regression, because it doesnt' mean that 
      ;; there are no achievable states - just that we don't have any bounds on them yet
      (if l 
	  (reduce #'binary-pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation (universal-set (planning-domain n)) 'infty))))

(defun or-node-progressed-pessimistic-aggregator (n)
  #'(lambda (l)
      (reduce #'binary-pointwise-max-lower-bound l :key #'cdr :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))))

(defun or-node-regressed-optimistic-aggregator (n)
  #'(lambda (l)
      ;; see progressed version above
      (if l
	  (reduce #'binary-pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation (universal-set (planning-domain n)) 'infty))))

(defun or-node-regressed-pessimistic-aggregator (n)
  #'(lambda (l)
      (reduce #'binary-pointwise-max-lower-bound l :key #'cdr :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self and pass control to best
;; child (like A*)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod action-node-type ((c (eql :or)))
  '<or-node>)

(defmethod compute-cycle ((n <or-node>))
  (debug-out :or-node 1 t "~&In compute cycle of or-node ~a with status ~a" (action n) (status n))
  (ecase (status n)
    (:initial (do-all-updates n) (setf (status n) :create-children))
    (:create-children (or-node-create-children n) (setf (status n) :cycle))
    (:cycle (unless (zerop (size (children n)))
	      (do-all-updates n)
	      (compute-cycle (child (mod (incf (next-child n)) (size (children n))) n))
	      (do-all-updates n))
	    (setf (status n) :best-child))
    (:best-child
     (do-all-updates n)
     (let ((children (child-ids n)))
       (debug-out :or-node 1 t "~&Child scores of ~a are ~a" (action n) (mapset 'list (or-node-child-evaluator n) children))
       (unless (is-empty children)
	 (let ((i (maximizing-element children #'(lambda (j) (or-node-evaluate-child n j)))))
	   (compute-cycle (child i n)))
	 (do-all-updates n))
       (setf (status n) :cycle)))))

(defun or-node-evaluate-child (n i)
  (max-achievable-value (make-sum-valuation (current-value n (cons 'child-progressed-optimistic i)) (current-value n 'final-optimistic))))


(defun or-node-create-children (n)
  (with-accessors ((action action) (h hierarchy)) n
    (do-elements (ref (refinements action h :init-opt-set (reachable-set (current-value n 'initial-optimistic))) nil i)
      (assert (= 1 (length ref)))
      (create-child-for-action h n i (sfirst ref)))))
      

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; extracting plans
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



    
    