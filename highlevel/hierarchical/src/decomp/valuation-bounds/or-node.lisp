(in-package :vb-node)

(define-debug-topic :or-node :vb-node)

(defclass <or-node> (<node>)
  ((status :initform :initial)))

(defmethod initialize-instance :after ((n <or-node>) &rest args)
  (declare (ignorable args))
  ;; Aggregated inputs from children
  (add-variable n 'children-progressed-optimistic :internal :simple-update-fn (child-progressed-optimistic-aggregator n) :dependants '(progressed-optimistic))
  (add-variable n 'children-progressed-pessimistic :internal :simple-update-fn (child-progressed-pessimistic-aggregator n) :dependants '(progressed-pessimistic))
  (add-variable n 'children-regressed-optimistic :internal :simple-update-fn (child-regressed-optimistic-aggregator n) :dependants '(regressed-optimistic))
  (add-variable n 'children-regressed-pessimistic :internal :simple-update-fn (child-regressed-pessimistic-aggregator n) :dependants '(regressed-pessimistic)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Within the node:
;; Update functions for output variables depend on
;; aggregated child outputs, and on node's own
;; progressed/regressed valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod optimistic-progressor ((n <or-node>))
  (make-simple-alist-updater (my-progressed-optimistic children-progressed-optimistic)
    (pointwise-min-upper-bound my-progressed-optimistic children-progressed-optimistic)))

(defmethod pessimistic-progressor ((n <or-node>))
  (make-simple-alist-updater (my-progressed-pessimistic children-progressed-pessimistic)
    (pointwise-max-lower-bound my-progressed-pessimistic children-progressed-pessimistic)))

(defmethod optimistic-regressor ((n <or-node>))
  (make-simple-alist-updater (my-regressed-optimistic children-regressed-optimistic)
    (pointwise-min-upper-bound my-regressed-optimistic children-regressed-optimistic)))

(defmethod pessimistic-regressor ((n <or-node>))
  (make-simple-alist-updater (my-regressed-pessimistic children-regressed-pessimistic)
    (pointwise-max-lower-bound my-regressed-pessimistic children-regressed-pessimistic)))




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

(defmethod child-progressed-optimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      ;; Note that we're treating the case with no children specially
      ;; in optimistic progression and regression
      (if l
	  (reduce #'pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation (universal-set (planning-domain n)) 'infty))))

(defmethod child-progressed-pessimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      (reduce #'pointwise-max-upper-bound l :key #'cdr :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))))

(defmethod child-regressed-optimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      ;; Note that we're treating the case with no children specially
      ;; in optimistic progression and regression
      (if l
	  (reduce #'pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation (universal-set (planning-domain n)) 'infty))))

(defmethod child-regressed-pessimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      (reduce #'pointwise-max-upper-bound l :key #'cdr :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self and pass control to best
;; child (like A*)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod action-node-type ((c (eql :or)))
  '<or-node>)

(defmethod compute-cycle ((n <or-node>))
  (debug-out :or-node 1 t "In compute cycle of or-node ~a with status ~a" (action n) (status n))
  (ecase (status n)
    (:initial (do-all-updates n) (setf (status n) :create-children))
    (:create-children (or-node-create-children n) (setf (status n) :children-created))
    (:children-created
     (do-all-updates n)
     (let ((children (child-ids n)))
       (unless (is-empty children)
	 (let ((i (maximizing-element children (or-node-child-evaluator n))))
	   (compute-cycle (child i n)))
	 (do-all-updates n))))))

(defun or-node-child-evaluator (n)
  #'(lambda (i)
      (max-achievable-value (make-sum-valuation (current-value n (cons 'child-progressed-optimistic i)) (current-value n 'final-optimistic)))))


(defun or-node-create-children (n)
  (with-accessors ((action action) (h hierarchy)) n
    (do-elements (ref (refinements action h :init-opt-set (reachable-set (current-value n 'initial-optimistic))) nil i)
      (assert (= 1 (length ref)))
      (create-child-for-action h n i (sfirst ref)))))
      

    
    