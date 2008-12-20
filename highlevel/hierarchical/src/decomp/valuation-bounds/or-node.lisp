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
  (add-variable n 'children-regressed-pessimistic :internal :simple-update-fn (child-regressed-optimistic-aggregator n) :dependants '(regressed-pessimistic)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Children outputs are aggregated together
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod child-progressed-optimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(children-progressed-optimistic))


(defmethod child-progressed-optimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      ;; Note that we're treating the case with no children specially
      ;; in optimistic progression and regression
      (if l
	  (reduce #'pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation t 'infty))))


(defmethod child-progressed-pessimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(children-progressed-pessimistic))

(defmethod child-progressed-pessimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      (reduce #'pointwise-max-upper-bound l :key #'cdr)))



(defmethod child-regressed-optimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(children-regressed-optimistic))

(defmethod child-regressed-optimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      ;; Note that we're treating the case with no children specially
      ;; in optimistic progression and regression
      (if l
	  (reduce #'pointwise-max-upper-bound l :key #'cdr)
	  (make-simple-valuation t 'infty))))


(defmethod child-regressed-pessimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(children-regressed-pessimistic))

(defmethod child-regressed-pessimistic-aggregator ((n <or-node>))
  #'(lambda (l)
      (reduce #'pointwise-max-upper-bound l :key #'cdr)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node's inputs are just passed over to children
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod child-initial-optimistic-tied-to ((n <or-node>) id)
  'initial-optimistic)

(defmethod child-initial-pessimistic-tied-to ((n <or-node>) id)
  'initial-pessimistic)

(defmethod child-final-optimistic-tied-to ((n <or-node>) id)
  'final-optimistic)

(defmethod child-final-pessimistic-tied-to ((n <or-node>) id)
  'final-pessimistic)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
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
      (max-achievable-value (make-sum-valuation (current-value (list 'child-progressed-optimistic i) n) (current-value 'final-regressed-optimistic n)))))


(defun or-node-create-children (n)
  (with-accessors ((action action) (h hierarchy)) n
    (do-elements (ref (refinements action h :init-opt-set (reachable-set (current-value n 'initial-optimistic))) nil i)
      (assert (= 1 (length ref)))
      (create-child-for-action h n i (sfirst ref)))))
      

    
    