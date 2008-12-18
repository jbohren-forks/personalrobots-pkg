(in-package :vb-node)

(defclass <or-node> (<node>)
  ((status :accessor status)))

(defmethod initialize-instance :after ((n <or-node>) &rest args)
  (declare (ignorable args))
  ;; Aggregated inputs from children
  (add-variable n 'children-progressed-optimistic :internal :simple-update-fn (aggregator #'unite) :dependants '(progressed-optimistic))
  (add-variable n 'children-progressed-pessimistic :internal :simple-update-fn (aggregator #'unite) :dependants '(progressed-pessimistic))
  (add-variable n 'children-regressed-optimistic :internal :simple-update-fn (aggregator #'unite) :dependants '(regressed-optimistic))
  (add-variable n 'children-regressed-pessimistic :internal :simple-update-fn (aggregator #'unite) :dependants '(regressed-pessimistic)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Children outputs are aggregated together
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod child-progressed-optimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(child-progressed-optimistic))

(defmethod child-progressed-pessimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(child-progressed-pessimistic))

(defmethod child-regressed-optimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(child-regressed-optimistic))

(defmethod child-regressed-pessimistic-dependants ((n <or-node>) i)
  (declare (ignore i))
  '(child-regressed-pessimistic))


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
  (make-simple-alist-updater (my-progressed-optimistic child-progressed-optimistic)
    (binary-intersection my-progressed-optimistic child-progressed-optimistic)))

(defmethod pessimistic-progressor ((n <or-node>))
  (make-simple-alist-updater (my-progressed-pessimistic child-progressed-pessimistic)
    (binary-union my-progressed-pessimistic child-progressed-pessimistic)))

(defmethod optimistic-regressor ((n <or-node>))
  (make-simple-alist-updater (my-regressed-optimistic child-regressed-optimistic)
    (binary-intersection my-regressed-optimistic child-regressed-optimistic)))

(defmethod pessimistic-regressor ((n <or-node>))
  (make-simple-alist-updater (my-regressed-pessimistic child-regressed-pessimistic)
    (binary-union my-regressed-pessimistic child-regressed-pessimistic)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self and pass control to best
;; child (like A*)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod action-node-type ((c (eql :or)))
  '<or-node>)

(defmethod compute-cycle ((n <or-node>))
  (ecase (status n)
    (:initial (do-all-updates n) (setf (status n) :create-children))
    (:create-children (or-node-create-children n) (setf (status n) :children-created))
    (:children-created
       (do-all-updates n)
       (let ((i (maximizing-element (child-ids n) (or-node-child-evaluator n))))
	 (compute-cycle (child i n)))
       (do-all-updates n))))

(defun or-node-child-evaluator (n)
  #'(lambda (i)
      (max-achievable-value (make-sum-valuation (current-value (list 'child-progressed-optimistic i) n) (current-value 'final-regressed-optimistic n)))))


(defun or-node-create-children (n)
  (with-accessors ((action action) (h hierarchy)) n
    (do-elements (ref (refinements action h :init-opt-set (current-value n 'initial-optimistic)) nil i)
      (assert (= 1 (length ref)))
      (create-child-for-action h n i (sfirst ref)))))

    
    