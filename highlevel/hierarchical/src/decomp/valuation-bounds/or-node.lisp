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
  (let ((child (child i n))
	(descs (descs n)))

    ;; Child inputs are tied to node's own inputs
    (dolist (v '(initial-optimistic initial-pessimistic final-optimistic final-pessimistic))
      (tie-variables n v child v))

    ;; Add output variables from child and tie them 
    (add-variable n (cons 'child-progressed-optimistic i) :external :dependants '(children-progressed-optimistic) 
		  :initial-value (maximal-valuation descs))
    (tie-variables child 'progressed-optimistic  n (cons 'child-progressed-optimistic i))

    (add-variable n (cons 'child-progressed-pessimistic i) :external :dependants '(children-progressed-pessimistic) 
		  :initial-value (minimal-valuation descs))
    (tie-variables child 'progressed-pessimistic n (cons 'child-progressed-pessimistic i))

    (add-variable n (cons 'child-regressed-optimistic i) :external :dependants '(children-regressed-optimistic) 
		  :initial-value (maximal-valuation descs))
    (tie-variables child 'regressed-optimistic n (cons 'child-regressed-optimistic i))

    (add-variable n (cons 'child-regressed-pessimistic i) :external :dependants '(children-regressed-pessimistic) 
		  :initial-value (minimal-valuation descs))
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
	(maximal-valuation (descs n)))))

(defun or-node-progressed-pessimistic-aggregator (n)
  #'(lambda (l)
      (reduce #'binary-pointwise-max-lower-bound l :key #'cdr :initial-value (minimal-valuation (descs n)))))

(defun or-node-regressed-optimistic-aggregator (n)
  #'(lambda (l)
      ;; see progressed version above
      (if l
	  (reduce #'binary-pointwise-max-upper-bound l :key #'cdr)
	(maximal-valuation (descs n)))))

(defun or-node-regressed-pessimistic-aggregator (n)
  #'(lambda (l)
      (reduce #'binary-pointwise-max-lower-bound l :key #'cdr :initial-value (minimal-valuation (descs n)))))

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
    (:cycle (let ((max-pessimistic (reduce-set #'mymax (children n) :key #'(lambda (i) (node-pessimistic-value-regressed (child i n)))))
		  (num-children (size (children n))))
	      (debug-out :or-node 2 t "~&Max pessimistic value is ~a" max-pessimistic)
	      (with-accessors ((c next-child)) n
		(dotimes (i num-children (do-all-updates n))
		  (setq c (mod-inc num-children c))
		  (unless (my< (node-optimistic-value-regressed (child c n)) max-pessimistic)
		    (do-all-updates n)
		    (compute-cycle (child c n))
		    (do-all-updates n)
		    (return nil))))))))


(defun or-node-create-children (n)
  (with-accessors ((action action) (h hierarchy)) n
    (do-elements (ref (refinements action h :init-opt-set (reachable-set (current-value n 'initial-optimistic))) nil i)
      (assert (= 1 (length ref)))
      (create-child-for-action h n i (sfirst ref)))))
      

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; extracting plans
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod primitive-plan-with-pessimistic-future-value-above ((n <or-node>) state v)
  (do-elements (i (children n))
    (let ((child (child i n)))
      (mvbind (plan successor reward) (primitive-plan-with-pessimistic-future-value-above child state v)
	(when plan (return (values plan successor reward)))))))