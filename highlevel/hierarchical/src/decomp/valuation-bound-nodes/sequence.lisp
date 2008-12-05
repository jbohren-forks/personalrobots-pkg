(in-package :vb-node)

(defclass <sequence-node> (<node>)
  ((sequence-length :reader sequence-length :initarg :sequence-length)))

(defmethod initialize-instance :after ((n <sequence-node>) &rest args &key sequence-length)
  ;; Add child nodes
  (dotimes (i sequence-length)
    (add-child n i 'todo))

  ;; Each child node effectively gets its inputs from its neighbours or, at the beginning and end, from the node's own inputs
  (dotimes (i sequence-length)
    (let ((child (child i n)))
      (cond 
	((zerop i)
	 (tie-variables n 'initial-optimistic child 'initial-optimistic)
	 (tie-variables n 'initial-pessimistic child 'initial-pessimistic))
	(t
	 (tie-variables n (cons 'child-progressed-optimistic (1- i)) child 'initial-optimistic)
	 (tie-variables n (cons 'child-progressed-pessimistic (1- i)) child 'initial-optimistic)))
      (cond
	((= i (1- sequence-length))
	 (tie-variables n 'final-optimistic child 'final-optimistic)
	 (tie-variables n 'final-pessimistic child 'final-pessimistic))
	(t
	 (tie-variables n (cons 'child-regressed-optimistic (1+ i)) child 'final-optimistic)
	 (tie-variables n (cons 'child-regressed-pessimistic (1+ i)) child 'final-pessimistic))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The only time child vars have an explicit dependant is 
;; at the ends of the sequence 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod child-progressed-optimistic-dependants ((n <sequence-node>) i)
   (when (= (1- (sequence-length n)) i)
     '(progressed-optimistic)))

(defmethod child-progressed-pessimistic-dependants ((n <sequence-node>) i)
   (when (= (1- (sequence-length n)) i)
     '(progressed-pessimistic)))

(defmethod child-regressed-optimistic-dependants ((n <sequence-node>) i)
   (when (zerop i)
     '(regressed-optimistic)))

(defmethod child-regressed-pessimistic-dependants ((n <sequence-node>) i)
   (when (zerop i)
     '(regressed-pessimistic)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The node's outputs are computed from the node's 
;; own progressed/regressed valuations together with the
;; final/initial child's output valuation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod optimistic-progressor ((n <sequence-node>))
  (let ((length (sequence-length n)))
    (make-simple-update-fn
     #'(lambda (l)
	 (set-intersection (evaluate l 'my-progressed-optimistic) (evaluate l (cons 'child-progressed-optimistic (1- length))))))))

(defmethod pessimistic-progressor ((n <sequence-node>))
  (let ((length (sequence-length n)))
    (make-simple-update-fn
     #'(lambda (l)
	 (set-union (evaluate l 'my-progressed-pessimistic) (evaluate l (cons 'child-progressed-pessimistic (1- length))))))))

(defmethod optimistic-regressor ((n <sequence-node>))
  (make-simple-update-fn
   #'(lambda (l)
       (set-intersection (evaluate l 'my-regressed-optimistic) (evaluate l (cons 'child-regressed-optimistic 0))))))

(defmethod pessimistic-regressor ((n <sequence-node>))
  (make-simple-update-fn
   #'(lambda (l)
       (set-union (evaluate l 'my-regressed-pessimistic) (evaluate l (cons 'child-regressed-pessimistic 0))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self, then pass control to child
;; with worst pessimistic estimate
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod compute-cycle ((n <sequence-node>))
  (do-all-updates n)
  (let ((i (maximizing-element (child-ids n) (sequence-node-child-evaluator n))))
    (compute-cycle (child i n)))
  (do-all-updates n))

(defun sequence-node-child-evaluator (n)
  #'(lambda (i)
      'todo))