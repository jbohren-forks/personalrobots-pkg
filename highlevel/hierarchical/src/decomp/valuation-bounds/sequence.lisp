(in-package :vb-node)

(define-debug-topic :sequence-node :vb-node)

(defclass <sequence-node> (<node>)
  ((action-sequence :reader action-sequence :writer set-action-sequence)
   (next-child-to-refine :accessor next-child-to-refine :initform 0)
   (status :initform :initial)))

;; We don't have an initialize-instance :after method to add the node's variables
;; This is because they depend on what the refinement is, and we can't compute this until
;; we know what the initial valuation is.  So initialization is done in initialize-sequence-node.

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The node's outputs are computed from the node's 
;; own progressed/regressed valuations together with the
;; final/initial child's output valuation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod optimistic-progressor ((n <sequence-node>))
  (make-simple-update-fn 
   #'(lambda (l)
       (binary-pointwise-min-upper-bound
	(evaluate l (cons 'child-progressed-optimistic (1- (sequence-length n))))
	(evaluate l 'my-progressed-optimistic)))))

(defmethod pessimistic-progressor ((n <sequence-node>))
  (make-simple-update-fn 
   #'(lambda (l)
       (binary-pointwise-max-lower-bound
	(evaluate l (cons 'child-progressed-pessimistic (1- (sequence-length n))))
	(evaluate l 'my-progressed-pessimistic)))))

(defmethod optimistic-regressor ((n <sequence-node>))
  (make-simple-update-fn 
   #'(lambda (l)
       (binary-pointwise-min-upper-bound
	(evaluate l '(child-regressed-optimistic . 0))
	(evaluate l 'my-regressed-optimistic)))))


(defmethod pessimistic-regressor ((n <sequence-node>))
  (make-simple-update-fn 
   #'(lambda (l)
       (binary-pointwise-max-lower-bound
	(evaluate l '(child-regressed-pessimistic . 0))
	(evaluate l 'my-regressed-pessimistic)))))

 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Adding children
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod add-child :after ((n <sequence-node>) i new-node-type &rest args)
  (declare (ignore new-node-type args))

  (let ((child (child i n)))
    ;; Output variables already created in initialize-instance, so just tie them
    (dolist (vars '((progressed-optimistic child-progressed-optimistic) (progressed-pessimistic child-progressed-pessimistic)
		    (regressed-optimistic child-regressed-optimistic) (regressed-pessimistic child-regressed-pessimistic)))
      (tie-variables child (first vars) n (cons (second vars) i)))
    

    ;; Forward input messages
    (cond 
      ((zerop i)
       (tie-variables n 'initial-optimistic child 'initial-optimistic)
       (tie-variables n 'initial-pessimistic child 'initial-pessimistic))
      (t
       (tie-variables n (cons 'child-progressed-optimistic (1- i)) child 'initial-optimistic)
       (tie-variables n (cons 'child-progressed-pessimistic (1- i)) child 'initial-pessimistic)))

    ;; Backward input messages
    (cond
      ((= i (1- (sequence-length n)))
       (tie-variables n 'final-optimistic child 'final-optimistic)
       (tie-variables n 'final-pessimistic child 'final-pessimistic))
      (t
       (tie-variables n (cons 'child-regressed-optimistic (1+ i)) child 'final-optimistic)
       (tie-variables n (cons 'child-regressed-pessimistic (1+ i)) child 'final-pessimistic)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self, then pass control to child
;; with worst pessimistic estimate
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod compute-cycle ((n <sequence-node>))
  (debug-out :sequence-node 1 t "~&In compute cycle of sequence-node ~a with status ~a" (action n) (status n))
  (ecase (status n)
    (:initial (initialize-sequence-node n) (setf (status n) :ready))
    (:ready 
       (do-all-updates n)
       (compute-cycle (child (setf (next-child-to-refine n) (mod-inc (sequence-length n) (next-child-to-refine n))) n))
       (do-all-updates n))))

(defun sequence-node-child-evaluator (n)
  ;; Roughly speaking, we want to refine the node with the biggest gap between
  ;; optimistic and pessimistic boundsppp
  #'(lambda (i)
      (let ((c (child i n)))
	(my- (max-achievable-value (make-sum-valuation (current-value c 'progressed-pessimistic) (current-value c 'final-pessimistic)))))))


(defun initialize-sequence-node (n)
  (set-action-sequence (item 0 (refinements (action n) (hierarchy n) :init-opt-set (reachable-set (current-value n 'initial-optimistic)))) n)

  (let ((h (hierarchy n))
	(ref (action-sequence n))
	(l (sequence-length n)))

    ;; Add child output variables
    ;; They have dependants within the node only at the ends
    ;; We have to add all these variables before adding the child nodes because of cyclic dependencies
    (dotimes (i l)
      (add-variable n (cons 'child-progressed-optimistic i) :external :initial-value (make-simple-valuation (universal-set (planning-domain n)) 'infty)
		    :dependants (when (= i (1- l)) '(progressed-optimistic)))
      (add-variable n (cons 'child-progressed-pessimistic i) :external :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty)
		    :dependants (when (= i (1- l)) '(progressed-pessimistic)))
      (add-variable n (cons 'child-regressed-optimistic i) :external :initial-value (make-simple-valuation (universal-set (planning-domain n)) 'infty)
		    :dependants (when (zerop i) '(regressed-optimistic)))
      (add-variable n (cons 'child-regressed-pessimistic i) :external :initial-value (make-simple-valuation (empty-set (planning-domain n)) '-infty)
		    :dependants (when (zerop i) '(regressed-pessimistic))))

    ;; Add the child nodes themselves
    (do-elements (child-action ref nil i)
      (create-child-for-action h n i child-action))))

(defmethod action-node-type ((c (eql ':sequence)))
  '<sequence-node>)

(defun sequence-length (n)
  (length (action-sequence n)))
