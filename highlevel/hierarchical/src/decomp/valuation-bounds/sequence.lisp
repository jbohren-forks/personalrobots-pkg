(in-package :vb-node)

(define-debug-topic :sequence-node :vb-node)

(defclass <sequence-node> (<node>)
  ((action-sequence :reader action-sequence :writer set-action-sequence)
   (next-child-to-refine :accessor next-child-to-refine :initform -1)
   (child-inc :initform 1 :accessor child-inc)
   (status :initform :initial)))

;; We don't have an initialize-instance :after method to add the node's variables
;; This is because the set of variables depends on what the refinement is, and we can't compute this until
;; we know what the initial valuation is.  So initialization is done in initialize-sequence-node.



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Adding children
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod add-child :after ((n <sequence-node>) i new-node-type &rest args)
  (declare (ignore new-node-type args))

  (let ((child (child i n)))
    ;; Output variables already created in initialize-sequence-node, so just tie them
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
;; Extracting plans
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod primitive-plan-with-pessimistic-future-value-above ((n <sequence-node>) s v)
  (let ((plan-so-far (make-adjustable-array)) (total-reward 0))
    (dotimes (i (sequence-length n) (values plan-so-far s total-reward))
      (let ((child (child i n)))
	(mvbind (plan successor reward) (primitive-plan-with-pessimistic-future-value-above child s (my- v total-reward))
	  (unless plan (return nil))
	  (append-to-adjustable-array plan-so-far plan)
	  (setq s successor)
	  (_f my+ total-reward reward))))))

    

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
       (with-accessors ((ind next-child-to-refine) (inc child-inc)) n
	 (when (= ind 0) (setf inc 1))
	 (when (= ind (1- (sequence-length n))) (setf inc -1))
	 (incf ind inc)
	 (compute-cycle (child ind n)))
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
	(l (sequence-length n))
	(descs (descs n)))

    ;; Add child output variables
    ;; They have dependants within the node only at the ends
    ;; We have to add all these variables before adding the child nodes because of cyclic dependencies
    (dotimes (i l)

      ;; When the child is created, it has to know it's initial optimistic set, because that's what it uses to decide
      ;; what its refinements are.  So we precompute this here.
      (let ((init-progressed-opt (if (zerop i)
				     (current-value n 'initial-optimistic)
				     (current-value n (cons 'child-progressed-optimistic (1- i))))))
	(add-variable n (cons 'child-progressed-optimistic i) :external :dependants (when (= i (1- l)) '(progressed-optimistic))
		      :initial-value (progress-optimistic descs (elt ref i) init-progressed-opt)))

      ;; Everything else gets a default initial value
      (add-variable n (cons 'child-progressed-pessimistic i) :external :initial-value (minimal-valuation descs)
		    :dependants (when (= i (1- l)) '(progressed-pessimistic)))
      (add-variable n (cons 'child-regressed-optimistic i) :external :initial-value (maximal-valuation descs)
		    :dependants (when (zerop i) '(regressed-optimistic)))
      (add-variable n (cons 'child-regressed-pessimistic i) :external :initial-value (minimal-valuation descs)
		    :dependants (when (zerop i) '(regressed-pessimistic))))

    ;; Add the child nodes themselves
    (do-elements (child-action ref nil i)
      (create-child-for-action h n i child-action))))

(defmethod action-node-type ((c (eql ':sequence)))
  '<sequence-node>)

(defun sequence-length (n)
  (length (action-sequence n)))