(in-package :vb-node)

(define-debug-topic :sequence-node :vb-node)

(defclass <sequence-node> (<node>)
  ((action-sequence :reader action-sequence :writer set-action-sequence)
   (status :initform :initial)))

(defun sequence-length (n)
  (length (action-sequence n)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The only time child vars have an explicit dependant is 
;; at the ends of the sequence 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod child-progressed-optimistic-dependants ((n <sequence-node>) i)
   (when (= (1- (sequence-length n)) i)
     '(children-progressed-optimistic)))

(defmethod child-progressed-pessimistic-dependants ((n <sequence-node>) i)
   (when (= (1- (sequence-length n)) i)
     '(children-progressed-pessimistic)))

(defmethod child-regressed-optimistic-dependants ((n <sequence-node>) i)
   (when (zerop i)
     '(children-regressed-optimistic)))

(defmethod child-regressed-pessimistic-dependants ((n <sequence-node>) i)
   (when (zerop i)
     '(children-regressed-pessimistic)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; The node's outputs are computed from the node's 
;; own progressed/regressed valuations together with the
;; final/initial child's output valuation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod optimistic-progressor ((n <sequence-node>))
  (make-simple-alist-updater (my-progressed-optimistic children-progressed-optimistic)
    (pointwise-min-upper-bound my-progressed-optimistic children-progressed-optimistic)))

(defmethod pessimistic-progressor ((n <sequence-node>))
  (make-simple-alist-updater (my-progressed-pessimistic children-progressed-pessimistic)
    (pointwise-max-lower-bound my-progressed-pessimistic children-progressed-pessimistic)))


(defmethod optimistic-regressor ((n <sequence-node>))
  (make-simple-alist-updater (my-regressed-optimistic children-regressed-optimistic)
    (pointwise-min-upper-bound my-regressed-optimistic children-regressed-optimistic)))

(defmethod pessimistic-regressor ((n <sequence-node>))
  (make-simple-alist-updater (my-regressed-pessimistic children-regressed-pessimistic)
    (pointwise-max-lower-bound my-regressed-pessimistic children-regressed-pessimistic)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compute cycle: update self, then pass control to child
;; with worst pessimistic estimate
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod compute-cycle ((n <sequence-node>))
  (debug-out :sequence-node 1 t "In compute cycle of sequence-node ~a with status ~a" (action n) (status n))
  (ecase (status n)
    (:initial (initialize-sequence-node n))
    (:ready (do-all-updates n)
	    (let ((i (maximizing-element (child-ids n) (sequence-node-child-evaluator n))))
	      (compute-cycle (child i n)))
	    (do-all-updates n))))

(defun sequence-node-child-evaluator (n)
  (declare (ignore n))
  #'(lambda (i)
      (format t "~&Stub: Enter value for child ~a of sequence node: " i)
      (read)))

(defmethod action-node-type ((c (eql ':sequence)))
  '<sequence-node>)


(defun initialize-sequence-node (n)
  "Called the first time a sequence node gets to run.  This creates all the child nodes and sets up communication"

  (set-action-sequence (item 0 (refinements (action n) (hierarchy n) :init-opt-set (reachable-set (current-value n 'initial-optimistic)))) n)

  ;; Add child nodes
  (let ((h (hierarchy n))
	(ref (action-sequence n)))

    (do-elements (child-action ref nil i)
      (create-child-for-action h n i child-action))
    (set-action-sequence ref n)

    (let ((length (sequence-length n)))

      ;; Outputs of the first and last children
      (add-variable n 'children-progressed-optimistic :internal :update-fn #'copier :dependants '(progressed-optimistic) :dependees (list (cons 'child-progressed-optimistic (1- length))))
      (add-variable n 'children-progressed-pessimistic :internal :update-fn #'copier :dependants '(progressed-pessimistic) :dependees (list (cons 'child-progressed-pessimistic (1- length))))
      (add-variable n 'children-regressed-optimistic :internal :update-fn #'copier :dependants '(regressed-optimistic) :dependees (list (cons 'child-regressed-optimistic 0)))
      (add-variable n 'children-regressed-pessimistic :internal :update-fn #'copier :dependants '(regressed-pessimistic) :dependees (list (cons 'child-regressed-pessimistic 0)))

      (dotimes (i (sequence-length n))
	(let ((child (child i n)))

	  ;; Forward communication between children
	  (cond 
	    ((zerop i)
	     (tie-variables n 'initial-optimistic child 'initial-optimistic)
	     (tie-variables n 'initial-pessimistic child 'initial-pessimistic))
	    (t
	     (tie-variables n (cons 'child-progressed-optimistic (1- i)) child 'initial-optimistic)
	     (tie-variables n (cons 'child-progressed-pessimistic (1- i)) child 'initial-optimistic)))

	  ;; Backward communication between children
	  (cond
	    ((= i (1- (sequence-length n)))
	     (tie-variables n 'final-optimistic child 'final-optimistic)
	     (tie-variables n 'final-pessimistic child 'final-pessimistic))
	    (t
	     (tie-variables n (cons 'child-regressed-optimistic (1+ i)) child 'final-optimistic)
	     (tie-variables n (cons 'child-regressed-pessimistic (1+ i)) child 'final-pessimistic)))))

      (setf (status n) :ready))))


