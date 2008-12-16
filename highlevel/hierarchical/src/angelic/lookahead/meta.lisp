(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Metagreedy lookahead
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *ref-level-mult* .00001 "A crude way of tiebreaking towards higher level actions")

(defun metagreedy (hpp mean-var-fn num-refinements 
		   &key (known-values (make-known-values 'state #'same-state))
			(subsumption-checker nil) (state (init-state hpp))
			(prim-first-val 1.0))
  "metagreedy HPP MEAN-VAR-FN NUM-REFS &key KNOWN-VALUES SUBSUMPTION-CHECKER (PRIM-FIRST-VAL 1)"
  (reset-subsumption-checker subsumption-checker state hpp)
  (let ((tree (initial-tree  hpp :init-state state :subsumption-checker subsumption-checker
			     :known-values known-values :node-type '<mean-var-node> :prim-then-act nil
			     :node-initargs (list ':mean-var-fn mean-var-fn))))
    
    (setf *tree* tree)
    ;; Main computation loop
    ;; Do num-refinements refinements, then return the plan beginning with a primitive action
    ;; with the highest expected reward
    (dotimes (i num-refinements tree)
      (mvbind (plan val path) (get-best-plan tree ':any :q-fn #'mean-q)
	      (debug-print 4 "Tree is ~/tree:pprint-tree/" tree)
	      (debug-print 2 "Best plan is ~a with value ~a" plan val)

	      ;; Figure out value of refining first hla of best plan
	      (let ((first-hla-bonus (my* prim-first-val (/ i (* (- num-refinements i) (1- num-refinements))))))
	    
	  	   
		;; Figure out the best optimal-plan-action to refine
		(mvbind (k best-plan-val best-plan-node)
			(argmax 
			 (filter ':implicit path (fn (and (not root?) (not is-primitive-node))))
			 :key #'(lambda (n) (best-plan-edge-vpi tree plan n first-hla-bonus known-values)))
			(declare (ignore k))
	  
			;; Figure out the best suboptimal-plan-action to refine
			(mvbind (j nonbest-plan-val nonbest-plan-node )
				(argmax
				 (ndunion ((c (active-child-nodes tree)))
					  (unless (eq c (ssecond path))
					    (active-descendants c)))
				 :key #'nonbest-vpi)
				(declare (ignore j))
	      
				;; Eventually it often happens that all nonoptimal edges have VPI 0
				;; In this case, just use the highest level action of the second best plan as the nonoptimal option
				(when (my< nonbest-plan-val .00001)
				  (mvbind (plan v p) (best-plan-excluding-node tree (ssecond path) known-values)
					  (declare (ignore v))
					  (let ((n (minimizing-element (filter ':implicit p (fn (not root?))) 
								       #'(lambda (n) (ref-level hpp (action n))))))
					    (unless (is-primitive (action n) hpp)
					      (debug-print 2 "Because no good nonbest nodes seem available, just using highest-level action ~a of second best plan action ~a" (action n) plan)
					      (setf nonbest-plan-node n)))))
	    
				(when-debugging 2
						(debug-prompt "Best optimal action to refine is ~a with vpi ~0,3f~&Best suboptimal action to refine is ~a~&   from plan ~a with vpi ~0,3f"
							      best-plan-node best-plan-val nonbest-plan-node 
							      (when nonbest-plan-node (get-node-plan (slast (best-path-through nonbest-plan-node)))) nonbest-plan-val))

				(let* ((node-to-refine (if (my> best-plan-val nonbest-plan-val) best-plan-node nonbest-plan-node))
				       (path (best-path-through node-to-refine))
				       (plan (get-node-plan (slast path))))
				  (cond
				   ((is-primitive-node node-to-refine) 
				    (debug-print 2 "Not refining node ~a as it is primitive" node-to-refine))
				   ((subsumed (slast path)) 
				    (debug-print 2 "Not refining node ~a because plan ~a is subsumed"
						 node-to-refine plan))
				   (t (refine tree plan (1- (depth node-to-refine))
					      :subsumption-checker subsumption-checker :known-values known-values)))))))))))
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Agent
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun metagreedy-agent (hpp mean-var-fn num-refs 
			 &key (subsumption-checker nil) (prim-first-val 1.0)
			      (visit-count-multiplier 1))
  "metagreedy-agent HPP MEAN-VAR-FN NUM-REFS &key (SUBSUMPTION-CHECKER nil) (PRIM-FIRST-VAL 1.0) (VISIT-COUNT-MULTIPLIER 1)
Return an agent that takes in a state and returns best primitive action after doing metagreedy lookahead."
  (let ((known-values (make-known-values 'state #'same-state)))
    #'(lambda (s)
	(debug-print 3 "Known values :~& ~a" known-values)
	(let ((tree (metagreedy 
		     hpp mean-var-fn num-refs 
		     :subsumption-checker subsumption-checker :state s
		     :prim-first-val prim-first-val :known-values known-values)))
	  (mvbind (plan val) (get-best-plan tree ':begins-with-primitive :q-fn #'mean-q)

	    (debug-print 0 "Intending ~a with expected value ~a" plan val)
	    (debug-prompt 1 "")
	    
	    (when (zerop (length plan))
	      (setf val (nth-value 1 (get-best-plan tree ':any :q-fn #'mean-q))))

	    ;; Update known values
	    (let ((rec 
		   (if (zerop (length plan))
		       (let ((v (lookup-known-value 'state s 'mean-value known-values)))
			 (modify-known-value 'state s 'mean-value (if v (- v visit-count-multiplier) val)
					     'replace known-values))
		     (modify-known-value 'state s 'mean-value val 'min known-values))))
	      (modify-known-value-rec rec 'known t 'replace)
	      (modify-known-value-rec rec 'visit-penalty visit-count-multiplier #'(lambda (x y) (+ (or x 0) y))))
	    
	    (if (zerop (length plan))
		(progn
		  (warn "Tree did not include primitive actions, so sampling randomly")
		  (sample-uniformly (avail-actions hpp s)))
	      (sfirst plan)))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node type
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <mean-var-node> (<node-with-action-rewards>)
  ((mean-var-fn :initarg :mean-var-fn :reader mean-var-function :initarg :mean-var-fn)
   (mean :accessor mean :initform nil)
   (var :accessor var :initform nil)
   (dist :accessor dist)   
   (mean-q :accessor mean-q :initform '-infty)))

(defun mean-var-fn (n)
  (mean-var-function (root n)))

(defun mean-value (n)
  (let ((c (active-child-nodes n)))
    (cond
     ((subsumed n) '-infty)
     ((is-empty c) 0.0)
     (t (reduce-set #'mymax c :key #'mean-q)))))

(defmethod update-forward :after ((n <mean-var-node>) parent)
  (declare (ignore parent))
  (with-accessors ((cr complete-reward) (sr sound-reward)) n
    (mvbind (new-mean new-var) (funcall (mean-var-fn n) (action n) cr sr)
      (when (change-if-necessary (mean n) new-mean)
	(push 'mean (changed n)))
      (when (change-if-necessary (var n) new-var)
	(push 'var (changed n)))
      (when (changed n)
	(setf (dist n) 
	  (make-instance '<truncated-gaussian> :a sr :b cr :mean new-mean :std (sqrt (var n))))))))

(defmethod update-backward :after ((l <mean-var-node>) known-values)
  (unless (root? l)
    (let (new-q)
      (cond
       ((subsumed l) (setf new-q '-infty))
       ((known-value l) (setf new-q 
			  (my+ (mean l) (lookup-known-value 'state (check-not-null (state l)) 'mean-value known-values))))
       ((member (action l) '(act finish)) (setf new-q (mean l)))
       (t (let ((best (maximizing-element (active-child-nodes l) #'mean-q)))
	    (setf new-q (my+ (mean l) (mean-q best))))))
      (when (change-if-necessary (mean-q l) new-q)
	(push 'mean-q (changed l))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Helper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun vpi (dist mean2)
  (let ((m (expectation dist #'identity)))
    (if (or (infinite m) (infinite mean2))
	0
      (let* ((int (make-instance '<interval> :a mean2))
	     (p (probability dist int)))
	(if (my< p .00001) ;; Avoid conditioning on low-probability event
	    0.0
	  (my-
	   (avg mean2 (conditional-expectation dist int) p)
	   (mymax mean2 m)))))))

(defun best-plan-excluding-node (tree node known)
  ;; A slightly hacky way of doing it: modify the q-values in the tree
  ;; to make this node seem very bad then figure out the best plan.
  ;; An unwind protect guarantees to put things back in order afterwards.
  (let ((saved-mean (mean node))
	(*debug-level* 0)) ;; To avoid debug messages being printed during backups
    (unwind-protect 
	(progn
	  (setf (mean node) '-infty
		(out-of-date-backward-flag node) t)
	  (update-nodes-backward node known)
	  (get-best-plan tree ':any :q-fn #'mean-q))
      (setf (mean node) saved-mean
	    (out-of-date-backward-flag node) t)
      (update-nodes-backward node known))))

(defun best-path-through (n)
  (let ((node n)
	(p (make-array 0 :adjustable t :fill-pointer 0)))
    (loop
      (let ((c (active-child-nodes node)))
	(if (is-empty c)
	    (return nil)
	  (let ((best (maximizing-element c #'mean-q)))
	    (vector-push-extend best p)
	    (setf node best)))))
    (concatenate 'vector (get-path-from-root n) p)))
	
      

(defun best-plan-edge-vpi (tree best-plan n first-hla-bonus known)
  (if (is-primitive (action n) (hpp n))
      '-infty
    (let ((first-action (root? (parent-node n)))
	  (starts-with-primitive (is-primitive (sfirst best-plan) (hpp n))))
      (my+
       (iwhen first-action first-hla-bonus)
       (let ((second-best-val 
	      (nth-value 
	       1 (best-plan-excluding-node 
		  tree
		  (if starts-with-primitive
		      (plan-final-node tree `#(,(sfirst best-plan)))
		    n)
		  known))))
	 (vpi (dist n) 
	      (my+ second-best-val (mean n) 
		   (- (mean-q (plan-final-node tree (subseq best-plan 0 1)))))))))))

(defun nonbest-vpi (n &aux (tree (root n)))
  (if (is-primitive (action n) (hpp n))
      '-infty
    (let ((best-val (mean-value tree)))
      (vpi (dist n) 
	   (my- best-val
		(sum-over (cdr (get-path-from-root n))
			  #'(lambda (node) (iunless (eq node n) (mean node))))
		(mean-value n))))))
    
    



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((l <mean-var-node>) str)
  (if (eq (status l) 'active)
      (print-unreadable-object (l str :type nil :identity nil)
	(format str "~a.  ~:[~*~;Up: ~a.  ~]Bounds: [~a, ~a].  Mean: ~0,1f. Std: ~0,1f. Q: ~0,1f. ~:[~;Subsumed.~]~:[~; Known~]"
		(action l) (upward-parent l) (awhen (upward-parent l) (action it))
		(sound-reward l) (complete-reward l)
		(mean l) (awhen (var l) (sqrt it)) (mean-q l)
		(subsumed l) (known-value l)))
    (print-unreadable-object (l str :type nil :identity nil)
      (format str "~a (inactive)" (action l)))))

