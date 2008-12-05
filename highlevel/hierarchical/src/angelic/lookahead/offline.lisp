(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Planning algorithms
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun aha* (hpp &key (state (init-state hpp))
		      (max-num-refinements 'infty) (termination ':primitive)
		      (valid-plan-pred (constantly t))
	     &aux (known-values (make-known-values 'state #'same-state)))
 		      
  "aha* HPP &key (STATE (init-state hpp)) (MAX-NUM-REFINEMENTS  'infty) (TERMINATION ':primitive)(VALID-PLAN-PRED (constantly t)))

Angelic hierarchical A* as described in the offline submission.

TERMINATION can be ':primitive, ':begins-with-primitive, or ':any

Returns 1) A provably optimal plan of the appropriate type, or nil if time runs out 2) The lookahead tree."
  
  (reset-subsumption-checker (subsumption-checker hpp) state hpp)
  (let ((tree (initial-tree hpp :init-state state :known-values known-values 
			    :node-type '<node-with-action-rewards>))
	(i 0))
    (setf *tree* tree)
    
    ;; Main computation loop
    (until (my> (incf i) max-num-refinements)
      (mvbind (plan val path) (get-best-plan tree ':any :q-fn #'complete-q)
	
	(debug-prompt 2 "Considering plan ~a with complete value ~a" plan val)
	
	;; 1. Return if a primitive plan is found
	(when (is-primitive-plan plan hpp)
	  (return-from aha* (values plan tree)))
	
	;; 2. Skip invalid plans
	(if (not (funcall valid-plan-pred plan))
	    (remove-path-from-tree plan tree known-values :update-backward t)
	
	  (let ((sound-val (sound-q (aref path 1)))
		(second-best-comp-val (second-best-complete-val path)))
	  
	    ;; 2. When a provably best plan is found
	    (when (my>= sound-val second-best-comp-val)
	    
	      ;; Return if plan-type allows it
	      (if (or (and (eq termination ':any) (eql sound-val val))
		      (and (eq termination ':begins-with-primitive) (is-primitive (aref plan 0) hpp)))
		  (return-from aha* (values plan tree))
	      
		;; Else, commit to this plan by removing all others
		(progn
		  (debug-print 2 "Committing to ~a as its sound value ~a is >= second best complete value ~a" 
			       plan sound-val second-best-comp-val)
		  (remove-all-paths-except path))))
	  
	    ;; 3. Refine the current plan
	    (refine tree plan (choose-ind path hpp) :known-values known-values)))))
    
    ;; Case when optimal plan is not found in time
    (values nil tree)))



(defun ahss (hpp r &key (state (init-state hpp)) (termination ':primitive) (valid-plan-pred (constantly t))
			(priority-fn #'(lambda (a c s) (declare (ignore a s)) c))
	     &aux (known-values (make-known-values 'state #'same-state)) (checker (subsumption-checker hpp)))
  "ahss HPP REWARD-THRESHOLD &key (STATE (init-state hpp)) (TERMINATION ':primitive)(VALID-PLAN-PRED (constantly t)) (PRIORITY-FN complete-reward)

Angelic hierarchical satisficing search. as described in the offline submission.

TERMINATION can be ':primitive, ':begins-with-primitive, or ':any

Returns 1) A plan of the appropriate type that has guaranteed reward > the threshold, or nil if no such plan exists 2) The lookahead tree."
  
  (reset-subsumption-checker checker state hpp)
  (let ((tree (initial-tree hpp :init-state state :known-values known-values 
			    :node-type '<node-with-priority> :node-initargs (list ':priority-fn priority-fn)))
	(i 0))
    (setf *tree* tree)
    
    (loop
      ;; 1. If guaranteed not to exist a reasonable plan, quit
      (when (my<= (nth-value 1 (get-best-plan tree ':any :q-fn #'complete-q)) r)
	(return (values nil tree)))
      
      (mvbind (sound-best-plan sound-val) (get-best-plan tree ':any :q-fn #'sound-q)

	;; 2. When a sufficiently good guaranteed plan is found
	(when (my> sound-val r)
	  
	  ;; a) Return it if possible
	  (if (or (eq termination ':any)
		  (and (eq termination ':begins-with-primitive) (is-primitive (sfirst sound-best-plan) hpp))
		  (is-primitive-plan sound-best-plan hpp))
	      (return-from ahss (values sound-best-plan tree))
	    
	    ;; b) Else commit to the complete best one that is above threshold
	    (mvbind (plan val path)
		(get-best-plan 
		 tree ':any
		 :q-fn #'(lambda (n)
			   (if (my> (my+ (sound-q n) (max-achievable-value (sound-valuation (parent-node n)))) r)
			       (complete-q n)
			     '-infty)))
	      
	      (when (my> val r)
		(debug-print 2 "Committing to ~a with complete value ~a" plan val)
		(remove-all-paths-except path)
		)))))
      
      ;; 3. Otherwise, refine the highest priority plan
      (mvbind (plan val path) (get-best-plan tree ':any :q-fn #'priority-q)
	
	(debug-prompt 2 "~a.  Considering plan ~a with priority ~a" (incf i) plan val)
	
	;; Skip invalid plans (todo remove, as hierarchy now does this)
	(if (not (funcall valid-plan-pred plan))
	    (remove-path-from-tree plan tree known-values :update-backward t)
	  
	  (refine tree plan (choose-ind path hpp) :known-values known-values))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun second-best-complete-val (path)
  (let ((v '-infty))
    (dotimes (i (1- (length path)) v)
      (let ((node (aref path i))
	    (child (aref path (1+ i)))
	    (s 0))
	(do-elements (c2 (active-child-nodes node))
	  (unless (eq child c2)
	    (_f mymax v (my+ s (complete-q c2)))))
	(_f my+ s (complete-reward child))))))

(defun remove-all-paths-except (path)
  (dotimes (i (1- (length path)))
    (let ((node (aref path i))
	  (child (aref path (1+ i))))
      (do-elements (c2 (active-child-nodes node))
	(unless (eq c2 child)
	  (make-subtree-inactive c2))))))


(defun choose-ind (path hpp)
  (argmax 
   (srest path)
   :key #'(lambda (n) (if (is-primitive (action n) hpp) '-infty (my- (complete-reward n) (sound-reward n))))
   :tiebreak-keys (list #'(lambda (n) (my- (ref-level hpp (action n)))))))
	
	  
	  
	  
	  
  
