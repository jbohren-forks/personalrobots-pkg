(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hierarchical forward search
;;
;; Debug levels
;; 0. Do not print anything or pause
;; 1. Print info about each call to fgp and decompose
;; 2. Print each plan that is considered in find-guaranteed-plan
;; 3. Print rewards and sets of each plan in fgp, and pause
;; 4. Print priority queue at each step
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun hierarchical-forward-search (hpp r priority-fn &key (state (init-state hpp)) (subsumption-checker nil)
							   (plan-type ':primitive))
  "hierarchical-forward-search HPP REWARD-THRESHOLD PRIORITY-FN &key (STATE (INIT-STATE HPP)) (SUBSUMPTION-CHECKER NIL) (PLAN-TYPE ':primitive)) 

Does hierarchical offline satisficing planning: returns an executable plan compatible with the hierarchy that soundly achieves reward strictly greater than REWARD-THRESHOLD, if one exists, or ':no-plan-found otherwise.  Returns the tree as a secondary value.  The definition of executable depends on PLAN-TYPE, which can be ':primitive, ':begins-with-primitive, or ':any-plan.

PRIORITY-FN is a function that takes in an action, complete reward and sound reward, and returns a single number."
  (assert (not subsumption-checker) nil "Subsumption checker argument currently not supported")
  (let ((tree (initial-tree hpp :prim-then-act nil :init-state state))
	(*num-extensions* 0))
    (find-guaranteed-plan hpp tree #(act) r '(terminal) priority-fn plan-type t)))
  



(defun find-guaranteed-plan (hpp tree plan r s priority-fn plan-type strict &aux (n (length plan)))
  "find-guaranteed-plan HPP TREE PLAN R S PRIORITY-FN PLAN-TYPE STRICT

If there exists a plan PLAN' such that
1) The first N-1 actions of PLAN' are the same as PLAN, where N = length (PLAN)
2) The remaining suffix is obtained by remaining PLAN_N at least once
3) PLAN' soundly achieves state S with reward at least R (or greater than R if strict is t)
4) PLAN' satisfies PLAN-TYPE, which can be one of ':primitive, ':begins-with-primitive, or ':any-plan
then return one such PLAN', and guarantee that TREE contains PLAN'.

Otherwise, return nil."

  (debug-print 0 "Looking for refinement of last action of plan ~a,  of type ~a, that achieves ~a and gets to~&  ~a" plan plan-type r s)
  (let ((q (pqueue:make-priority-queue))
	(comp-fn (if strict #'my< #'my<=)))

    (flet ((add-all-refinements (p i)
	     (dolist (node (refine tree p i hpp))
	       (let* ((plan3 (get-node-plan node))
		      (pri (plan-priority tree plan3 priority-fn)))
		 (when (my> pri '-infty) 
		   (pqueue:enqueue q plan3 pri))))))

      (add-all-refinements plan (1- n))
      (with-debug-indent
	  (loop
	    (debug-print 3 "Priority queue is ~a" q)
	    (if (zerop (pqueue:num-entries q))
		(return-from find-guaranteed-plan nil)

	      (let ((plan2 (pqueue:dequeue-highest q)))
		(mvbind (path edges) (get-plan-path tree plan2)
		  (let ((first-node-label (node-label (aref path (1- n))))
			(last-node-label (node-label (slast path))))
		    (let ((sound-reward (my- (sound-reward-upto last-node-label) (sound-reward-upto first-node-label)))
			  (complete-reward (my- (complete-reward-upto last-node-label) (complete-reward-upto first-node-label)))
			  (sound-set (sound-set last-node-label))
			  (complete-set (complete-set last-node-label)))

		      (debug-print 1 "Considering plan ~a with priority ~a.~&~a extensions so far." 
				   plan2 (plan-priority tree plan2 priority-fn) *num-extensions*)
		      (debug-prompt 2 "Complete reward : ~a~&Sound reward : ~a~&Complete set : ~a~&Sound set : ~a"
				    complete-reward sound-reward complete-set sound-set)
		      (cond
	       
		       ((not (and (funcall comp-fn r complete-reward) (intersects complete-set s)))
			(debug-print 1 "Does not succeed completely"))
	       
		       ((and (funcall comp-fn r sound-reward) (intersects sound-set s))
			(debug-print 1 "Succeeds soundly")
			(return-from find-guaranteed-plan
			  (ecase plan-type
			    (:any-plan (values plan2 sound-reward))
			    ((:primitive :begins-with-primitive)
			     (decompose-plan plan2 (1- n) hpp tree plan-type s priority-fn)))))
	       
		       (t
			(let ((ind (choose-ind-to-refine edges hpp (1- n))))
			  (debug-print 1 "Succeeds completely but not soundly")
			  (add-all-refinements plan2 ind))))))))))))))



(defun decompose-plan (plan start hpp tree plan-type goal priority-fn &aux (n (- (length plan) start)))
  "decompose-plan PLAN START LENGTH HPP TREE PLAN-TYPE GOAL PRIORITY-FN 

Returns a plan PLAN' which is obtained by repeatedly refining the suffix of PLAN starting at action START, such that PLAN' reaches GOAL and achieves the sound reward guaranteed by PLAN.  Also guarantees that the tree contains said PLAN'."
  (debug-print 0 "Decomposing plan ~a, starting at position ~a, into plan of type ~a" plan start plan-type)
  (with-debug-indent
      (mvbind (nodes edges) (get-plan-path tree plan)
	(when (or (zerop n) (and (eq plan-type ':begins-with-primitive) (is-primitive (sfirst plan) hpp)))
	  (return-from decompose-plan (values plan (sound-reward-upto (node-label (slast nodes))))))
    
	(flet ((sound-set (i) (sound-set (node-label (aref nodes (+ i start)))))
	       (sound-reward (i) (sound-reward (edge-label (aref edges (+ i start)))))
	       (action (i) (action (edge-label (aref edges (+ i start))))))
	  (let ((subgoals (make-array n)))

	    ;; The last subgoal is the intersection of the goal with the last sound set
	    (setf (aref subgoals (1- n)) (intersect goal (sound-set n)))
	
	    ;; Work backwards to find the subgoals at each step
	    (for-loop (i (1- n) 1 -1 #'<)
	      (setf (aref subgoals (1- i)) 
		(regress-reward (sound-set i) (aref subgoals i) (sound-reward i) (sound-desc hpp (action i)))))
	    
	    (debug-print 3 "The subgoals are ~a" subgoals)
	    (ecase plan-type
	      (:begins-with-primitive
	       (assert (zerop start))
	       (concatenate 
		   'vector
		 (find-guaranteed-plan hpp tree (subseq plan 0 1) (sound-reward 0) (aref subgoals 0) 
				       priority-fn ':begins-with-primitive nil)
		 (subseq plan 1)))
	      (:primitive
	       (let ((p (subseq plan 0 start)))
		 (dotimes (i n)
		   (let ((a (aref plan (+ start i))))
		     (setf p (concatenate 'vector p (list a)))
		     (unless (is-primitive a hpp)
		       (add-path-to-tree p tree hpp)
		       (setf p (find-guaranteed-plan 
				hpp tree p
				(sound-reward i) (aref subgoals i)
				priority-fn ':primitive nil)))))
		 (add-path-to-tree p tree hpp)
		 (values p (sound-reward-upto (node-label (slast (get-plan-path tree p)))))))))))))

	    
	   
