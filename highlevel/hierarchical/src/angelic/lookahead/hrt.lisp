;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; lookahead/lookahead/hrt.lisp
;; Hierarchical Real Time algorithm and associated agent
;; This is based on the algorithm from the NIPS07 talk
;; 
;; Debug levels
;; 0 - do not print anything or pause
;; 1 - Print intended plan at each env step
;; 2 - Pause and validate tree after each env step.  Print each plan considered, 
;;     and periodically print the best one so far.
;; 3 - Print each new plan added and its priority.  Pause and validate tree after each call to refine.
;; 4 - Print known values after each step
;; 5 - Print tree after each refinement
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)



(defvar *print-best-plan-inc* 5)
(defparameter *default-prim-search-exp* 2)
(defparameter *act-q-weight* .99)
(defparameter *default-visit-count-multiplier* 2)
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node type
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <hrt-node> (<node-with-priority>)
  ((act-priority-q :accessor act-priority-q :initform '-infty)
   (visit-count-mult :initarg :visit-count-mult :accessor visit-count-multiplier))
  (:documentation "Class <hrt-node>
Subclass of <node-with-priority> that also keeps track of the best action w.r.t a modified priority function that only cares about the final Act."))

(defmethod update-forward :after ((n <hrt-node>) parent)
  (setf (visit-count-multiplier n) (visit-count-multiplier parent)))

(defmethod update-backward :after ((n <hrt-node>) known-values)
  (unless (root? n)
    (let ((new-q
	   (cond
	    ((member (action n) '(act finish)) (priority n))
	    ((known-value n) 
	     (let ((rec (known-value-mapping 'state (check-not-null (state n)) known-values)))
	       (my- (mapping:evaluate rec 'act-priority)
		    (* (visit-count-multiplier n) (mapping:evaluate rec 'visit-count)))))
	    ((subsumed n) '-infty)
	    (t (let ((children (active-child-nodes n)))
		 (assert (not (is-empty children)) nil "Unknown, nonsubsumed node ~a had no children" n)
		 (reduce-set #'mymax children :key #'act-priority-q))))))
      (when (change-if-necessary (act-priority-q n) new-q)
	(push 'act-priority-q (changed n))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hrt alg
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun hierarchical-real-time (hpp priority-fn max-num-refinements
			       &key (state (init-state hpp)) (subsumption-checker nil) 
				    (prim-search-exp *default-prim-search-exp*)
				    (known-values (make-known-values 'state #'same-state))
				    (visit-count-multiplier *default-visit-count-multiplier*))
  "hierarchical-real-time HPP PRIORITY-FN MAX-NUM-REFINEMENTS &key (STATE (INIT-STATE HPP)) (KNOWN-VALUES NIL) (SUBSUMPTION-CHECKER NIL) (prim-search-exp *default-prim-search-exp*) (VISIT-COUNT-MULTIPLIER *default-visit-count-multiplier*)

HPP - hierarchical planning problem
PRIORITY-FN - function that takes in action name, sound reward, and complete reward, and returns an extended real, where higher values mean the corresponding plan will be refined earlier.
PRIM-SEARCH-EXP - a positive real number.  At step k, with prob (k/num-ref)^PSE, the first action is refined is possible.  Default is 2.

Carries out hierarchical real time search for at most MAX-NUM-REFINEMENTS refinements, and returns the tree."
  
  ;; Initialization
  (reset-subsumption-checker subsumption-checker state hpp)
  (assert (null *prim-then-act*))
  (let ((tree (initial-tree hpp :prim-then-act nil :init-state state :known-values known-values 
			    :subsumption-checker subsumption-checker :node-type '<hrt-node> 
			    :node-initargs (list ':priority-fn priority-fn ':visit-count-mult visit-count-multiplier)))
	(removed-plans nil))
    (setf *tree* tree)
    
    (flet ((main-loop-debug (i)
	     (debug-print 1 "~:[~;~&Best plan is now ~a~]" 
			  (divisible-by i *print-best-plan-inc*) (hrt-best-plan tree))
	     (debug-prompt 2)
	     (debug-prompt 4 "Tree is ~/tree:pprint-tree/" tree)))
    
      ;; Main loop over computational cycles
      (dotimes (i max-num-refinements)
	(mvbind (plan pri path) (get-best-plan tree ':any :q-fn #'priority-q)
	  (declare (ignore pri))
	  (when (zerop (length plan)) (return nil))
	  (let ((ind (hrt-choose-ind path hpp i max-num-refinements prim-search-exp)))
	    (if (is-primitive (aref plan ind) hpp)
		(progn
		  (debug-print 1 "~a.  Not refining plan ~a further." i plan)
		  (push plan removed-plans)
		  (remove-path-from-tree plan tree known-values))
	      (progn
		(refine tree plan ind :subsumption-checker subsumption-checker :known-values known-values)
		(main-loop-debug i))))))
      
      ;; Add back any removed plans and return tree
      (dolist (p removed-plans tree)
	(add-path-to-tree p tree :known-values known-values :subsumption-checker subsumption-checker)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hrt agent
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun hrt-agent (hpp priority-fn num-refs 
		  &key (subsumption-checker nil) (prim-search-exp *default-prim-search-exp*)
		       (visit-count-multiplier *default-visit-count-multiplier*))
  "hrt-agent HPP PRIORITY-FN NUM-REFS &key (SUBSUMPTION-CHECKER nil) (PRIM-SEARCH-EXP *default-prim-search-exp*) (VISIT-COUNT-MULTIPLIER 2)
Return an agent that takes in a state and returns best primitive action after doing hierarchical-real-time lookahead."
  (let ((known-values (make-known-values 'state #'same-state)))
    #'(lambda (s)
	(debug-print 3 "Known values :~& ~a" known-values)
	(let ((tree (hierarchical-real-time 
		     hpp priority-fn num-refs :subsumption-checker subsumption-checker :state s
		     :prim-search-exp prim-search-exp :known-values known-values
		     :visit-count-multiplier visit-count-multiplier)))
	  (mvbind (plan val) (hrt-best-plan tree)
	    
	    ;; Update known values
	    (let* ((best-priority (nth-value 1 (get-best-plan tree ':any :q-fn #'priority-q)))
		   (rec (modify-known-value 'state s 'priority best-priority 'min known-values)))
	      (modify-known-value-rec rec 'act-priority val 'min)
	      (modify-known-value-rec rec 'known t 'replace)
	      (modify-known-value-rec rec 'visit-count 1 #'(lambda (x y) (+ (or x 0) y))))
	    
	    (debug-print 0 "Intending ~a with act-priority ~a" plan val)
	    (debug-prompt 1 "")
	    (if (zerop (length plan))
		(progn
		  (warn "Tree did not include primitive actions, so sampling randomly")
		  (sample-uniformly (avail-actions hpp s)))
	      (sfirst plan)))))))
  
  



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun hrt-best-plan (tree)
  ;; We want to select plans based on act-q, but also include a small constant times priority-q for tiebreaking.
  (get-best-plan tree ':begins-with-primitive 
		 :q-fn #'(lambda (n) (avg (priority-q n) (act-priority-q n) *act-q-weight*))))


(defun hrt-choose-ind (path hpp i max-i prim-search-exp)
  "hrt-choose-ind PATH HPP I MAX-I PRIM-SEARCH-EXP"
  (if (and (< (random 1.0) (expt (/ i max-i) prim-search-exp))
	   (not (is-primitive (action (ssecond path)) hpp)))
      0
    (flet ((gap (i)
	     (let ((n (aref path i)))
	       (if (is-primitive (action n) hpp)
		   '-infty ;; A hack: it should be 0, but we want to make sure it doesn't ever get selected by tiebreak
		 (my- (complete-reward n) (sound-reward n)))))
	   (ref-level (i) 
	     (let ((a (action (aref path (1+ i)))))
	       (if (is-primitive a hpp)
		   'infty
		 (ref-level hpp a)))))
      (let ((best-gaps (sort (nth-value 3 (argmax (int-range 1 (length path)) :key #'gap)) #'<)))
	(nth-value 2 (argmin best-gaps :key #'ref-level))))))
    
    
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((n <hrt-node>) str)
  (print-unreadable-object (n str :type nil :identity nil)
    (format str "~a~:[ (inactive)~5*~;.  [~a, ~a].  Priority: ~a.  PQ=~a.  APQ=~a.~]  ~:[~;OODF. ~]~:[~;OODB~]~:[~;Subsumed. ~]~:[~;Known. ~]"
	    (action n) (active n) (sound-reward n) (complete-reward n)
	    (priority n) (priority-q n) (act-priority-q n) (out-of-date-forward-flag n) (out-of-date-backward-flag n)
	    (subsumed n) (known-value n))))

