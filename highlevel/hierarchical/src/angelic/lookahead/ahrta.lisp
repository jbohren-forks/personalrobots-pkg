(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types, special vars
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <ahrta*-node> (<node-with-priority>)
  ((noncached-priority-q :accessor noncached-priority-q :initform '-infty)
   (act-priority-q :accessor act-priority-q :initform '-infty))
  (:documentation "Class <ahrta*-node> (<node-with-priority>)

Node type used by the ahrta* algorithm.  

Noncached priority-q: the priority-q, except does not consider plans that end in a known node.  This is used when selecting the highest priority noncached plan for refinement.

Act priority-q: Act priority of a plan is the priority of the final Act or Finish only (-infty for known plans).  A small multiple (by *act-priority-mult*) of priority-q is added on for tiebreaking"))

(defparameter *act-priority-mult* .000001)


(defmethod update-backward :after ((n <ahrta*-node>) known-values)
  (declare (ignore known-values))
  (unless (root? n)
    (let ((children (active-child-nodes n)))
    
      ;; noncached-priority-q
      (let ((new-q (my+ (priority n)
			(if (is-empty children)
			    (cond
			     ((known-value n)
			      (if (eq (state n) 'terminal) 0 '-infty))
			     ((subsumed n) '-infty)
			     (t (assert (member (action n) '(act finish)) nil
				  "At leaf node with unknown state ~a, action was ~a instead of act or finish"
				  (state n) (action n))
				0.0))
			  (reduce-set #'mymax children :key #'noncached-priority-q)))))
	(when (change-if-necessary (noncached-priority-q n) new-q)
	  (push 'noncached-priority-q (changed n))))
      
      ;; act-priority-q
      (let ((new-q (if (is-empty children)
		       (cond 
			((or (known-value n) (subsumed n)) '-infty)
			((member (action n) '(act finish)) (priority n))
			(t (error "Leaf node ~a was unknown with no children" n)))
		     (my+ (my* (priority n) *act-priority-mult*)
			  (reduce-set #'mymax children :key #'act-priority-q)))))
	(when (change-if-necessary (act-priority-q n) new-q)
	  (push 'act-priority-q (changed n)))))))
			

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; AHRTA* algorithm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *ref-level-max* 4)
(defvar *cycle-prevention* t)
(defvar *first-hla-ref-fraction* 0) ;; not used

(defun ahrta* (hpp priority-fn max-num-refinements
	       &key (state (init-state hpp)) (tiebreak-fn (constantly 0))
		    (known-values (make-known-values 'state #'same-state)) (max-num-actions nil)
		    (refinement-ind-choice ':first-highest) (final-choice ':lowest-c))
		    
  "ahrta* HPP PRIORITY-FN NUM-REFINEMENTS &key (STATE (INIT-STATE HPP)) KNOWN-VALUES (TIEBREAK-FN (constantly 0)) (MAX-NUM-ACTIONS nil) (REFINEMENT-IND ':first-highest) (FINAL-CHOICE ':lowest-c)

HPP - hierarchical planning problem
PRIORITY-FN - function that takes in action name, sound reward, and complete reward, and returns an extended real.
TIEBREAK-FN - a extended-real-valued function on nodes. Used to break ties between plans, where higher value is better.
MAX-NUM-ACTIONS - if provided, must be the max number of primitive actions at a state
REFINEMENT-IND - either ':first-highest or ':sound-complete-gap
FINAL-C - either ':lowest-c or ':highest-h. 

Carries out AHRTA* for at most NUM-REFINEMENTS refinements and returns the best plan, as well as the tree."
		      
  ;; Initialize
  (reset-subsumption-checker (subsumption-checker hpp) state hpp)
  (modify-known-value 'state 'terminal 'known t 'replace known-values)
  (modify-known-value 'state 'terminal 'priority 0 'replace known-values)
  (let ((tree (initial-tree hpp :prim-then-act t :init-state state :known-values known-values
			    :node-type '<ahrta*-node> :node-initargs (list ':priority-fn priority-fn)))
	(lowest-expanded-c 'infty)
	(expanded-lowest-c-plan nil)
	(best-plan-f '-infty)
	(best-tiebreak-value '-infty))
    (setf *tree* tree)
    (when max-num-actions
      (let ((states (list state)))
	(do-elements (n (active-child-nodes tree))
	  (when (not (known-value n))
	    (adjoinf states (state n) :test #'same-state)))
	(multf max-num-refinements (/ (1- (length states)) max-num-actions))
	(debug-print 1 "~&~a unknown children, so max-num-ref is now ~a" 
		     (1- (length states))  max-num-refinements)))
    (when (eq tiebreak-fn ':ref-level)
      (setf tiebreak-fn #'(lambda (n) (my+ 1 (mymin (ref-level hpp (action n)) *ref-level-max*)))))
    

    ;; Helper functions
    (labels ((c-value (path)
	       (loop for j from 1 below (1- (length path)) summing (priority (aref path j))))
	     
	     (f-value (plan)
	       "Total priority within plan, together with cached priority at last node if necessary."
	       (let* ((best-path (get-plan-path tree plan t))
		      (last-node (slast best-path)))
		 (my+ (sum-over (srest best-path) #'priority)
		      (iwhen (known-value last-node)
			     (lookup-known-value 'state (check-not-null (state last-node)) 'priority known-values)))))
	     
	     (tiebreak-value (path)
	       (loop for j from 1 below (length path) summing (funcall tiebreak-fn (aref path j))))
	       
	     (best-plan ()
	       "If best f-val achieved by plan ending at known state, that plan.  Else, lowest c-value expanded plan."
	       (mvbind 
		(plan val path) (get-best-plan tree ':begins-with-primitive :q-fn #'priority-q)
		(let ((best (if (and (known-value (slast path)) (my> val best-plan-f))
				plan
			      (ecase final-choice
				(:lowest-c expanded-lowest-c-plan)
				(:highest-h (get-best-plan tree ':begins-with-primitive :q-fn #'act-priority-q))))))
		  (debug-print 1 "Best plan is ~a with f-value ~a~:[ and c value ~a~;~]"
			       best (f-value best) (eq best plan) lowest-expanded-c)
		  (values best (f-value best))))))

      
      ;; Main computational loop
      (dotimes (i (ceiling max-num-refinements) (best-plan))
	(mvbind (plan val path) (get-best-plan tree ':any :q-fn #'noncached-priority-q)
		(when (eq val '-infty)
		  (debug-print 1 "Best plan ~a.  No refinable plans remaining" plan)
		  (return (best-plan)))
		(let ((ind 
		       (funcall
			(ecase refinement-ind-choice 
			  (:first-highest #'first-highest-ind)
			  (:sound-complete-gap #'ahrt-choose-ind))
			path hpp)))
		  (let ((c-val (c-value path)) 
			(tiebreak-val (tiebreak-value path)))
		    (when (and (my<= c-val lowest-expanded-c)
			       (or (not *cycle-prevention*) (my> tiebreak-val best-tiebreak-value)))
				  
		      (debug-print 1 "Last plan is now best")
		      (setf lowest-expanded-c c-val
			    best-plan-f val
			    expanded-lowest-c-plan plan
			    best-tiebreak-value tiebreak-val))
		    (cond
		     ((is-primitive (aref plan ind) hpp)
		      (debug-print 1 "~a.  Not refining ~a further." i plan)
		      ;; Somewhat of a hack.  If the highest priority plan is primitive, to avoid
		      ;; considering it further, we) mark its noncached-priority-q as being -infty.
		      ;; This is ok as noncached-priority-q is only used to choose which plan to refine.
		      (return (values plan (f-value plan))))
;		     (let ((node (slast path))
; 			     (par (parent-node node)))
; 			(setf (noncached-priority-q node) '-infty
; 			      (out-of-date-backward-flag par) t)
; 			(update-nodes-backward par known-values)))

		     (t
		      (refine tree plan ind :known-values known-values)
		      (debug-print 1 "Last plan had f-val ~a, c-val ~a, and tiebreak val ~a."
				   val c-val tiebreak-val)
		      ))
		    (debug-prompt 2))))))))

      
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Agent
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ahrta*-agent (hpp priority-fn num-refs 
		     &key (tiebreak-fn ':ref-level) (final-choice ':lowest-c)
			  (max-num-actions nil) (refinement-ind-choice ':first-highest))
  "ahrta*-agent HPP PRIORITY-FN NUM-REFS &key (MAX-NUM-ACTIONS nil) (REFINEMENT-IND-CHOICE ':first-highest) (FINAL-CHOICE ':lowest-c)
Return an agent that takes in a state and returns best primitive action after doing ahrta* lookahead.

See ahrta for arguments."
  (let ((known-values (make-known-values 'state #'same-state)))
    #'(lambda (s)
	(debug-print 3 "Known values :~& ~a" known-values)
	(mvbind (plan val)
	    (ahrta* hpp priority-fn num-refs :state s
		    :tiebreak-fn tiebreak-fn :known-values known-values :max-num-actions max-num-actions
		    :refinement-ind-choice refinement-ind-choice :final-choice final-choice)
	  (let ((rec (modify-known-value 'state s 'priority val 'min known-values)))
	    (modify-known-value-rec rec 'known t 'replace))
	    (debug-print 0 "Intending ~a with priority ~a" plan val)
	    (debug-prompt 1 "")
	    (sfirst plan)))))
      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ahrt-choose-ind (path hpp)
  (flet ((gap (i)
	   (let ((n (aref path i)))
	     (if (or (is-primitive (action n) hpp) (eql (complete-reward n) '-infty))
		 '-infty ;; A hack: it should be 0, but we want to make sure it doesn't ever get selected by tiebreak
	       (my- (complete-reward n) (sound-reward n)))))
	 (ref-level (i) 
	   (let ((a (action (aref path (1+ i)))))
	     (if (is-primitive a hpp)
		 'infty
	       (ref-level hpp a)))))
    (let ((best-gaps (sort (nth-value 3 (argmax (int-range 1 (length path)) :key #'gap)) #'<)))
      (nth-value 2 (argmin best-gaps :key #'ref-level)))))

(defun first-highest-ind (path hpp)
  (if (any path #'(lambda (n) (aand (action n) (not (is-primitive it hpp)))))
      (1- (minimizing-element 
	   (ndlet-fail
	    ((i (length path)))
	    (if (aand (action (aref path i)) (not (is-primitive it hpp)))
		i
	      'fail))
	   #'(lambda (i) (ref-level hpp (action (aref path i))))))
    1))

(defun first-hla (path hpp)
  (or
   (1- (position-if #'(lambda (n) (not (is-primitive (action n) hpp))) path :start 1))
   0))

			   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((n <ahrta*-node>) str)
  (print-unreadable-object (n str :type nil :identity nil)
    (format str "~a~:[ (inactive)~5*~;.  [~a, ~a].  Priority: ~a.  PQ=~a.~]  ~:[~;OODF. ~]~:[~;OODB~]~:[~;Subsumed. ~]~:[~;Known. ~]"
	    (action n) (active n) (sound-reward n) (complete-reward n)
	    (priority n) (priority-q n) (out-of-date-forward-flag n) (out-of-date-backward-flag n)
	    (subsumed n) (known-value n))))