;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; lookahead/abstract-planning-problem.lisp
;; Define an abstract planning problem
;; Also contains the algorithm from the ICAPS07 paper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; type
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <abstract-planning-problem> ()
 ((planning-problem :type <planning-problem> :initarg :planning-problem :accessor planning-problem)
  (hierarchy :initarg :hierarchy :type <hierarchy> :accessor hierarchy)
  (preprocess :initarg :preprocess :initform nil :accessor preprocess)
  (admissible-heuristic :initarg :admissible-heuristic :initform (constantly 'infty)
			:accessor admissible-heuristic)
  (guarantee-heuristic :initarg :guarantee-heuristic :initform nil :accessor guarantee-heuristic)
  (subsumption-checker :initarg :subsumption-checker :initform nil :accessor subsumption-checker)
  (complete-desc-fn :initarg :complete-desc-fn :reader complete-desc-fn)
  (sound-desc-fn :initarg :sound-desc-fn :reader sound-desc-fn)
  )
 (:documentation "Class <abstract-planning-problem>.  Create using make-instance with initargs
:planning-problem - object of type <planning-problem>
:hierarchy - hierarchy
:preprocess - whether to preprocess by adding a dummy terminal state, and 'finish and 'act actions
:admissible-heuristic - a function from sets of states to extended reals that maps a set of states to an upper bound on achievable reward from that state.  Defaults to (constantly 'infty).  It will be better to use (constantly 0) if the domain has nonpositive rewards.


:complete-desc-fn, :sound-desc-fn.  Only necessary if not using a subclass of <abstract-planning-problem> that defines these.  Take in action and hierarchy and return description.
:guarantee-heuristic - Optional.  If provided, must be a function that takes in a set of states, and returns a guarantee on achievable reward to goal from any of those states.  If not provided, the assumption is that termination can only be guaranteed from goal states.
:subsumption-checker - Optional.
"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; descriptions and reward functions
;; wrappers that deal specially with the case
;; of the dummy actions 'finish or 'act
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (finish-desc (:conc-name fd-) (:constructor make-finish-desc (goal)))
  goal)

(defstruct (act-sound-desc (:conc-name ad-) (:constructor make-act-sound-desc (goal guarantee)))
  guarantee
  goal)

(defstruct (act-complete-desc (:conc-name acd-) (:constructor make-act-complete-desc (heuristic)))
  heuristic)

(defgeneric complete-desc (p a)
  (:method :around (p a)
	   (when (preprocess p) 
	     (case a
	       (finish (return-from complete-desc (make-finish-desc (goal (planning-problem p)))))
	       (act (return-from complete-desc (make-act-complete-desc (admissible-heuristic p))))))
	   (call-next-method))   
  (:method (p a) (funcall (complete-desc-fn p) a (hierarchy p))))

(defgeneric sound-desc (p a)
  (:method :around (p a)
	   (when (preprocess p)
	     (case a
	       (finish (return-from sound-desc (make-finish-desc (goal (planning-problem p)))))
	       (act (return-from sound-desc (make-act-sound-desc (goal (planning-problem p))
								 (guarantee-heuristic p))))))
	   (call-next-method))
  (:method (p a) (funcall (sound-desc-fn p) a (hierarchy p))))

(defmethod successor-set ((desc finish-desc) s)
  (if (intersects s (fd-goal desc)) '(terminal) nil))

(defmethod successor-set ((desc act-complete-desc) s)
  (declare (ignore s))
  '(terminal))

(defmethod successor-set ((desc act-sound-desc) s)
  (if (or (intersects s (ad-goal desc))
	  (aand (ad-guarantee desc) (my> (funcall it s) '-infty)))
      '(terminal) nil))

(defun progress-finish-valuation (desc val)
  (if (intersects (sv-s val) (fd-goal desc))
      (make-simple-valuation '(terminal) (sv-v val))
    (make-simple-valuation nil '-infty)))

(defmethod progress-sound-valuation ((desc finish-desc) (val simple-valuation))
  (progress-finish-valuation desc val))

(defmethod progress-complete-valuation ((desc finish-desc) (val simple-valuation))
  (progress-finish-valuation desc val))

(defmethod progress-complete-valuation ((desc act-complete-desc) (val simple-valuation))
  (make-simple-valuation '(terminal) (my+ (sv-v val) (funcall (acd-heuristic desc) (sv-s val)))))

(defmethod progress-sound-valuation ((desc act-sound-desc) (val simple-valuation))
  (cond
   ((ad-guarantee desc)
    (make-simple-valuation '(terminal) (my+ (sv-v val) (funcall (ad-guarantee desc) (sv-s val)))))
   ((intersects (sv-s val) (ad-goal desc))
    (make-simple-valuation '(terminal) (sv-v val)))
   (t (make-simple-valuation nil '-infty))))

(defmethod succ-state (s (desc finish-desc))
  (if (member? s (fd-goal desc)) 'terminal 'illegal-finish))

      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; forwarded operations
;; These are provided for convenience so that algorithms 
;; can just send requests to the <abstract-planning-problem>
;; which will forward them to the relevant component
;;
;; TODO: all other operations below should use these
;; rather than calling them on the underlying planning 
;; problem or hierarchy
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod primitive-action-description ((p <abstract-planning-problem>) a)
  (let ((d (planning-problem p)))
    (if (eq a 'finish)
	(make-finish-desc (goal d))
	(primitive-action-description d a))))

(defmethod init-state-set ((p <abstract-planning-problem>))
  (init-state-set (planning-problem p)))

(defmethod init-state ((p <abstract-planning-problem>))
  (init-state (planning-problem p)))

(defmethod avail-actions ((p <abstract-planning-problem>) s)
  (avail-actions (planning-problem p) s))

(defmethod make-state-set ((p <abstract-planning-problem>) s)
  (make-state-set (planning-problem p) s))

(defmethod applicable-refinements ((p <abstract-planning-problem>) a hset)
  (let ((h (hierarchy p)))
    (if (and (preprocess p) (eq a 'act))
	
	;; When preprocessed, deal specially with 'act action
	(disjoint-union
	 (ndlet ((f (applicable-top-level-actions h hset)))
	   (list f 'act))
	 (when (intersects hset (goal (planning-problem p))) '((finish))))
      
      ;; For other actions, or nonpreprocessed problems, forward the request to the hierarchy
      (applicable-refinements h a hset))))
      
(defun lookup-action (a p)
  "lookup-action A HPP.  Return either 'high-level or 'primitive."
  (when (and (preprocess p) (member a '(act finish)))
    (return-from lookup-action (case a (act 'high-level) (finish 'primitive))))
  ;; Otherwise forward
  (action-type a (hierarchy p)))

(defun is-primitive (a p)
  (eq 'primitive (lookup-action a p)))

(defmethod ref-level ((p <abstract-planning-problem>) a)
  (cond ((and (preprocess p) (eq a 'act)) -1)
	((and (preprocess p) (eq a 'finish)) 'infty)
	(t (ref-level (hierarchy p) a))))
      
(defun complete-reachable-set (a s p)
  (successor-set (complete-desc p a) s))

(defun sound-reachable-set (a s p)
  (successor-set (sound-desc p a) s))

(defun progress-sound (a v p)
  "progress-sound A V HPP.  Progress valuation V under the sound description of A."
  (progress-sound-valuation (sound-desc p a) v))

(defun progress-complete (a v p)
  "progress-complete A V HPP.  Progress valuation V under the complete description of A."
  (progress-complete-valuation (complete-desc p a) v))

(defmethod valid-plan? ((hpp <abstract-planning-problem>) plan)
  (valid-plan? (hierarchy hpp) plan))
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic progression operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric progress-hset (p s a)
 (:documentation "progress-hset ABSTRACT-PLANNING-PROBLEM HSET ACTION.  Progress the heuristic set through action"))

(defgeneric hset-initial-value (p s)
 (:documentation "Initial value of heuristic set at state S."))

(defun hset-result (p s plan)
 "hset-result P S PLAN.  Return the sequence of heuristic sets if doing PLAN starting at set S."
 (let* ((l (length plan))
        (sets (make-array (1+ l))))
   (setf (aref sets 0) s)
   (do-elements (a plan sets i)
     (setf (aref sets (1+ i)) (progress-hset p (aref sets i) a)))))


(defun complete-result (p s plan)
 "complete-result ABSTRACT-PLANNING-PROB STATE-SET ACTION-SEQUENCE.  Return the successor set, according to the complete semantics, of this action sequence starting at STATE-SET."
 (let ((states s))
   (do-elements (a plan states)
     (setf states (successor-set (complete-desc p a) states)))))


(defun sound-result (p s a)
 "sound-result ABSTRACT-PLANNING-PROB STATE-SET ACTION-SEQUENCE.  Return the successor set, according to the sound semantics, of this action sequence starting at STATE-SET.  As a secondary value, return the sound sequence."
 (let* ((l (length a))
        (sets (make-array (1+ l))))
   (setf (aref sets 0) s)
   (do-elements (action a (values (slast sets) sets) i)
     (setf (aref sets (1+ i))
       (successor-set (sound-desc p action) (aref sets i))))))

(defun achieves-complete (prob a s goal)
 "achieves-complete ABSTRACT-PLANNING-PROBLEM ACTION-SEQUENCE INIT-STATE-SET GOAL.  Does the complete result of doing ACTION-SEQUENCE at INIT-STATE intersect GOAL?"
 (intersects (complete-result prob s a) goal))

(defun achieves-sound (prob a s goal)
 "achieves-sound ABSTRACT-PLANNING-PROBLEM ACTION-SEQUENCE INIT-STATE-SET GOAL.  Does the sound result of doing ACTION-SEQUENCE at INIT-STATE intersect GOAL?"
 (intersects (sound-result prob s a) goal))

(defun succeeds-complete (p a)
 "succeeds-complete ABSTRACT-PLANNING-PROB ACTION-SEQUENCE.  Can this sequence possibly achieve the goal starting from the initial state, according to the complete semantics?  As a secondary value, return the complete sequence."
 (let ((d (planning-problem p)))
   (achieves-complete p a (init-state-set d) (goal d))))

(defun succeeds-sound (p a)
 "succeeds-sound ABSTRACT-PLANNING-PROB ACTION-SEQUENCE.  Can this sequence possibly achieve the goal starting from the initial state, according to the sound semantics?"
 (let ((d (planning-problem p)))
   (achieves-sound p a (init-state-set d) (goal d))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; iteration over top-level plans
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-plan-tree-node (hset &rest args)
 (apply #'make-instance '<node> :node-label hset args))

(defun top-level-plans (p)
 "top-level-plans ABSTRACT-PLANNING-PROBLEM.  Return an iteratable representation of the set of top-level plans for this planning problem.  The plans are ordered by length.  Plans are only included if each of their actions is applicable (according to applicable-top-level-actions)."
 (make-instance '<recursive-enumeration>

   ;; the initial state of the plan-generator tree - a queue consisting of a single node representing the empty plan
   :init-state-fn
   #'(lambda ()
       (make-queue
        (list (make-plan-tree-node (hset-initial-value p (init-state (planning-problem p)))))))

   ;; given a current plan generator queue and corresponding tree, return the plan corresponding to the first node on the queue
   ;; the plan is computed by going up the tree from the node until reaching the root, and then getting the corresponding edge labels
   :output-fn
   #'(lambda (q)
       (let ((n (peek-front q)))
         (mapcar (fn (edge-label parent-edge))
                 (rest (get-path-from-root n)))))

   ;; to update the plan generator state
   :trans-fn
   (let ((h (hierarchy p)))
     #'(lambda (q)
           (if (queue-empty q)
               ':no-more-elements

             ;; Dequeue the first node in the queue
             (let ((n (dequeue q)))
               (assert (leaf? n) () "Front node ~a in queue in plan generator was not a leaf in the plan generation tree - it had children ~a"
                       n (children n))

               ;; Figure out its sound and complete sets if necessary
               (when (eq (node-label n) 'node-label-unset)
                 (setf (node-label n)
                   (progress-hset p (node-label (get-parent n)) (edge-label (parent-edge n)))))

               ;; Create a child node for each one and add to the back of the queue.  Finally, return the new queue.
               (do-elements (a (applicable-top-level-actions h (node-label n)) q)
                   (enqueue (add-new-child n a 'node-label-unset) q))))))

   ))

(defvar *plans-so-far* nil)
(defvar *print-inc* 10000)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Algorithm that works in arbitrary hierarchies.  Use iterative deepening
;; in top-level, and a loop-checker in recursive calls to decompose
;; Note that this algorithm doesn't take rewards into account or explicitly
;; use a lookahead tree.  See abstract-lookahead-tree for the 'modern' version.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *refinement-set-type* ':stack)

(defun sound-complete-forward-search (p &key (depth-multiplier 2) (first-action-only nil) (init-depth-limit 1)
				      &aux (prob (planning-problem p)))
 "sound-complete-forward-search ABSTRACT-PLANNING-PROBLEM &key (DEPTH-MULTIPLIER 2) (FIRST-ACTION-ONLY nil) (INIT-DEPTH-LIMIT 1)

Algorithm that works for arbitrary recursive hierarchies.

DEPTH-MULTIPLIER: every time we increase the depth limit by 1, we multiply the number of top-level plans considered by this number.
FIRST-ACTION-ONLY: if this is true, return as soon as the first primitive action is found

Behaviour is also affected by
*refinement-set-type* - either ':queue or ':stack"

 (let ((s (init-state prob))
       (goal (goal prob))
       (top-level-plans (top-level-plans p)))

   ;; Top-level iterative deepening loop
   (do ((depth-limit init-depth-limit (incf depth-limit))
        (num-plans 1 (multf num-plans depth-multiplier)))

       ;; never terminate naturally (only by a return within the body)
       (nil)

     (debug-print 1 "~&Top level loop.  Depth limit is ~a and num-plans is ~a." depth-limit num-plans)

     ;; Loop over first num-plans plans
     (do-elements (plan top-level-plans nil plan-ind)
       (when (>= plan-ind num-plans) (return))

       ;; when we find one that works, return it
       (let ((ref (find-primitive-refinement
                   p s goal (make-refinement-set
                             *refinement-set-type*
                             (list (cons plan (make-array (length plan) :element-type 'fixnum :initial-element 0))))
                   depth-limit nil first-action-only)))
         (unless (eq ref 'no-plan-found)
           (return-from sound-complete-forward-search ref)))))))


(defun find-primitive-refinement (p s goal refinements depth-limit decompose-stack first-only
                                 &aux (init-hset (hset-initial-value p s)) (h (hierarchy p))
                                      (init-states (make-state-set (planning-problem p) s)))
 (debug-print 1 "~&~%Looking for primitive refinement of one of the plans ~a at depth limit ~a~&with init state: ~a"
              refinements depth-limit s)

 (flet ((handle-next-refinement (refinement)
          (debug-print 1 "~&Next refinement is ~a" refinement)
          (dsbind (next-plan . next-plan-depths) refinement

            (debug-print 1 "... ~:[does not succeed completely~;succeeds completely~:[...not within depth limit~;~]~]"
                         (achieves-complete p next-plan init-states goal)
                         (my< (reduce #'mymax next-plan-depths :initial-value '-infty) depth-limit))


            (when (and (my< (reduce #'mymax next-plan-depths :initial-value '-infty) depth-limit)
                       (achieves-complete p next-plan init-states goal))

              ;; When it succeeds completely and is within the depth limit
              (cond

               ;; A. If it's primitive, return it at once
               ((is-primitive-sequence next-plan h)
                (debug-print 1 "... and is primitive.")
                (return-from find-primitive-refinement next-plan))

               ;; B. Otherwise, if it succeeds soundly and does not introduce a cycle,
               ;; decompose it
               ((and
                 (achieves-sound p next-plan init-states goal)
                 (not (member-if #'(lambda (item) (item-equal item next-plan goal s)) decompose-stack)))

                (debug-print 1 "... and soundly.")
                (let ((primitive-plan (decompose p s next-plan next-plan-depths goal
                                                 (cons (list s next-plan goal) decompose-stack) first-only)))
                  (debug-print 1 "~&Returning plan ~a" primitive-plan)
                  (return-from find-primitive-refinement primitive-plan)))

               ;; C. Otherwise, add all its immediate refinements to the refinement set
               (t
                (debug-print 1  "...~:[not guaranteed soundly~;introduces a cycle~]." (achieves-sound p next-plan init-states goal))
                (do-elements (refinement (immediate-applicable-refinements-and-depths
                                          h next-plan next-plan-depths (hset-result p init-hset next-plan)))
                  (add-refinement refinements refinement))))))))

   ;; Loop until refinement set is empty
   (loop
     (if (empty? refinements)
         (progn
           (debug-print 1 "~&Did not find a primitive refinement of ~a within depth limit" refinements)
           (return-from find-primitive-refinement 'no-plan-found))
       (handle-next-refinement (get-next refinements))))))


(defun decompose (p s plan depths goal decompose-stack first-only
                 &aux (n (length plan)) (d (planning-problem p)) (h (hierarchy p)))
 (debug-print 1 "~&~%Calling decompose with plan ~a~&Init state: ~a" plan s)
 (let ((subgoals (make-array n)))
   (mvbind (res sseq) (sound-result p (make-state-set d s) plan)
     (declare (ignore res))
     ;; The last subgoal is the part of the goal that we can reach
     (setf (aref subgoals (1- n)) (intersect goal (aref sseq n)))

     ;; Backchain to find subgoals at each step
     (do ((i (1- n) (decf i)))
         ((zerop i))
       (setf (aref subgoals (1- i))
         (regress (aref sseq i) (aref subgoals i) (sound-desc p (nth i plan)))))

     (do ((depth-limit
           (ecase *refinement-set-type*
             (:queue 'infty)
             (:stack (reduce #'mymax depths :initial-value '-infty)))
           (incf depth-limit)))
         (nil )
       (loop
           with state = s
           with refined-plan = nil
           for i below n
           for a in plan
           for depth in depths
           for subgoal across subgoals
           for subplan = (ecase (action-type a h)
                           (primitive (list a))
                           (high-level
                            (find-primitive-refinement
                             p state subgoal
                             (make-refinement-set
                              *refinement-set-type*
                              (mapset 'list #'identity
                                      (immediate-applicable-refinements-and-depths
                                       h (list a) (list depth)
                                       (hset-result p (hset-initial-value p state) (list a)))))
                             (mymax depth-limit (+ 2 depth)) decompose-stack first-only)))
           when (eq subplan 'no-plan-found)
           return nil

           when first-only
           do (return-from decompose subplan)

           do (setf state (action-seq-result d state subplan)
                    refined-plan (nconc refined-plan subplan))
           finally (return-from decompose refined-plan))))))


(defun item-equal (item next-plan goal s)
 (dsbind (state plan subgoal) item
   (and (same-state state s)
        (equalp plan next-plan)
        (set-eq goal subgoal)))
 )

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; refinement sets store refinements
;; can either be a stack or queue
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric make-refinement-set (refinement-set-type elements)
 (:method ((refinement-set-type (eql ':stack)) elements)
          (list elements))
 (:method ((refinement-set-type (eql ':queue)) elements)
          (make-queue elements)))

(defgeneric empty? (refinement-set)
 (:method ((refinement-set list))
          (null (first refinement-set)))
 (:method ((refinement-set queue))
          (queue-empty refinement-set)))

(defgeneric get-next (refinement-set)
 (:method ((refinement-set list))
          (pop (first refinement-set)))
 (:method ((refinement-set queue))
          (dequeue refinement-set)))

(defgeneric add-refinement (refinement-set refinement)
 (:method ((refinement-set list) refinement)
          (push refinement (first refinement-set)))
 (:method ((refinement-set queue) refinement)
          (enqueue refinement refinement-set)))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

