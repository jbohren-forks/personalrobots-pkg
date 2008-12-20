(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class def
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <planning-problem> (<fully-observable-env>)
  ((goal :initarg :goal)
   (init-state :initarg :init-state)
   (action-descriptions :initarg :action-descriptions :accessor action-descriptions)
   (reward-fn :initarg :reward-fn :reader reward-fn :initform nil)
   (avail-actions-fn :initarg :avail-actions-fn :accessor avail-actions-fn))
  (:documentation "Class <planning-problem> (<env>)

Subclasses: <prop-domain>

A planning problem is viewed as an environment where the initial state and transitions are deterministic, and the episode terminates when a goal state is reached.

Subclasses may implement
- goal
- init-state
- primitive-action-description
- avail-actions
- init-state-set
- same-state

And optionally
- reward

The top-level allows initargs :goal, :init-state, :action-descriptions, :reward-fn, and :avail-actions-fn, which may be used instead of implementing the above methods.
")
  (:default-initargs :initialize-state nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; constructor
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod initialize-instance :around ((d <planning-problem>) &rest args)
  (declare (ignore args))
  (call-next-method)
  (create-env:set-state (init-state d) d))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun action-result (e s a)
  "action-result E S A.  Resulting state after doing primitive action A in state S of env E."
  (succ-state s (primitive-action-description e a)))

(defgeneric goal (prob)
  (:documentation "goal PROB.  Return goal set of PROB.")
  (:method ((e <planning-problem>)) (slot-value e 'goal)))

(defun goal? (prob s)
  (values
   (member? s (goal prob))))

(defgeneric reward (e s a)
  (:documentation "Reward PLANNING-DOMAIN STATE ACTION.  Return the reward for doing ACTION in STATE.  Default method just returns -1.")
  (:method ((e <planning-problem>) s a)
	   (aif (reward-fn e)
	       (funcall it s a)
	     -1)))
	   

(defgeneric init-state (e)
  (:documentation "init-state ENV.")
  (:method ((e <planning-problem>)) (slot-value e 'init-state)))

(defgeneric init-state-set (e)
  (:documentation "init-state-set PLANNING-DOMAIN.  Return a representation of the singleton set consisting of the initial state of this domain.  Can also be called on <abstract-planning-problem>s, in which case it forwards the request to the underlying planning domain.")
  (:method ((e <planning-problem>)) (make-state-set e (init-state e))))

(defgeneric make-state-set (e s)
  (:documentation "make-state-set DOMAIN STATE.  Return a representation of the singleton set consisting of this state.")
  (:method ((e <planning-problem>) s) (list s)))

(defgeneric empty-set (d)
  (:documentation "Return the empty set in the state representation used by this planning problem."))

(defgeneric universal-set (d)
  (:documentation "Return the set of all states for this planning problem."))

  

(defun action-seq-result (e s actions)
  "action-seq-result E S A.  Return resulting state if we do (primitive) actions in A starting in state S."
  (map nil (lambda (a) (setf s (action-result e s a))) actions)
  s)

(defgeneric primitive-action-description (e a)
  (:documentation "primitive-action-description PLANNING-DOMAIN ACTION.  Get description of this action.")
  (:method ((e <planning-problem>) a) (mapping:evaluate (action-descriptions e) a)))

(defun succeeds? (e actions &optional (s (init-state e)))
  "succeeds? E PRIMITIVE-ACTION-SEQUENCE STATE.  Does doing this action sequence in the initial state result in a state that satisfies the goal?  STATE defaults to the initial state."
  (goal? e (action-seq-result e s actions)))

(defgeneric all-actions (e)
  (:documentation "all-actions PLANNING-PROBLEM.  Get set of all possible primitive-actions in PLANNING-PROBLEM.")
  (:method ((e <planning-problem>)) (mapping:domain (action-descriptions e))))


  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations from <env>
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod sample-next ((e <planning-problem>) s a)
  (values (action-result e s a) (reward e s a)))

(defmethod sample-init ((e <planning-problem>))
  (init-state e))

(defmethod is-terminal-state ((e <planning-problem>) s)
  (goal? e s))

(defmethod avail-actions ((e <planning-problem>) s)
  (funcall (avail-actions-fn e) s))