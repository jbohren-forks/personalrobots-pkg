(in-package env)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; type def
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (trajectory (:constructor create-trajectory))
  "Type trajectory.  Create using make-trajectory.  Access fields using trajectory-X where X=env, states, actions, rewards.  The state, action, and reward sequences must always be adjustable vectors, and must always maintain the invariant that length(rewards) = length(actions) = length(states) - 1, which can be checked by calling is-valid."
  env
  states
  actions
  rewards)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun is-valid-trajectory (trajectory)
  "is-valid-trajectory TRAJECTORY.  Checks if the states, actions, and rewards are vectors, with length(states)-1 = length(actions) = length(rewards)."
  (let ((s (trajectory-states trajectory))
	(a (trajectory-actions trajectory))
	(r (trajectory-rewards trajectory)))
    (and (typep s '(array * 1))
	 (typep a '(array * 1))
	 (typep r '(array * 1))
	 (= (1- (length s)) (length a) (length r)))))
  

(defun make-trajectory (env states actions rewards)
  "make-trajectory ENV STATES ACTIONS REWARDS

ENV - an environment.  Most of the operations will work even if this is nil.
STATES, ACTIONS, REWARDS - sequences.  STATES must be one longer than the other two, since it includes the terminal state, at which no action is done.  If the sequences are lists, they are first coerced to vectors.

Returns an object of type trajectory."
  
  (flet ((coerce-if-necessary (seq) (if (listp seq) (let ((l (length seq))) (make-array l :fill-pointer l :adjustable t :initial-contents seq)) seq)))
    (let ((tr (create-trajectory :env env :states (coerce-if-necessary states)
				 :actions (coerce-if-necessary actions)
				 :rewards (coerce-if-necessary rewards))))
      (assert (is-valid-trajectory tr) nil
	"Trajectory ~a not valid - see documentation of is-valid-trajectory for details." tr)
      tr)))


(defun num-transitions (trajectory)
  "num-transitions TRAJECTORY.  Return the length of the trajectory, which equals the number of actions, or one less than the number of states."
  (assert (is-valid-trajectory trajectory) nil
	  "Trajectory ~a not valid - see documentation of is-valid-trajectory for details." trajectory)
  (length (trajectory-actions trajectory)))


(defun transitions (trajectory &key (share-structure nil))
  "transitions TRAJECTORY &key (SHARE-STRUCTURE nil)

Returns the implicitly represented [set] (really a multiset) of transitions in this trajectory, each of which is a list (S A R S').  If SHARE-STRUCTURE is nil, then lists do not share structure with each other."
  (assert (is-valid-trajectory trajectory) nil
	  "Trajectory ~a not valid - see documentation of is-valid-trajectory for details." trajectory)
  (let ((common-list (when share-structure (make-list 4)))
	(states (trajectory-states trajectory))
	(actions (trajectory-actions trajectory))
	(rewards (trajectory-rewards trajectory)))
    (set:ndlet ((i (num-transitions trajectory)))
      (let ((l (if share-structure common-list (make-list 4))))
	(setf (first l) (aref states i)
	      (second l) (aref actions i)
	      (third l) (aref rewards i)
	      (fourth l) (aref states (1+ i)))
	l))))
    
      
  
  
  