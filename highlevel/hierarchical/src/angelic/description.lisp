;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; description.lisp
;; Define action descriptions and valuations
;;
;; 
;; Dec 30, 2008
;; Went through the different types and ops.  It's gotten a bit clunky over time
;; and in need of refactoring.  But for now:
;; 1. There are a bunch of base ops that take in a description and either a
;; state (succ-state) or set(s) (succ-set, regress, hla-complete/sound reward)
;; 2. There are a couple more ops that take in a description and a valuation
;; (progress/regress-sound/complete-valuation).  In the only case that's 
;; implemented (simple valuations and descriptions), these just call out to
;; the ops from 1.
;; 3. Then there are other ops that take in a planning problem (or abstract-
;; planning-problem), an action and a state.  These look up the action description 
;; using one of primitive-action-desc, sound-desc, or complete-desc, then 
;; call one of the ops from 1 or 2.  Examples are in abstract-planning-problem,
;; and in ../decomp/valuation-bounds/descriptions.lisp, as well as some of
;; the planning algorithms themselves.
;;
;; The main option would be to not have separate objects for descriptions and
;; just pass the planning-problem and action name around, but the current
;; method seems more flexible, e.g., for having different types of descriptions
;; for different actions, at the cost of complexity and possibly needing to
;; define a separate description type when defining a planning problem (some
;; standard types exist such as ncstrips).
;;
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package :lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Base ops
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric successor-set (desc states)
  (:documentation "successor-set DESCRIPTION STATES.  Return the successor set of STATES according to DESCRIPTION.")
  (:method ((desc function) states)
	   (funcall desc states)))

(defgeneric succ-state (s d)
  (:documentation "succ-state STATE DESC.  Return next state after doing action with description DESC in STATE.  Applies only to descriptions that are, or can be treated as, deterministic.")
  (:method (s (d function)) (funcall d s)))
	   
(defgeneric regress (s s2 d)
  (:documentation "regress STATE-SET NEXT-STATE-SET DESC.  Return the set of states in STATE-SET such that progressing each of them through DESC yields a set that intersects NEXT-STATE-SET."))

(defgeneric hla-complete-reward (d s s2)
  (:documentation "hla-complete-reward SIMPLE-DESCRIPTION STATE-SET SUCC-STATE-SET.  
Return a number R with the property that, for any refinement of the action referred to by description that goes between a state in STATE-SET and a state in SUCC-STATE-SET, the reward is <= R."))

(defgeneric hla-sound-reward (d s s2)
  (:documentation "hla-sound-reward DESC S S'.  Return a number R, with the property that for every state in SUCC-STATE-SET, there is some state in STATE-SET and a refinement of the HLA of DESC that goes between the given states with reward >= R."))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Valuations and reward bounds
;; There are two kinds of valuations: forward and backward
;; - (forward) valuations map each state to a bound on 
;;   how much reward can be attained for an action sequence
;;   that ends at that state
;; - backward valuations map each state to a bound on 
;;   how much reward can be attained in future starting at
;;   this state
;; The type system does not distinguish between the two
;; kinds.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (simple-valuation (:conc-name sv-) (:constructor make-simple-valuation (s v)))
  "A simple valuation consists of a set and number and represents the function that equals the number on the set and -infty elsewhere."
  s v)

(defun initial-valuation (d)
  (make-simple-valuation (init-state-set d) 0))

(defun final-valuation (d)
  (make-simple-valuation (goal d) 0))

(defgeneric progress-sound-valuation (desc val)
  (:documentation "progress-sound-valuation DESC V.  Suppose V is a lower-bound on the current valuation.  Returns a new valuation guaranteed to be a lower-bound on the result of progressing V through DESC.")
  (:method ((desc function) val) (funcall desc val)))

(defgeneric progress-complete-valuation (desc val)
  (:documentation "progress-complete-valuation DESC V.  Suppose V is an upper-bound on the current valuation.  Returns a new valuation guaranteed to be an upper-bound on the result of progressing V through DESC.")
  (:method ((desc function) val) (funcall desc val)))

(defgeneric regress-sound-valuation (desc val1 val2)
  (:documentation "regress-sound-valuation DESC V1 V2.  If V2 is a lower-bound on backward valuation, and V1 is a lower bound on the previous forward valuation, return a lower bound on the backward valuation before the action with description DESC happens.")
  (:method ((desc function) val1 val2)
    (funcall desc val1 val2)))

(defgeneric regress-complete-valuation (desc val1 val2)
  (:documentation "regress-complete-valuation DESC V1 V2.  If V2 is an upper-bound on backward valuation, and V1 is an upper bound on the previous forward valuation, return an upper bound on the backward valuation before the action with description DESC happens.")
  (:method ((desc function) val1 val2)
    (funcall desc val1 val2)))

(defgeneric evaluate-valuation (v s)
  (:documentation "evaluate-valuation VALUATION STATE.  Returns an extended real.")
  (:method ((v simple-valuation) s) (if (member? s (sv-s v)) (sv-v v) '-infty)))


(defgeneric equal-valuations (v1 v2)
  (:documentation "equal-valuations V1 V2.  Returns t only if V1 and V2 are pointwise equal on the state space.  On the other hand, may not always detect equality.  In general, depending on the specific type of V1 and V2, will only return t if their representations are the same.")
  (:method (v1 v2) (eq v1 v2))
  (:method ((val1 simple-valuation) (val2 simple-valuation))
	   (let ((v1 (sv-v val1))
		 (v2 (sv-v val2))
		 (s1 (sv-s val1))
		 (s2 (sv-s val2)))
	     (or (and (eq v1 '-infty) (eq v2 '-infty))
		 (and (is-empty s1) (is-empty s2))
		 (and (eql v1 v2) (set-eq s1 s2))))))

(defgeneric pointwise-subsumes (v1 v2)
  (:documentation "pointwise-subsumes V1 V2.  Return t only if V1 >= V2 pointwise.  May not always detect subsumption.")
  (:method (v1 v2) (eq v1 v2))
  (:method ((v1 simple-valuation) (v2 simple-valuation))
	   (and (subset (sv-s v2) (sv-s v1))
		(my<= (sv-v v2) (sv-v v1)))))



(defgeneric reachable-set (v)
  (:documentation "reachable-set VALUATION.  Return the set of states for which this valuation is greater than '-infty.")
  (:method ((val simple-valuation))
    (when (my> (sv-v val) '-infty)
      (sv-s val))))

(defgeneric max-achievable-value (v)
  (:documentation "max-achievable-value VALUATION.  Return the maximum achievable value of this VALUATION.")
  (:method ((val simple-valuation))
	   (if (is-empty (sv-s val))
	       '-infty
	     (sv-v val))))

(defgeneric binary-pointwise-max-upper-bound (v1 v2)
  (:method ((v1 simple-valuation) (v2 simple-valuation))
    ;; For simple valuations, union the sets and take the max value, except in the special case where the set for the max val is empty, in which case return the other valuation
    (when (my> (sv-v v2) (sv-v v1))
      (rotatef v1 v2))
    (if (is-empty (sv-s v1))
	v2
	(make-simple-valuation (binary-union (sv-s v1) (sv-s v2)) (sv-v v1)))))

(defgeneric binary-pointwise-min-upper-bound (v1 v2)
  (:method ((v1 simple-valuation) (v2 simple-valuation))
    ;; We can do this exactly
    (make-simple-valuation (binary-intersection (sv-s v1) (sv-s v2)) (mymin (sv-v v1) (sv-v v2)))))


;; Note we treat the lower bound differently - instead of making an approximate simple valuation like in the upper bound, we make an exact
;; upper bound of type max-valuation.  Will need to eventually allow calling code to control when this is done to prevent blowup.
(defgeneric binary-pointwise-max-lower-bound (v1 v2)
  (:method ((v1 simple-valuation) (v2 simple-valuation))
    (make-max-valuation (list v1 v2))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; max valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (max-valuation (:constructor make-max-valuation (vals)))
  vals)

(defmethod reachable-set ((v max-valuation))
  (reduce #'binary-union (max-valuation-vals v) :key #'reachable-set))

(defmethod max-achievable-value ((v max-valuation))
  (reduce #'mymax (max-valuation-vals v) :key #'max-achievable-value))

(def-symmetric-method binary-pointwise-max-upper-bound ((v1 max-valuation) v2)
  (make-max-valuation (cons v2 (max-valuation-vals v1))))

;; same as upper bound, as it is exact
(def-symmetric-method binary-pointwise-max-lower-bound ((v1 max-valuation) v2)
  (make-max-valuation (cons v2 (max-valuation-vals v1))))

(defmethod evaluate-valuation ((v max-valuation) s)
  (reduce #'mymax (max-valuation-vals v) :key #'(lambda (val) (evaluate-valuation val s))))
  




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sum valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (sum-valuation (:constructor create-sum-valuation))
  vals)

(defun make-sum-valuation (&rest vals)
  (create-sum-valuation :vals vals))

(defmethod max-achievable-value ((v sum-valuation))
  (with-struct (sum-valuation- vals) v
    (assert (= 2 (length vals)) nil "max-achievable-value currently only implemented for sums of two valuations (easy to extend using DP if necessary)")
    (let ((s1 (sv-s (first vals)))
	  (v1 (sv-v (first vals)))
	  (s2 (sv-s (second vals)))
	  (v2 (sv-v (second vals)))
	  (achievable nil))
      (when (intersects s1 s2) (push (my+ v1 v2) achievable))
      (unless (subset s1 s2) (push v1 achievable))
      (unless (subset s2 s1) (push v2 achievable))
      (apply #'mymax achievable))))

	       

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Simple descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <simple-description> ()
  ((succ-state-fn :initarg :succ-state-fn :reader succ-state-fn)
   (predecessor-fn :initarg :predecessor-fn :reader predecessor-fn)
   (reward-fn :initarg :reward-fn :reader reward-fn))
  (:documentation "A simple description has methods for successor-set and hla-sound/complete-reward.  It then progresses/regresses valuations by first finding the successor/predecessor-set, then calling hla-reward.   The successor, predecessor and reward functions can be provided using the initargs :succ-state-fn, :predecessor-fn and :reward-fn (note that there aren't separate functions for sound/complete - override hla-sound/complete-reward if this is a problem, like in ncstrips.lisp).  Alternatively, subclasses may override successor-set, regress and/or hla-reward."))

(defun make-simple-description (succ-state-fn predecessor-fn reward-fn)
  (make-instance '<simple-description> :succ-state-fn (designated-function succ-state-fn) :predecessor-fn (designated-function predecessor-fn) :reward-fn (designated-function reward-fn)))

(defmethod hla-complete-reward ((d <simple-description>) s s2)
  (funcall (reward-fn d) s s2))

(defmethod hla-sound-reward ((d <simple-description>) s s2)
    (funcall (reward-fn d) s s2))

(defmethod successor-set ((d <simple-description>) s)
  (funcall (succ-state-fn d) s))

(defmethod regress (s1 s2 (d <simple-description>))
  (funcall (predecessor-fn d) s1 s2))


(defmethod progress-sound-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-sound-reward d s s2)))
      (make-simple-valuation s2 (iunless (is-empty s2) (my+ r v))))))

(defmethod progress-complete-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-complete-reward d s s2)))
      (make-simple-valuation s2 (my+ r v)))))

(defmethod regress-sound-valuation ((d <simple-description>) (val1 simple-valuation) (val2 simple-valuation))
  (let* ((s2 (sv-s val2))
	 (s1 (regress (sv-s val1) s2 d))
	 (r (hla-sound-reward d s1 s2)))
    ;; Adding this check won't change behavior, and avoids occasional annoying exceptions with adding infty and -infty
    (make-simple-valuation s1 (iunless (is-empty s1) (my+ r (sv-v val2))))))

(defmethod regress-complete-valuation ((d <simple-description>) (val1 simple-valuation) (val2 simple-valuation))
  (let* ((s2 (sv-s val2))
	 (s1 (regress (sv-s val1) s2 d))
	 (r (hla-complete-reward d s1 s2)))
    (make-simple-valuation s1 (my+ r (sv-v val2)))))



    
(defmethod progress-sound-valuation ((d <simple-description>) (val max-valuation))
  (make-max-valuation (mapcar #'(lambda (v) (progress-sound-valuation d v)) (max-valuation-vals val))))

(defmethod regress-sound-valuation ((d <simple-description>) val1 (val max-valuation))
  (make-max-valuation (mapcar #'(lambda (v) (regress-sound-valuation d val1 v)) (max-valuation-vals val))))

(defmethod regress-sound-valuation ((d <simple-description>) (val1 max-valuation) val)
  (make-max-valuation (mapcar #'(lambda (v) (regress-sound-valuation d v val)) (max-valuation-vals val1))))