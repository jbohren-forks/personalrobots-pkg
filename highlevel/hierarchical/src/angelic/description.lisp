;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; description.lisp
;; Define abstract action descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Action descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric successor-set (desc states)
  (:documentation "successor-set DESCRIPTION STATES.  Return the successor set of STATES according to DESCRIPTION.")
  (:method ((desc function) states)
	   (funcall desc states)))


(defgeneric succ-state (s d)
  (:documentation "succ-state STATE DESC.  Return next state after doing action with description DESC in STATE.  Applies only to descriptions that are, or can be treated as, deterministic.")
  (:method (s (d function)) (funcall d s)))

	   
(defgeneric regress (s s2 d)
  (:documentation "regress STATE-SET NEXT-STATE-SET DESC.  Find states in STATE-SET such that progressing them through DESC yields a set that intersects NEXT-STATE-SET."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Valuations and reward bounds
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (simple-valuation (:conc-name sv-) (:constructor make-simple-valuation (s v)))
  "A simple valuation consists of a set and number and represents the function that equals the number on the set and -infty elsewhere."
  s v)

(defgeneric progress-sound-valuation (desc val)
  (:documentation "progress-sound-valuation DESC V.  Suppose V is a lower-bound on the current valuation.  Returns a new valuation guaranteed to be a lower-bound on the result of progressing V through DESC.")
  (:method ((desc function) val) (funcall desc val)))

(defgeneric progress-complete-valuation (desc val)
  (:documentation "progress-complete-valuation DESC V.  Suppose V is an upper-bound on the current valuation.  Returns a new valuation guaranteed to be an upper-bound on the result of progressing V through DESC.")
  (:method ((desc function) val) (funcall desc val)))

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



;; TODO: Are these next two necessary?
(defgeneric reachable-set (v)
  (:documentation "reachable-set VALUATION.  Return the set of states for which this valuation is greater than '-infty.")
  (:method ((val simple-valuation))
	   ;; TODO this isn't quite right the case when v equals '-infty
	   ;; In any case, get rid of this function eventually
	   (sv-s val)))

(defgeneric max-achievable-value (v)
  (:documentation "max-achievable-value VALUATION.  Return the maximum achievable value of this VALUATION.")
  (:method ((val simple-valuation))
	   (if (is-empty (sv-s val))
	       '-infty
	     (sv-v val))))

	       

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Simple descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <simple-description> ()
  ((succ-state-fn :initarg :succ-state-fn :reader succ-state-fn)
   (reward-fn :initarg :reward-fn :reader reward-fn))
  (:documentation "A simple description has methods for successor-set and hla-reward.  It then progresses valuations by first finding the successor-set, then calling hla-reward.
Both the successor and reward functions can be provided using the initargs :succ-state-fn and :reward-fn.  Alternatively, subclasses may override successor-set and hla-reward."))

(defgeneric hla-complete-reward (d s s2)
  (:documentation "hla-complete-reward SIMPLE-DESCRIPTION STATE-SET SUCC-STATE-SET.  
Return upper bound on reward for going from STATE-SET to SUCC-STATE-SET.")
  (:method ((d <simple-description>) s s2)
	   (funcall (reward-fn d) s s2)))

(defgeneric hla-sound-reward (d s s2)
  (:documentation "hla-sound-reward DESC S S'.  Return a number R, with the property that for every state in SUCC-STATE-SET, there is some state in STATE-SET and a refinement of the HLA of DESC that goes between the given states with reward >= R.")
  (:method ((d <simple-description>) s s2)
	   (funcall (reward-fn d) s s2)))

(defmethod successor-set ((d <simple-description>) s)
  (funcall (succ-state-fn d) s))
			  

(defmethod progress-sound-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-sound-reward d s s2)))
      (make-simple-valuation s2 (my+ r v)))))

(defmethod progress-complete-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-complete-reward d s s2)))
      (make-simple-valuation s2 (my+ r v)))))
  