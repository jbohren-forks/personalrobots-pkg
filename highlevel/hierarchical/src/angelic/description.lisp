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
  (:documentation "hla-complete-reward DESCRIPTION STATE-SET SUCC-STATE-SET.  
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
;; kinds of valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

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
  (:documentation "evaluate-valuation VALUATION STATE.  Returns an extended real."))


(defgeneric equal-valuations (v1 v2)
  (:documentation "equal-valuations V1 V2.  Returns t only if V1 and V2 are pointwise equal on the state space.  On the other hand, may not always detect equality.  In general, depending on the specific type of V1 and V2, will only return t if their representations are the same.")
  (:method (v1 v2) (eq v1 v2)))


(defgeneric pointwise-subsumes (v1 v2)
  (:documentation "pointwise-subsumes V1 V2.  Return t only if V1 >= V2 pointwise.  May not always detect subsumption.")
  (:method (v1 v2) (eq v1 v2)))



(defgeneric reachable-set (v)
  (:documentation "reachable-set VALUATION.  Return the set of states for which this valuation is greater than '-infty.")
  )

(defgeneric max-achievable-value (v)
  (:documentation "max-achievable-value VALUATION.  Return the maximum achievable value of this VALUATION.")
  )


;; pointwise max and min are analogs of union and intersection respectively
;; upper and lower bounds are analogs of super- and subsets respectively
(defgeneric binary-pointwise-max-upper-bound (v1 v2))

(defgeneric binary-pointwise-min-upper-bound (v1 v2))

(defgeneric binary-pointwise-max-lower-bound (v1 v2))

(defgeneric print-valuation (v &optional str)
  (:documentation "Print valuation V to stream STR.  Respects the pretty-printing special variables."))

(defun pprint-valuation (&rest args)
  (bind-pprint-args (str v) args
    (print-valuation v str)))

(defun pprint-valuation-as-list (&rest args)
  (handler-bind
      ((add-infinity-and-minus-infinity #'(lambda (c) (declare (ignore c)) (use-value 'infty-infty))))
    (bind-pprint-args (str v) args
      (pprint-logical-block (str (mapset 'list #'(lambda (x) (cons x (evaluate-valuation v x))) (reachable-set v)))
	(format str "~a: " (class-name (class-of v)))
	(awhile (pprint-pop)
	  (format str "~w " it))))))