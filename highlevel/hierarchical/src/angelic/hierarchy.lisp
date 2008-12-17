(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic type def and accessors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <hierarchy> ()
  ((planning-problem :initarg :planning-problem :accessor planning-problem)
   (high-level-actions :accessor high-level-actions :initarg :high-level-actions)
   (valid-plan-fn :initarg :valid-plan-fn :initform (constantly t) :accessor valid-plan-fn))
		       
  (:documentation "Class <hierarchy>

Initargs
:planning-problem - of type <planning-problem>
:valid-plan-fn - Function that takes in a plan and must return nil for plans generated from Act that are nevertheless invalid for some reason.  Defaults to (constantly t).
"))


(defun action-type (a h)
  "action-type ACTION-NAME HIERARCHY.  Returns one of 'high-level, 'primitive, or 'unknown-action."
  (cond
   ((member? a (primitive-actions h)) 'primitive)
   ((member? a (high-level-actions h)) 'high-level)
   (t 'unknown-action)))

(defun primitive-actions (h)
  (all-actions (planning-problem h)))

(defun is-primitive-sequence (a h)
  "is-primitive-sequence ACTION-SEQUENCE HIERARCHY.  Are all the actions primitive?"
  (every #'(lambda (x) (eq (action-type x h) 'primitive)) a))

(defgeneric valid-plan? (h plan)
  (:documentation "valid-plan? HIERARCHY PLAN.  Return t iff the PLAN is valid according to HIERARCHY.  This is used to specify global constraints about plans generated from this hierarchy.  An <abstract-planning-problem> may also be used as the first argument, in which case it just forwards to the underlying hierarchy.")
  (:method (h plan) 
	   (funcall (valid-plan-fn h) plan)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by sound-complete-forward-search
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *random-refinement-ordering* t
  "Variable *random-refinement-ordering*.  Defaults to t.  If true, then the set of refinements of a plan is (uniformly) randomly permuted each time.")

(defun immediate-applicable-refinements-and-depths (h actions depths hsets)
  "immediate-applicable-refinements-and-depths HIERARCHY ACTION-SEQ HSETS"
  (mvbind (i key a) (argmin actions :key #'(lambda (a) (ref-level h a)))
    (declare (ignore key))
    (let ((depth (elt depths i)))
      (when (eq (action-type a h) 'high-level)
	(ndlet ((ref (funcall (if *random-refinement-ordering* #'random-permutation #'identity)
			      (applicable-refinements h a (aref hsets i)))))
	  (let ((new-depths (make-array (length ref) :element-type 'fixnum :initial-element (1+ depth))))
	    (cons
	     (substitute-ref 'list actions i ref)
	     (substitute-ref 'list depths i new-depths) ;; not exactly what it was intended for...
	     )))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; other generic functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric is-refinable (h a cset)
  (:documentation "Does it make sense to refine A at complete set CSET.  The intended use is for situations where CSET is large and the set of refinements will depend strongly on the particular state, so there would be too many refinements.")
  (:method ((h <hierarchy>) a cset)
	   "Default top level method - always t."
	   (declare (ignore a cset))
	   t))

(defgeneric applicable-refinements (h a cset)
  (:documentation "applicable-refinements HIERARCHY ACTION CSET.  Return the set of refinements of ACTION that are 'applicable' given this complete set.

Can also be called on <abstract-planning-problems> in which case the request is forwarded to the underlying hierarchy.")
  (:method :around ((h <hierarchy>) a cset)
	   "A debugging check that the action is in fact refinable."
	   (assert (is-refinable h a cset) nil "Attempted to find applicable refinements of ~a at ~a, but the action is not refinable at this set." a cset)
	   (call-next-method)))
			       

(defgeneric applicable-top-level-actions (h hset)
  (:documentation "applicable-top-level-actions HIERARCHY HSET.  Return the set of 'applicable' top level actions given this hset.  The meaning of 'applicable' may vary.  For example, given a precondition function for actions, it may return the set of actions whose preconditions are satisfied at some state in the set.  The default method returns all the high-level and primitive actions.")
  (:method ((h <hierarchy>) hset)
	   (declare (ignore hset))
	   (disjoint-union (high-level-actions h) (primitive-actions h))))


(defgeneric ref-level (h a)
  (:documentation "ref-level H A.  What is the refinement level of this action?  Used for controlling when things get refined.  Higher level actions are assumed to have lower refinement levels (though I don't believe anything in the code actually depends on that).")
  (:method ((h <hierarchy>) a) (declare (ignore a)) 0))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; miscellaneous
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun substitute-ref (result-type actions ind ref)
  "substitute-ref RESULT-TYPE ACTIONS IND REF.  Replace the action at position IND of ACTIONS with sequence REF and return the new sequence.  Nondestructive."
  (concatenate result-type (subseq actions 0 ind) ref (subseq actions (1+ ind))))