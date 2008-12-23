(defpackage :decomp
  (:use :cl :utils :set)
  (:export
   :refinements
   :action-class
   :make-hlas))
  

(in-package :decomp)


(defgeneric refinements (hla hierarchy &key &allow-other-keys)
  (:documentation "Return the set of refinements of a high level action.  Hierarchy is the overall action hierarchy.  Additionally, depending on the hierarchy type, there may be other required keyword arguments that contain the context of this action.  For example, for <vb-hierarchy> hierarchies, an argument :initial-optimistic-set must be provided, which is an upper bound on the set of reachable states at the start of this HLA."))

(defgeneric action-class (a h)
  (:documentation "Hierarchies can have different types of actions.  For example, a sequential hierarchy has :sequence, :or, and :primitive actions."))

(defgeneric action-level (a h)
  (:documentation "An nonnegative extended real, where 0 means a top level action, 'infty means primitive actions."))


(defmacro make-hlas ((hierarchy-var htype) keys primitive-actions &body descs)
  "Definition macro for hierarchies.  HIERARCHY-VAR is a symbol and HTYPE is a symbol naming a hierarchy type.   KEYS (unevaluated) is a list of symbols.  PRIMITIVE-ACTIONS is a list of primitive action names.  DESCS is a list of HLA descriptions.  For example: (make-hlas (h <hierarchy>) (init-set) (a1 a2) (a (x y) (list (foo x h) (bar y init-set))))
This will define methods for generic functions refinements, action-class, and action-level for <hierarchy>.   High level actions for this hierarchy are 3-element lists with first-element 'a, and primitive actions are lists beginning with 'a1 or 'a2.  The set of refinements of such an action are given by the form (list ...).  Note that this form can refer to the hierarchy using HIERARCHY-VAR, and to keyword arguments in KEYS.

The levels of the actions are 0, 1, 2,... in the order that they appear in the descriptions.  Primitive actions are level infinity."

  ;; Possibly the generated code could be made more efficient by having the macro precompute a hashtable that is used at runtime
  ;; instead of a big case statement which takes O(hla-types)
  (with-gensyms (hla)
    (let ((level -1))
      `(progn
	 (defmethod refinements (,hla (,hierarchy-var ,htype) &key ,@keys)
	   (ecase (first (designated-list ,hla))
	     (,primitive-actions (error "Can't find refinements of primitive action ,hla"))
	     ,@(mapcar #'(lambda (desc)
			   (dsbind (action-name action-args type &body body) desc
			     (declare (ignore type))
			     `(,action-name 
			       (dsbind ,action-args (rest (designated-list ,hla))
				 (declare (ignorable ,@action-args))
				 ,@body))))
		       descs)))
	 (defmethod action-class (,hla (,hierarchy-var ,htype))
	   (ecase (first (designated-list ,hla))
	     (,primitive-actions :primitive)
	     ,@(mapcar #'(lambda (desc)
			   (dsbind (action-name args action-type &body body) desc
			     (declare (ignore args body))
			     `(,action-name ',action-type)))
		       descs)))
	 
	 (defmethod action-level (,hla (,hierarchy-var ,htype))
	   (ecase (first (designated-list ,hla))
	     (,primitive-actions 'infty)
	     ,@(mapcar #'(lambda (desc) `(,(first desc) ,(incf level))) descs)))))))
			   
			   


