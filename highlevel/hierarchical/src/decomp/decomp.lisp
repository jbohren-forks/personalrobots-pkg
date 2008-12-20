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


(defmacro make-hlas ((hierarchy-var htype) keys primitive-actions &body descs)
  "Definition macro for hierarchies.  HIERARCHY-VAR is a symbol and HTYPE is a symbol naming a hierarchy type.   KEYS (unevaluated) is a list of symbols.  PRIMITIVE-ACTIONS is a list of primitive action names.  DESCS is a list of HLA descriptions.  For example: (make-hlas (h <hierarchy>) (init-set) (a1 a2) (a (x y) (list (foo x h) (bar y init-set))))
This will define methods for generic functions refinements and action-class for <hierarchy>.   High level actions for this hierarchy are 3-element lists with first-element 'a, and primitive actions are lists beginning with 'a1 or 'a2.  The set of refinements of such an action are given by the form (list ...).  Note that this form can refer to the hierarchy using HIERARCHY-VAR, and to keyword arguments in KEYS."


  (with-gensyms (hla)
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
		descs))))))
			   
			   


