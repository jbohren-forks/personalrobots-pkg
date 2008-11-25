(in-package hla)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Def, constructor
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <variable-hierarchy> (<hierarchy>)
  ((descs :initarg :descs :reader descs)
   (top-level :initarg :top-level :reader top-level)
   (refinability-conditions :initarg :refinability-conditions :reader refinability-conditions)))



(defmacro make-variable-hierarchy (options &rest descs)
  "make-variable-hierarchy OPTIONS &rest DESCS  

OPTIONS is a keyword list.  Right now the only keyword is :env
DESCS is a list of action descriptions.  

An action description is a list of the form
ACTION-SCHEMA &key VARS REF TOP-LEVEL

The action schema looks like (ACTION-NAME &rest ARGUMENTS) where ACTION-NAME is a symbol, and each argument is either a symbol or a list (ARG VALS).
VARS is a list of items of the form (VAR-NAME VAR-DOMAIN) where VAR-NAME is a symbol, and VAR-DOMAIN is a form which evaluates to a set of possible values for that variable.  
The form may contain free references to the action arguments or any previous variable names.

REFS is a list of actions (which may contain free references to variables)."

  (dsbind (&key env (hierarchy-type '<variable-hierarchy>) (hierarchy-initargs nil)) options
	  `(apply #'make-instance ',hierarchy-type
		  :planning-problem ,env
		  :high-level-actions (disjoint-union
				       ,@(mapcar #'hla-set descs))
		  :refinability-conditions (list ,@(mapcar #'make-refinability-condition descs))
		  :descs (list ,@(mapcar #'make-desc descs))
		  :top-level (append ,@(mapcar #'make-top-level descs))
		  ,(quote-even-args hierarchy-initargs))))

(defun quote-even-args (args)
  `(list 
    ,@(loop 
	  for a in args
	  for i from 0
	  collect (if (evenp i)
		      `',a
		    a))))




;; For now, the set of top level actions is fixed
(defun make-top-level (desc)
  (with-gensyms (cset)
    (dsbind ((action &rest args) &key top-level &allow-other-keys) desc
      (when top-level
	(let ((var-names (mapcar #'car args)))
	  `(list (cons ',action
		       #'(lambda (,cset)
			   (declare (ignore ,cset))
			   (ndlet ,args
			     (declare (ignorable ,@(mapcar #'car args)))
			     (list ',action ,@var-names))))))))))

(defun hla-set (desc)
  (dsbind (action &rest args) (car desc)
    (let ((var-sets (if (consp (first args)) 
			(mapcar #'second args) 
		      (make-list (length args) :initial-element t))))
      `(make-instance '<direct-product-set>
	 :sets (list ,@var-sets)
	 :inst-acc (inst-vars:make-list-accessors ,(length var-sets) :prefix ',action)))))
		       


(defun make-desc (desc)
  (with-gensyms (params)
    (dsbind ((action &rest args) &key vars bindings (ref nil ref-supp) ref-form top-level &allow-other-keys) desc
      (declare (ignore top-level))
      (assert (xor ref-supp ref-form))
      (setf vars (append vars (mapcar #'(lambda (binding) (list (first binding) `(list ,(second binding)))) bindings)))
      (let* ((arg-names (if (symbolp (first args)) args (mapcar #'car args)))
	     (actual-ref (subst-vars ref (nconc (mapcar #'car vars) arg-names))))
	`(cons ',action
	       #'(lambda (complete-set ,params)
		   (declare (ignorable complete-set))
		   (dsbind ,arg-names ,params
		     (declare (ignorable ,@arg-names))
		     ,(if ref
			  `(ndlet* ,vars ,actual-ref)
			ref-form))))))))
			    


;; TODO: this only works when the refinability condition only refers to the complete set, not the action args
(defun make-refinability-condition (desc)
  (dsbind ((action &rest args) &key (refinable-when t) &allow-other-keys) desc
    (declare (ignore args))
    `(cons ',action
	   #'(lambda (complete-set)
	       (declare (ignorable complete-set))
	       ,refinable-when))))
	       

(defun subst-vars (tree vars)
  (labels ((helper (x)
	     (cond ((listp x) (cons 'list (mapcar #'helper x)))
		   ((member x vars) x)
		   (t `',x))))
    (helper tree)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Methods for hierarchies
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod applicable-refinements ((h <variable-hierarchy>) a cset)
  (dsbind (action &rest args) a
    (funcall (evaluate (descs h) action) cset args)))
    
(defmethod applicable-top-level-actions ((h <variable-hierarchy>) cset)
  (set:disjoint-union-of-sets
   (mapcar #'(lambda (pair) (funcall (cdr pair) cset)) (top-level h))))
   

(defmethod is-refinable ((h <variable-hierarchy>) a cset)
  (funcall (evaluate (refinability-conditions h) (car a)) cset))





