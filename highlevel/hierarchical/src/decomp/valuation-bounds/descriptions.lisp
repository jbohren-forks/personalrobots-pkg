(in-package :vb-node)


(defgeneric action-description (descs type action-name action-args)
  (:documentation "Retrieve the description (see angelic/description.lisp) of an action of the form (NAME . ARGS).  TYPE is either :optimistic or :pessimistic."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by algorithms to progress/regress using the 
;; descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun progress-optimistic (descs a v)
  (progress-complete-valuation (action-description descs :optimistic (car a) (cdr a)) v))

(defun progress-pessimistic (descs a v)
  (progress-sound-valuation (action-description descs :pessimistic (car a) (cdr a)) v))

(defun regress-optimistic (descs a v1 v2)
  "Return (an upper bound on) the pointwise max of [the regression of valuation V2 through A] with V1."
  (regress-complete-valuation (action-description descs :optimistic (car a) (cdr a)) v1 v2))

(defun regress-pessimistic (descs a v1 v2)
  "Return (a lower bound on) the pointwise min of [the regression of valuation V2 through A] with V1."
  (regress-sound-valuation (action-description descs :pessimistic (car a) (cdr a)) v1 v2))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Previous version
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


#|


(defgeneric progress-optimistic-internal (descs a args v))
(defgeneric progress-pessimistic-internal (descs a args v))
(defgeneric regress-optimistic-internal (descs a args v))
(defgeneric regress-pessimistic-internal (descs a args v))
(defgeneric desc-domain (descs)
  (:documentation "Return the planning problem corresponding to a set of descriptions"))


(defmacro make-simple-descriptions ((desc-var dtype) (action-var set-var val-var) &body hla-descriptions)
  (with-gensyms (action-args s r valuation-var)
    (flet ((make-desc (action-name fn-name fn args default-set default-val)
	     `(defmethod ,fn-name ((,desc-var ,dtype) (,action-var (eql ',action-name)) ,action-args ,valuation-var)
		(declare (ignorable ,desc-var ,action-var ,action-args ,valuation-var))
		(dsbind (,@args) ,action-args
		  (declare (ignorable ,@args))
		  (let ((,set-var (sv-s ,valuation-var))
			(,val-var (sv-v ,valuation-var)))
		    (declare (ignorable ,set-var ,val-var))
		    ,(if fn
			 `(mvbind (,s ,r) ,fn
			    (make-simple-valuation ,s (+ ,r ,val-var)))
			 `(make-simple-valuation ,default-set ,default-val)))))))
    
      `(progn
	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key progress-optimistic &allow-other-keys) d
		  (make-desc name 'progress-optimistic-internal progress-optimistic args `(universal-set (desc-domain ,desc-var)) ''infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key progress-pessimistic &allow-other-keys) d
		  (make-desc name 'progress-pessimistic-internal progress-pessimistic args `(empty-set (desc-domain ,desc-var)) ''-infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key regress-optimistic &allow-other-keys) d
		  (make-desc name 'regress-optimistic-internal regress-optimistic args `(universal-set (desc-domain ,desc-var)) ''infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key regress-pessimistic &allow-other-keys) d
		  (make-desc name 'regress-pessimistic-internal regress-pessimistic args `(empty-set (desc-domain ,desc-var)) ''-infty)))
	    hla-descriptions)))))




|#