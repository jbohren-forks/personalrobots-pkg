(in-package :vb-node)


(defgeneric action-description (descs type action-name action-args)
  (:documentation "Retrieve the description (see angelic/description.lisp) of an action of the form (NAME . ARGS).  TYPE is either :optimistic or :pessimistic."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by algorithms to progress/regress using the 
;; descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro def-valuation-update (name fn type)
  (with-gensyms (descs a v)
    `(defun ,name (,descs ,a ,v)
       (,fn (action-description ,descs ,type (car ,a) (cdr ,a)) ,v))))

(def-valuation-update progress-optimistic progress-complete-valuation :optimistic)
(def-valuation-update progress-pessimistic progress-sound-valuation :pessimistic)
(def-valuation-update regress-optimistic regress-complete-valuation :optimistic)
(def-valuation-update regress-pessimistic regress-sound-valuation :pessimistic)


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