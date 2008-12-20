(in-package :vb-node)


(defgeneric progress-optimistic-internal (descs a args v))
(defgeneric progress-pessimistic-internal (descs a args v))
(defgeneric regress-optimistic-internal (descs a args v))
(defgeneric regress-pessimistic-internal (descs a args v))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Use this when defining descriptions for a domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


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
		  (make-desc name 'progress-optimistic-internal progress-optimistic args (universal-set d) ''infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key progress-pessimistic &allow-other-keys) d
		  (make-desc name 'progress-pessimistic-internal progress-pessimistic args (empty-set d) ''-infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key regress-optimistic &allow-other-keys) d
		  (make-desc name 'regress-optimistic-internal regress-optimistic args (universal-set d) ''infty)))
	    hla-descriptions)

	 ,@(mapcar
	    #'(lambda (d)
		(dsbind (name args &key regress-pessimistic &allow-other-keys) d
		  (make-desc name 'regress-pessimistic-internal regress-pessimistic args (empty-set d) ''-infty)))
	    hla-descriptions)))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by algorithms to progress/regress using the 
;; descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun progress-optimistic (descs a v)
  (dsbind (name &rest args) a
    (progress-optimistic-internal descs name args v)))

(defun progress-pessimistic (descs a v)
  (dsbind (name &rest args) a
    (progress-pessimistic-internal descs name args v)))

(defun regress-optimistic (descs a v)
  (dsbind (name &rest args) a
    (regress-optimistic-internal descs name args v)))

(defun regress-pessimistic (descs a v)
  (dsbind (name &rest args) a
    (regress-pessimistic-internal descs name args v)))

       
