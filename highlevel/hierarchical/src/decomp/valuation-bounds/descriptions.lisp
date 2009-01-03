(in-package :vb-node)

(define-debug-topic :valuation :vb-node)

(defclass <vb-descriptions> ()
  ((hierarchy :initarg :hierarchy :reader hierarchy)
   (top-node-type)
   (top-action)))

(defmethod planning-domain ((descs <vb-descriptions>))
  (planning-domain (hierarchy descs)))


(defgeneric action-description (descs action-name action-args type)
  (:documentation "Retrieve the description (see angelic/description.lisp) of an action of the form (NAME . ARGS).  TYPE is either :optimistic or :pessimistic."))

(defun top-node (descs)
  (let* ((n (make-instance (top-node-type descs) :action (top-action descs) :parent nil :descs descs))
	 (e (planning-domain descs))
	 (init (new-val-diff (initial-valuation e)))
	 (final (new-val-diff (final-valuation e))))
    (update-external-variable n 'initial-optimistic init)
    (update-external-variable n 'initial-pessimistic init)
    (update-external-variable n 'final-optimistic final)
    (update-external-variable n 'final-pessimistic final)
    n))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by algorithms to progress/regress using the 
;; descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *progress-optimistic-counts* nil)
(defvar *progress-pessimistic-counts* nil)
(defvar *regress-pessimistic-counts* nil)
(defvar *regress-optimistic-counts* nil)


(defun progress-optimistic (descs a v)
  (when *progress-optimistic-counts* (incf *progress-optimistic-counts*))  
  (let* ((a (designated-list a))
	 (v2 (progress-complete-valuation (action-description descs (car a) (cdr a) :optimistic) v)))
    (debug-out :valuation 1 t "~&Optimistic progression~& a: ~a~& v: ~a~& v': ~a" a v v2)
    v2))

(defun progress-pessimistic (descs a v)
  (when *progress-pessimistic-counts* (incf *progress-pessimistic-counts*))
  (let* ((a (designated-list a))
	 (v2 (progress-sound-valuation (action-description descs (car a) (cdr a) :pessimistic) v)))
    (debug-out :valuation 1 t "~&Pessimistic progression~& a: ~a~& v: ~a~& v': ~a" a v v2)
    v2))

(defun regress-optimistic (descs a v1 v2)
  "Return (an upper bound on) the pointwise max of [the regression of valuation V2 through A] with V1."
  (when *regress-optimistic-counts* (incf *regress-optimistic-counts*))
  (let* ((a (designated-list a))
	 (v3 (regress-complete-valuation (action-description descs (car a) (cdr a) :optimistic) v1 v2)))
    (debug-out :valuation 1 t "~&Optimistic regression~& a: ~a~& v2: ~a~& v3: ~a" a v2 v3)
    v3))

(defun regress-pessimistic (descs a v1 v2)
  "Return (a lower bound on) the pointwise min of [the regression of valuation V2 through A] with V1."
  (when *regress-pessimistic-counts* (incf *regress-pessimistic-counts*))
  (let* ((a (designated-list a))
	 (v3 (regress-sound-valuation (action-description descs (car a) (cdr a) :pessimistic) v1 v2)))
    (debug-out :valuation 1 t "~&Pessimistic regression~& a: ~a~& v2: ~a~& v3: ~a" a v2 v3)
    v3))



