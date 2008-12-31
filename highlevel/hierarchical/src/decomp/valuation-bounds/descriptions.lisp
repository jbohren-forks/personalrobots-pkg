(in-package :vb-node)

(define-debug-topic :valuation :vb-node)

(defclass <vb-descriptions> ()
  ((hierarchy :initarg :hierarchy :reader hierarchy)))

(defmethod planning-domain ((descs <vb-descriptions>))
  (planning-domain (hierarchy descs)))


(defgeneric action-description (descs action-name action-args type)
  (:documentation "Retrieve the description (see angelic/description.lisp) of an action of the form (NAME . ARGS).  TYPE is either :optimistic or :pessimistic."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Called by algorithms to progress/regress using the 
;; descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun progress-optimistic (descs a v)
  (let* ((a (designated-list a))
	 (v2 (progress-complete-valuation (action-description descs (car a) (cdr a) :optimistic) v)))
    (debug-out :valuation 1 t "~&Optimistic progression~& a: ~a~& v: ~a~& v': ~a" a v v2)
    v2))

(defun progress-pessimistic (descs a v)
  (let* ((a (designated-list a))
	 (v2 (progress-sound-valuation (action-description descs (car a) (cdr a) :pessimistic) v)))
    (debug-out :valuation 1 t "~&Pessimistic progression~& a: ~a~& v: ~a~& v': ~a" a v v2)
    v2))

(defun regress-optimistic (descs a v1 v2)
  "Return (an upper bound on) the pointwise max of [the regression of valuation V2 through A] with V1."
  (let* ((a (designated-list a))
	 (v3 (regress-complete-valuation (action-description descs (car a) (cdr a) :optimistic) v1 v2)))
    (debug-out :valuation 1 t "~&Optimistic regression~& a: ~a~& v2: ~a~& v3: ~a" a v2 v3)
    v3))

(defun regress-pessimistic (descs a v1 v2)
  "Return (a lower bound on) the pointwise min of [the regression of valuation V2 through A] with V1."
  (let* ((a (designated-list a))
	 (v3 (regress-sound-valuation (action-description descs (car a) (cdr a) :pessimistic) v1 v2)))
    (debug-out :valuation 1 t "~&Pessimistic regression~& a: ~a~& v2: ~a~& v3: ~a" a v2 v3)
    v3))



