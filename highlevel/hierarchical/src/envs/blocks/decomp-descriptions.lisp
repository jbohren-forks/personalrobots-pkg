(in-package :blocks)


(defclass <blocks-descriptions> ()
  ((domain :initarg :domain :reader desc-domain)
   (heuristic :accessor heuristic :initarg :heuristic)
   (optimistic-descs :accessor optimistic-descs)
   (pessimistic-descs :accessor pessimistic-descs)))

(defmethod initialize-instance :after ((d <blocks-descriptions>) &rest args)
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (setf (heuristic d) (dist-heuristic dom)
	  (optimistic-descs d) (make-complete-descriptions dom)
	  (pessimistic-descs d) (make-sound-descriptions dom))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; By default reuse the old code in descriptions.lisp
(defmethod action-description ((d <blocks-descriptions>) (type (eql :optimistic)) name args)
  (lookup-in-ncstrips-schemas (optimistic-descs d) (cons name args)))

(defmethod action-description ((d <blocks-descriptions>) (type (eql :pessimistic)) name args)
  (lookup-in-ncstrips-schemas (pessimistic-descs d) (cons name args)))


