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
  (lookup-in-ncstrips-schemas (cons name args) (optimistic-descs d)))

(defmethod action-description ((d <blocks-descriptions>) (type (eql :pessimistic)) name args)
  (lookup-in-ncstrips-schemas (cons name args) (pessimistic-descs d)))


(defun make-simple-description (succ-state-fn predecessor-fn reward-fn)
  (make-instance '<simple-description> :succ-state-fn succ-state-fn :predecessor-fn predecessor-fn :reward-fn reward-fn))


(defmethod action-description ((d <blocks-descriptions>) (type (eql :optimistic)) (name (eql 'act)) args)
  (make-simple-description
   (constantly (goal (desc-domain d)))
   #'(lambda (s1 s2) (declare (ignore s2)) s1)
   #'(lambda (s1 s2)
       (declare (ignore s2))
       (funcall (heuristic d) s1))))


(defmethod action-description ((d <blocks-descriptions>) (type (eql :pessimistic)) (name (eql 'act)) args)
  (make-simple-description
   #'(lambda (s) (intersect s (goal (desc-domain d))))
   #'(lambda (s1 s2) (declare (ignore s2)) (intersect s1 (goal (desc-domain d))))
   (constantly 0)))


		 
