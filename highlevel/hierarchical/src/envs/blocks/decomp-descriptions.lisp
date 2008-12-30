(in-package :blocks)


(defclass <blocks-descriptions> ()
  ((domain :initarg :domain :reader desc-domain)
   (heuristic :accessor heuristic :initarg :heuristic)
   (optimistic-descs :accessor optimistic-descs)
   (hierarchy :initarg :hierarchy :reader hierarchy)
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
(defmethod action-description ((d <blocks-descriptions>) name args type)
  (let ((action (cons name args)))
    (if (eq :primitive (action-class action (hierarchy d)))
	(primitive-action-description (desc-domain d) action)
	(lookup-in-ncstrips-schemas action (ecase type (:pessimistic (pessimistic-descs d)) (:optimistic (optimistic-descs d)))))))

(defun make-simple-description (succ-state-fn predecessor-fn reward-fn)
  (make-instance '<simple-description> :succ-state-fn (designated-function succ-state-fn) :predecessor-fn (designated-function predecessor-fn) :reward-fn (designated-function reward-fn)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Specific actions that are not defined in
;; the earlier code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod action-description ((d <blocks-descriptions>) (name (eql 'act)) args (type (eql :optimistic)))
  (declare (ignore args))
  (make-simple-description
   (goal (desc-domain d))
   (nth-arg-fn 0)
   #'(lambda (s1 s2)
       (declare (ignore s2))
       (funcall (heuristic d) s1))))


(defmethod action-description ((d <blocks-descriptions>) (name (eql 'act)) args (type (eql :pessimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description
     #'(lambda (s) (intersect s (goal dom)))
     #'(lambda (s1 s2) (declare (ignore s2)) (intersect s1 (goal dom)))
     0)))


(defmethod action-description ((d <blocks-descriptions>) (name (eql 'finish)) args type)
  (declare (ignore args type))
  (let ((dom (desc-domain d)))
    (make-simple-description
     #'(lambda (s) (intersect s (goal dom)))
     #'(lambda (s1 s2) (intersect s1 s2 (goal dom)))
     #'(lambda (s1 s2) (declare (ignore s2)) (if (intersects s1 (goal dom)) 0 '-infty)))))
  

(defmethod action-description ((d <blocks-descriptions>) (name (eql 'move-then-act)) args (type (eql :optimistic)))
  (action-description d 'act nil :optimistic))

(defmethod action-description ((d <blocks-descriptions>) (name (eql 'move-then-act)) args (type (eql :pessimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description (empty-set dom) (empty-set dom) '-infty)))


(defmethod action-description ((d  <blocks-descriptions>) (name (eql 'pickup)) args (type (eql :optimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description (universal-set dom) (nth-arg-fn 0) 0)))


(defmethod action-description ((d  <blocks-descriptions>) (name (eql 'pickup)) args (type (eql :pessimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description (empty-set dom) (empty-set dom) '-infty)))

(defmethod action-description ((d  <blocks-descriptions>) (name (eql 'stack)) args (type (eql :optimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description (universal-set dom) (nth-arg-fn 0) 0)))


(defmethod action-description ((d  <blocks-descriptions>) (name (eql 'stack)) args (type (eql :pessimistic)))
  (declare (ignore args))
  (let ((dom (desc-domain d)))
    (make-simple-description (empty-set dom) (empty-set dom) '-infty)))





#|
(defmethod action-description ((d <blocks-descriptions>) (name (eql 'navigate-beside)) args (type (eql :optimistic)))
  (dsbind (x y) args
    (make-simple-description
     #'nav-beside-progressor
     #'(lambda (s1 s2) (declare (ignore s2)) s1))))
     
|#