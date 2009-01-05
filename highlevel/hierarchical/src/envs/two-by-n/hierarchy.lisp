(in-package :two-by-n)

;; Inheritance from <hierarchy> is just so we can compare with the old code
(defclass <two-by-n-hierarchy> (<decomp-hierarchy> <hierarchy>)
  ())

(defmethod initialize-instance :after ((h <two-by-n-hierarchy>) &rest args)
  (declare (ignore args))
  ;; compatibility
  (let ((dom (planning-domain h)))
    (setf (planning-problem h) dom
	  (high-level-actions h) (mapset 'list #'(lambda (i) `(traverse ,i)) (n dom)))))
  

(make-hlas (h <two-by-n-hierarchy>) () (0 1)
  
  ;; Top: move through each stage in sequence
  (top () :sequence
       (let ((n (n (planning-domain h))))
	 (list (loop for i below n collecting `(traverse ,i)))))

  (traverse (i) :or '((0) (1))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Methods from old hierarchy code
;; to do comparisons
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod applicable-top-level-actions ((h <decomp-hierarchy>) cset)
  (let ((actions nil))
    (dotimes (i (n (planning-domain h)) actions)
      (when (or (member? `(0 ,i) cset) (member? `(1 ,i) cset))
	(pushnew `(traverse ,i) actions :test #'equal)))))

(defmethod applicable-refinements ((h <decomp-hierarchy>) a cset)
  (assert (eq (car a) 'traverse))
  '((0) (1)))

