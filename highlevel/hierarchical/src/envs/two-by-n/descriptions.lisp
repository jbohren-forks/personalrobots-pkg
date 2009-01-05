(in-package :two-by-n)

(defclass <two-by-n-descriptions> (<vb-descriptions>)
  ((top-node-type :initform '<sequence-node> :reader top-node-type)
   (top-action :initform '(top) :reader top-action)
   (final-valuation :accessor final-valuation)
   (minimal-valuation :reader minimal-valuation :initform nil)
   (maximal-valuation :accessor maximal-valuation)))

(defmethod initialize-instance :after ((d <two-by-n-descriptions>) &rest args)
  (declare (ignore args))
  (let ((n (n (planning-domain d))))
    (setf (final-valuation d) `(((0 ,n) . 0) ((1 ,n) . 0)))
    (setf (maximal-valuation d)
	  (loop for i upto n
		append `(((0 ,i) . 0) ((1 ,i) . 0))))))
   

(defmethod initial-valuation ((d <two-by-n-descriptions>))
  `(((0 0) . 0)))


;; Primitive
(defmethod action-description ((d <two-by-n-descriptions>) (name number) args type &aux (dom (planning-domain d)))
  (declare (ignore args))
  (make-instance 
   '<functional-description>
   :optimistic-progressor (alist-progressor dom name)
   :pessimistic-progressor (alist-progressor dom name)
   :optimistic-regressor (alist-regressor dom name)
   :pessimistic-regressor (alist-regressor dom name)))

;; Traverse
(defmethod action-description ((d <two-by-n-descriptions>) (name (eql 'traverse)) args type)
  (let* ((i (first args))
	 (costs (costs (planning-domain d)))
	 (stage-i-costs (ndlet ((j 2) (k 2)) (aref costs j i k)))
	 (max-cost (reduce-set #'mymax stage-i-costs))
	 (min-cost (reduce-set #'mymin stage-i-costs)))
	 
    (make-instance 
     '<functional-description>
     :optimistic-progressor #'(lambda (val) 
				(let ((x (my- (mymax (evaluate-valuation val (list 0 i)) (evaluate-valuation val (list 1 i))) min-cost)))
				  `(((0 ,(1+ i)) . ,x) ((1 ,(1+ i)) . ,x))))
     :pessimistic-progressor #'(lambda (val) 
				 (let ((x (my- (mymin (evaluate-valuation val (list 0 i)) (evaluate-valuation val (list 1 i))) max-cost)))
				   `(((0 ,(1+ i)) . ,x) ((1 ,(1+ i)) . ,x))))
     :optimistic-regressor  #'(lambda (v1 v2)
				(let ((x (my- (mymax (evaluate-valuation v2 (list 0 (1+ i))) (evaluate-valuation v2 (list 1 (1+ i)))) min-cost)))
				  (mapcan #'(lambda (m)
					      (let ((prev-state (list m i)))
						(when (member? prev-state (reachable-set v1))
						  (list (cons prev-state x)))))
					  '(0 1))))
     :pessimistic-regressor  #'(lambda (v1 v2)
				 (let ((x (my- (mymin (evaluate-valuation v2 (list 0 (1+ i))) (evaluate-valuation v2 (list 1 (1+ i)))) max-cost)))
				   (mapcan #'(lambda (m)
					       (let ((prev-state (list m i)))
						 (when (member? prev-state (reachable-set v1))
						   (list (cons prev-state x)))))
					   '(0 1)))))))



;; Top
(defmethod action-description ((d <two-by-n-descriptions>) (name (eql 'top)) args type &aux (dom (planning-domain d)) (n (n dom)))
  (declare (ignore args))
  (make-instance
   '<functional-description>
   :optimistic-progressor (constantly `(((0 ,n) . 0) ((1 ,n) . 0)))
   :pessimistic-progressor (constantly nil)
   :optimistic-regressor (constantly `(((0 0) . 0)))
   :pessimistic-regressor (constantly nil)))

				   
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Compatibility with older code
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun two-by-n-abstract-planning-problem (descs)
  (make-instance 
   '<abstract-planning-problem>
   :planning-problem (planning-domain descs) :hierarchy (hierarchy descs) :preprocess t :admissible-heuristic (constantly 0) 
   :complete-desc-fn #'(lambda (a h) (declare (ignore h))
			 (dsbind (name &rest args) (designated-list a)
			   (action-description descs name args :optimistic)))
   :sound-desc-fn #'(lambda (a h) (declare (ignore h))
		      (dsbind (name &rest args) (designated-list a)
			(action-description descs name args :pessimistic)))
   :init-val-type :list))




