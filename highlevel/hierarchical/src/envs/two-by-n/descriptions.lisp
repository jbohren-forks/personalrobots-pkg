(in-package :two-by-n)

(defclass <two-by-n-descriptions> ()
  ((hierarchy :initarg :hierarchy :reader hierarchy)
   (planning-domain :writer set-domain :reader planning-domain)))

(defmethod initialize-instance :after ((descs <two-by-n-descriptions>) &rest args &key hierarchy)
  (declare (ignore args))
  (set-domain (planning-domain hierarchy) descs))



;; Primitive
(defmethod action-description ((d <two-by-n-descriptions>) (name number) args type &aux (dom (planning-domain d)))
  (let ((desc (primitive-action-description dom name))
	(costs (costs dom))
	(aggregator (ecase type (:optimistic #'mymax) (:pessimistic #'mymin))))
    (make-simple-description 
     #'(lambda (s)
	 (remove-duplicates
	  (to-list
	   (ndunion ((state s))
	     (when (< (second state) (n dom)) 
	       (list (succ-state state desc)))))
	  :test #'equal))
     #'(lambda (s1 s2)
	 (remove-duplicates 
	  (to-list
	   (intersect s1
		      (ndunion ((state s2)) ;; we have to use an ndunion rather than because s2 may not be a list
			(when (and (= (first state) name) (> (second state) 0)) 
			  `((0 ,(1- (second state))) (1 ,(1- (second state))))))))
	  :test #'equal))
     #'(lambda (s1 s2) (reduce-set aggregator s1 :key #'(lambda (state) (if (member? (action-result dom state name) s2) (my- (aref costs (first state) (second state) name)) '-infty)))))))


;; Traverse
(defmethod action-description ((d <two-by-n-descriptions>) (name (eql 'traverse)) args type &aux (dom (planning-domain d)))
  (make-simple-description
   #'(lambda (s)
       (remove-duplicates 
	(to-list
	 (ndunion ((state s)) 
	   (dsbind (m i) state 
	     (declare (ignore m)) 
	     (when (< i (n dom)) 
	       `((0 ,(1+ i)) (1 ,(1+ i)))))))
	:test #'equal))
   #'(lambda (s1 s2) 
       (remove-duplicates  
	(to-list
	 (intersect s1 
		    (ndunion ((state s2)) 
		      (dsbind (m i) state 
			(declare (ignore m)) 
			(when (> i 0) 
			  `((0 ,(1- i)) (1 ,(1- i))))))))
	:test #'equal))
   (ecase type (:optimistic 0) (:pessimistic '-infty))))

;; Top
(defmethod action-description ((d <two-by-n-descriptions>) (name (eql 'top)) args type &aux (dom (planning-domain d)))
  (declare (ignore args))
  (make-simple-description
   (goal dom) '((0 0) (1 0))
   (ecase type (:optimistic 0) (:pessimistic '-infty))))


				   


