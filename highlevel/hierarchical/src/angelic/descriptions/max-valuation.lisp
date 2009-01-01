(in-package :hla)


(defclass <max-valuation> ()
  ((vals :accessor max-valuation-vals))
  (:documentation "Pointwise max of a bunch of simple valuations"))


(defmethod initialize-instance :after ((v <max-valuation>) &rest args &key vals)
  (declare (ignore args))
  (let* ((sorted-vals
	  (sort (mapcan #'(lambda (v)
			     (typecase v
			       (<max-valuation> (max-valuation-vals v))
			       (otherwise (list v))))
			 vals)
		#'my< :key #'sv-v))
	 (undominated-vals
	  (coerce
	  (mapcon
	   #'(lambda (l)
	       (let ((v1 (first l)))
		 (unless (some #'(lambda (v2) (pointwise-subsumes v2 v1)) (rest l))
		   (list v1))))
	   sorted-vals)
	  'vector)))
	     

    
    (setf (max-valuation-vals v)
	  (mapcar
	   #'(lambda (b)
	       (dsbind (val start end) b
		 (make-simple-valuation
		  (reduce #'unite undominated-vals :key #'sv-s :start start :end end)
		  val)))
	   (contiguous-blocks undominated-vals :key #'sv-v)))))
    


(defun make-max-valuation (vals)
  (make-instance '<max-valuation> :vals vals))

(defmethod reachable-set ((v <max-valuation>))
  (reduce #'binary-union (max-valuation-vals v) :key #'reachable-set))

(defmethod max-achievable-value ((v <max-valuation>))
  (reduce #'mymax (max-valuation-vals v) :key #'max-achievable-value))

(def-symmetric-method binary-pointwise-max-upper-bound ((v1 <max-valuation>) v2)
  (make-max-valuation (cons v2 (max-valuation-vals v1))))

;; same as upper bound, as it is exact
(def-symmetric-method binary-pointwise-max-lower-bound ((v1 <max-valuation>) v2)
  (make-max-valuation (cons v2 (max-valuation-vals v1))))


(def-symmetric-method binary-pointwise-min-upper-bound ((v1 <max-valuation>) v2)
  (make-max-valuation
   (mapcar #'(lambda (v) (binary-pointwise-min-upper-bound v v2)) (max-valuation-vals v1))))
   

(defmethod evaluate-valuation ((v <max-valuation>) s)
  (reduce #'mymax (max-valuation-vals v) :key #'(lambda (val) (evaluate-valuation val s))))

(defmethod equal-valuations ((v1 <max-valuation>) (v2 <max-valuation>))
  (and (= (length (max-valuation-vals v1)) (length (max-valuation-vals v2)))
       (every #'equal-valuations (max-valuation-vals v1) (max-valuation-vals v2))))



(defmethod print-valuation ((v <max-valuation>) &optional (str t))
  (pprint-logical-block (str (max-valuation-vals v))
    (format str "Max valuation")
    (do-elements (val (max-valuation-vals v) nil i)
      (pprint-pop)
      (format str "~:@_ Valuation ~a:~:@_  ~/hla::pprint-valuation/" i val))))
      



(set-pprint-dispatch '<max-valuation> #'pprint-valuation-as-list)