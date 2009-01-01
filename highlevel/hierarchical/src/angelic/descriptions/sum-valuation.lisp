(in-package :hla)

(defclass <sum-valuation> ()
  ((vals :accessor sum-valuation-vals :initarg :vals)))



(defun make-sum-valuation (&rest vals)
  (make-instance '<sum-valuation> :vals vals))

(defmethod max-achievable-value ((v <sum-valuation>))
  (with-accessors ((vals sum-valuation-vals)) v
    (assert (= 2 (length vals)) nil "max-achievable-value currently only implemented for sums of two valuations (easy to extend using DP if necessary)")
    (let ((comps (mapcar #'(lambda (val) (etypecase val (simple-valuation (list val)) (<max-valuation> (max-valuation-vals val)))) vals)))
      (reduce-set #'mymax (fast-product comps)
		  :key #'(lambda (v)
			   (dbind (val1 val2) v
			     (if (intersects (sv-s val1) (sv-s val2))
				 (my+ (sv-v val1) (sv-v val2))
				 '-infty)))))))

(defmethod reachable-set ((v <sum-valuation>))
  (reduce #'binary-union (sum-valuation-vals v) :key #'reachable-set))

(defmethod evaluate-valuation ((v <sum-valuation>) s)
  (reduce #'my+ (sum-valuation-vals v) :key #'(lambda (val) (evaluate-valuation val s))))

(defmethod equal-valuations ((v1 <sum-valuation>) (v2 <sum-valuation>))
  (and (= (length (sum-valuation-vals v1)) (length (sum-valuation-vals v2)))
       (every #'equal-valuations (sum-valuation-vals v1) (sum-valuation-vals v2))))
	       

(set-pprint-dispatch '<sum-valuation> #'pprint-valuation-as-list)


    
