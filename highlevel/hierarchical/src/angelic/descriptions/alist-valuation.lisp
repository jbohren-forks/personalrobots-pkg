(in-package :hla)

(defmethod evaluate-valuation ((v list) s)
  (aif (assoc s v :test #'same-state)
    (cdr it)
    '-infty))

(defun remove-alist-valuation-duplicates (v)
  "Destructive operation that returns a new alist valuation with duplicate items removed."
  (let ((l nil))
    (dolist (pair v l)
      (dsbind (item . val) pair
	(let ((existing-pair (assoc item l :test #'same-state)))
	  (if existing-pair
	    (_f mymax (cdr existing-pair) val)
	    (when (my> val '-infty)
	      (push pair l))))))))

(defmethod pointwise-subsumes ((v1 list) (v2 list))
  (dolist (pair v2 t)
    (dsbind (s . val) pair
      (unless (my>= (evaluate-valuation v1 s) val)
	(return nil)))))

(defmethod reachable-set ((v list))
  (mapcan #'(lambda (pair) (when (my> (cdr pair) '-infty) (list (car pair)))) v))

(defmethod max-achievable-value ((v list))
  (reduce #'mymax v :key #'cdr))

(defmethod binary-pointwise-min-upper-bound ((v1 list) (v2 list))
  (mapcan
   #'(lambda (pair)
       (dsbind (item . val) pair
	 (let ((min-val (mymin val (evaluate-valuation v2 item))))
	   (when (my> min-val '-infty)
	     (list (cons item min-val))))))
   v1))

(defun max-alist-valuations (v1 v2)
  (remove-alist-valuation-duplicates (nconc (copy-alist v1) (copy-alist v2))))
		
(defmethod binary-pointwise-max-upper-bound ((v1 list) (v2 list))
  (max-alist-valuations v1 v2))

(defmethod binary-pointwise-max-lower-bound ((v1 list) (v2 list))
  (max-alist-valuations v1 v2))

(defmethod print-valuation ((v list) &optional (str t))
  (pprint-logical-block 
   (str nil)
   (pprint-fill str v)))

(defun alist-progressor (e a)
  #'(lambda (v)
      (remove-alist-valuation-duplicates
       (mapcar 
	#'(lambda (pair)
	    (dsbind (s . val) pair
	      (cons (action-result e s a) (my+ val (reward e s a)))))
	v))))

(defun alist-regressor (e a)
  #'(lambda (v1 v2)
      (mapcan #'(lambda (pair)
		  (dsbind (s . val) pair
		    (when (my> val '-infty)
		      (let ((s2 (action-result e s a)))
			(let ((pair2 (assoc s2 v2 :test #'same-state)))
			  (when pair2
			    (list (cons s (my+ (cdr pair2) (reward e s a))))))))))
	      v1)))
      
(defun sum-alist-valuations (vals)
  (cond
    ((null vals) nil)
    ((null (cdr vals)) (car vals))
    (t (let ((v1 (car vals))
	     (v2 (sum-alist-valuations (cdr vals))))
	 (remove-alist-valuation-duplicates
	  (let ((l nil))
	    (dolist (val1 (list v1 v2) l)
	      (let ((val2 (if (eq val1 v1) v2 v1)))
		(dolist (pair val1)
		  (dsbind (s . val) pair
		    (push (cons s (my+ val (evaluate-valuation val2 s))) l)))))))))))