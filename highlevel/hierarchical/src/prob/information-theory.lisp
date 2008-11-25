;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; TODO: Feb 08 conditional-entropy and mutual-info don't 
;; work and not sure why
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package prob)

(defparameter *base* 2)
(defvar *info-theory-debug-level* 100)

(defun entropy (dist)
  "entropy PROB-DIST.  Return the entropy of PROB-DIST."
  (let ((tot 0))
    (do-sample-points (u p dist (progn (assert (> tot -.01)) (max tot 0.0)))
      (incf tot (iunless (zerop p) (- (* p (log p *base*))))))))


(defun conditional-entropy (dist x y &key (x-test #'equal) (y-test #'equal))
  "conditional-entropy DIST X Y &key (X-TEST #'equal) (Y-TEST #'equal)
Conditional entropy of X given Y, both of which are random variables or lists thereof w.r.t. DIST.  X-TEST and Y-TEST are used to check equality of X and Y values."
  (let* ((x (designated-list x))
	 (y (designated-list y))
	 (x-tests (make-array (length x) :initial-element x-test))
	 (y-tests (make-array (length y) :initial-element y-test))
	 (joint (make-instance '<tabular-dist> :dist dist 
			       :vars (append y x) :tests (concatenate 'vector y-tests x-tests) :iterate-quickly t))
	 (marg (make-instance '<tabular-dist> :dist dist :vars y :tests y-tests :iterate-quickly t)))
    (- (entropy joint) (entropy marg))))


(defun mutual-information (dist x y &key (x-test #'equal) (y-test #'equal))
  "mutual-information DIST X Y &key (X-TEST #'equal) (Y-TEST #'equal).  Mutual information of random variables X and Y.  X-TEST and Y-TEST are equality tests for their values."
  (let ((x (designated-list x))
	(y (designated-list y)))
    (let ((x-tests (make-array (length x) :initial-element x-test))
	  (y-tests (make-array (length y) :initial-element y-test)))
      (let ((x-dist (make-instance '<tabular-dist> :dist dist :vars x :tests x-tests :iterate-quickly t))
	    (y-dist (make-instance '<tabular-dist> :dist dist :vars y :tests y-tests :iterate-quickly t))
	    (xy-dist (make-instance '<tabular-dist> :dist dist :vars (append x y) 
				    :tests (concatenate 'vector x-tests y-tests) :iterate-quickly t)))
	(let ((x-entropy (entropy x-dist))
	      (y-entropy (entropy y-dist))
	      (xy-entropy (entropy xy-dist)))
	
	  (debug-print 
	   *info-theory-debug-level* 
	   "Mutual information~&X-dist : ~a~&X-entropy : ~a~&Y-dist : ~a~&Y-entropy : ~a~&XY-dist : ~a~& XY-entropy : ~a"
	   x-dist x-entropy y-dist y-entropy xy-dist xy-entropy)
	  (+ x-entropy y-entropy (- xy-entropy)))))))
     



(defun normalized-conditional-mutual-information (dist x y z)
  "normalized-conditional-mutual-information DIST X Y Z.

DIST must be of type [prob-dist], and X,Y,Z must be of type [random-variable] or lists of random variables

Return mutual information of X and Y conditional on Z, normalized by the average of H(X|Z) and H(Y|Z).  So, 0 indicates conditional independence, and 1 indicates (conditional) functional dependence."

  (let ((x (designated-list x))
	(y (designated-list y))
	(z (designated-list z)))
    (flet ((entropy (v) (entropy (make-instance '<tabular-dist> :dist dist :vars v :iterate-quickly t))))
      (let ((hxz (entropy (append x z)))
	    (hyz (entropy (append y z)))
	    (hxyz (entropy (append x y z)))
	    (hz (entropy z)))
	(let ((chx (- hxz hz))
	      (chy (- hyz hz))
	      (chxy (- hxyz hz)))
	  (let ((m (avg chx chy .5)))
	    (iunless (zerop m)
		     (/ (+ chx chy (- chxy)) m))))))))
