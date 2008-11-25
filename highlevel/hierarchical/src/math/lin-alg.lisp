;; Package info
(defpackage lin-alg
  (:use 
   common-lisp
   set
   utils)
  (:documentation "Package lin-alg

lp-dist
l2-dist
l1-dist
is-diagonal

inner-product
norm
unit-vector
angle
2d-unit-normal
2d-normal

eye

lu-decomposition
inverse
determinant

cholesky

svd
svd-backsub
svd-fit")
  
  
  (:export 
   lp-dist 
   l2-dist
   l1-dist
   is-diagonal
   
   inner-product
   norm
   unit-vector
   angle
   2d-unit-normal
   2d-normal
   
   eye

   lu-decomposition
   inverse
   determinant
   
   cholesky
   
   svd 
   svd-backsub
   svd-fit
   ))

(in-package lin-alg)


(defmacro aref1 (a &rest inds)
  `(aref ,a ,@(mapcar #'(lambda (x) `(1- ,x)) inds)))


(defgeneric lp-dist (a1 a2 &optional p) ;(p 1))
  ;; This should really be a defun, but is generic for backward compatibility
  (:documentation "lp-dist A1 A2 &optional (P 1)
Return the l^p distance between two arrays of reals.  P is the norm to use. Either a nonnegative real, or the symbol 'infty (or 'infinity for backward compatibility)  to indicate infinity-norm")
  (:method (a1 a2 &optional (p 1))
	   (flet ((num-elts (a)
		    (if (vectorp a) (length a) (array-total-size a))))
	     (let ((n (num-elts a1)))
	       (assert (= n (num-elts a2)))
	       (cond
		;; L0
		((eql p 0) 
		 (sum-over n #'(lambda (i) (indicator (not (= (row-major-aref a1 i) (row-major-aref a2 i)))))))
	      
		;; L^p
		((numberp p)
		 (expt
		  (sum-over n #'(lambda (i) (expt (abs-diff (row-major-aref a1 i) (row-major-aref a2 i)) p)))
		  (/ 1 p)))
	      
		;; L^infinity
		(t
		 (reduce-set #'mymax n :key #'(lambda (i) (abs-diff (row-major-aref a1 i) (row-major-aref a2 i))))))))))

(defun l2-dist (a1 a2)
  "l2-dist A1 A2.  L2 distance"
  (sqrt
   (loop 
       for x across a1
       for y across a2
       sum (expt (- x y) 2))))

(defun l1-dist (a1 a2)
  "l1-dist A1 A2.  L1 distance"
  (loop
      for x across a1
      for y across a2
      sum (abs-diff x y)))



(defun is-diagonal (m)
  "return true iff argument is a square matrix with 0's off the diagonal."
  (dbind (i j) (array-dimensions m)
    (and (= i j)
	 (dotimes (k i t)
	   (dotimes (l i)
	     (unless (= k l)
	       (unless (zerop (aref m k l))
		 (return-from is-diagonal nil))))))))


(defun norm (v)
  "norm X.  2-norm of vector X."
  (sqrt (sum-over v #'(lambda (x) (* x x)))))

(defun unit-vector (v)
  "unit-vector V.  Return a unit vector in the direction of V, or the zero vector if V is 0."
  (let ((n (norm v)))
    (if (zerop n)
	v
      (a/ v n))))

(defun angle (v1 v2)
  "angle in radians between V1 and V2.  If they're close to 0, signal a continuable error and return 0.

The angle is signed and counterclockwise, so that:
- angle((1,0),(0,1)) = pi/2
- angle((1,0),(0,-1)) = -pi/2"
  (let ((n1 (norm v1))
	(n2 (norm v2)))
    (cond 
     ((< (min n1 n2) .00001)
      (cerror "Use angle 0." "Attempted to find angle between vectors ~a and ~a, one of which is close to 0" v1 v2)
      (return-from angle 0.0))
     (t
      (let ((x1 (/ (sfirst v1) n1))
	    (x2 (/ (sfirst v2) n2))
	    (y1 (/ (ssecond v1) n1))
	    (y2 (/ (ssecond v2) n2)))
	(atan (- (* x1 y2) (* x2 y1)) (+ (* x1 x2) (* y1 y2))))))))

(defun 2d-unit-normal (v)
  "2d-unit-normal V.  Return the unit normal (in the counterclockwise direction) to 2d-vector V."
  (let ((r (norm v)))
    (if (< r .00000001)
	#(0 0)
	(dbind (x y) v
	  (vector (/ (- y) r) (/ x r))))))

(defun 2d-normal (v)
  "2d-normal V.  Return normal (in counterclockwise direction) to 2d-vector V, having same norm as V."
  (dbind (x y) v
    (vector (- y) x)))


(defun eye (n)
  "Return the identity NxN matrix."
  (let ((m (make-array (list n n) :element-type 'float :initial-element 0.0)))
    (dotimes (i n m)
      (setf (aref m i i) 1.0))))


(defun lu-decomposition (m &key (tol .0001))
  "lu-decomposition M.  Return two matrices L and U such that M = UL and L (resp U) is lower (resp upper) triangular.  Note that this version is numerically unstable."
  ;; TODO fix the numerical problems
  
  (let ((n (array-dimension m 0)))
    (let ((u (clone m))
	  (max '-infty)
	  (l (eye n)))
      
      ;; remove close to 0 elements
      (dotimes (i n)
	(dotimes (j n)
	  (maxf max (abs (aref m i j)))))
      (let ((threshold (* max tol)))
	(dotimes (i n)
	  (dotimes (j n)
	    (when (< (abs (aref m i j)) threshold)
	      (setf (aref u i j) 0.0)))))
      
      (dotimes (i (1- n) (values l u))
	(let ((a (aref u i i)))
	  (assert (not (zerop a)))
	  (for-loop (j (1+ i) n)
	    (let ((c (/ (aref u j i) a)))
	      (unless (zerop c)
		(for-loop (k i n)
		  (decf (aref u j k) (* c (aref u i k)))
		  (incf (aref l k i) (* c (aref l k j))))))))))))
  
  

(defun transpose-in-place (m)
  (let ((n (array-dimension m 0)))
    (dotimes (i n m)
      (dotimes (j i)
	(rotatef (aref m i j) (aref m j i))))))

(defun invert-lower-triangular (m)
  (let* ((n (array-dimension m 0))
	 (inv (make-array (list n n) :element-type 'float :initial-element 0.0)))
    (dotimes (i n inv)
      (dotimes (j (1+ i))
	(setf (aref inv i j)
	  (/ (if (= i j) 
		 1
	       (- (loop for k from j below i summing (* (aref m i k) (aref inv k j)))))
	     (aref m i i)))))))


(defun inverse (m)
  (mvbind (l u) (lu-decomposition m)
    (a* (transpose-in-place (invert-lower-triangular (transpose-in-place u)))
	(invert-lower-triangular l))))

(defun determinant (m)
  (let ((n (array-dimension m 0)))
    (mvbind (l u) (lu-decomposition m)
      (reduce-set #'* n :key #'(lambda (i) (* (aref l i i) (aref u i i)))))))
    
    
    

(defun cholesky (m &aux (n (when (arrayp m) (array-dimension m 0))))
  "cholesky M.  M is assumed symmetric positive-definite.  Returns L such that LL' = M.  Also, if given a number returns its square root."
  ;; TODO: this could be modified to use O(n^2) memory instead of O(n^3)
  (if (numberp m)
      (sqrt (verify-type m (real 0.0 *) "nonnegative number"))
    (let ((l (eye n))
	  (a (clone m))
	  (new-l (eye n)))
      (dotimes (i n l)

	;; Revert new-l to the identity matrix
	(unless (zerop i)
	  (setf (aref new-l (1- i) (1- i)) 1.0)
	  (for-loop (j i n)
	    (setf (aref new-l j (1- i)) 0.0)))

	;; Add the new entries
	(let ((root (sqrt (if (> (aref a i i) 0) (aref a i i) 0))))
	  (setf (aref new-l i i) root)
	  (for-loop (j (1+ i) n)
	    (setf (aref new-l j i) (/ (aref a j i) root))))
      
      
	;; Multiply into the overall l
	(setf l (a* l new-l))
      
	(let ((div (if (> (aref a i i) 0) (/ 1 (aref a i i)) 0)))
	  ;; Modify a
	  (for-loop (j (1+ i) n)
	    (for-loop (k (1+ i) n)
	      (decf (aref a j k) (* (aref a j i) (aref a k i) div)))))))))

(in-package cl-user)


