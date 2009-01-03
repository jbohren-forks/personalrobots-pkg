;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; array-utils.lisp
;;
;; general utilities for handling arrays
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package utils)



(defun round-array (a &optional (d 2))
  "round-array A &optional (D 2).  return a new array in which all elements of array a are rounded to d decimal places (usually used for displaying).  Coerces all elements to floats in the process.  Inefficient implementation intended only for debugging."
  
  (let* ((dims (array-dimensions a))
	 (is-1d (= (length dims) 1)))
    (condlet ((is-1d (b-dims (length a)) (tot-size (length a)))
	      ((not is-1d) (b-dims dims) (tot-size (array-total-size a))))
	     (loop
		 with b = (make-array b-dims)
	       
		 for i below tot-size
		 do (setf (row-major-aref b i)
		      (round-decimal (row-major-aref a i) d))
		    
	 
		 finally (return b)))))

(defun map-array (f &rest arrays)
  "map-array FN &rest ARRAYS.  ARRAYS = A1...An must all be arrays with the same dimensions (and n must be >= 1).  Create a new array with the same dimension, where A[indices] = FN (A1[indices] A2[indices]... An[indices])."
  (let* ((dims (array-dimensions (first arrays)))
	 (result (make-array dims))
	 (l (make-list (length arrays))))
    (assert (every #'(lambda (a) (equal dims (array-dimensions a))) (rest arrays))
	nil "All array arguments to map-array must have same dimensions.")
    (dotimes (i (array-total-size result) result)
      (setf (row-major-aref result i)
	(apply f (map-into l #'(lambda (x) (row-major-aref x i)) arrays))))))



(defun a+ (&rest arrays)
  "a+ &rest ARRAYS.  Elementwise addition."
  (if (numberp (first arrays))
      (apply #'+ arrays)
    (apply #'map-array #'+ arrays)))


(defun a- (&rest arrays)
  "a- &rest ARRAYS.  Elementwise subtraction."
  (apply #'map-array #'- arrays))

(defun inner-product (a1 a2)
  (loop for x across a1 for y across a2 summing (* x y)))

(defgeneric convex-combination (a1 a2 b)
  (:documentation "convex-combination A1 A2 B.  Return B*A1 + (1-B)*A2 for numbers or arrays.")
  (:method ((a1 number) (a2 number) b) (+ (* b a1) (* (- 1 b) a2)))
  (:method ((a1 vector) (a2 vector) b) (let ((n (length a1))) 
					 (assert (= n (length a2))) 
					 (let ((x (make-array n :initial-element 0.0 :element-type 'float)))
					   (dotimes (i n x)
					     (setf (aref x i) (+ (* b (aref a1 i)) (* (- 1 b) (aref a2 i)))))))))

(defun mv* (m w)
  "mv* MATRIX VECTOR.  Matrix-vector multiplication."
  (dbind (i j) (array-dimensions m)
    (assert (= j (length w)))
    (let ((v (make-array i :initial-element 0.0 :element-type 'float)))
      (dotimes (a i v)
	(setf (aref v a)
	  (loop 
	      for b below j
	      sum (* (aref m a b) (aref w b))))))))


(defun a* (a1 a2)
  "a* A1 A2.  Various cases
1) A1 is a number and A2 is any array (or vice versa), then multiply each element of A2 by A1 
2) Both are vectors.  Do an inner-product
3) A1 is a vector, A2 is a matrix.  Do a row-vector x matrix multiplication
4) A1 is a matrix, A2 is a vector.  Do a matrix x column-vector multiplication
5) Both are matrices.  Do a matrix multiplication using the straightforward O(mnk) algorithm.

Otherwise error.  In cases 2-5, error if dimensions don't match up.

See also inner-product."
  
  (if (numberp a1)
      (if (numberp a2)
	  (* a1 a2)
	(map-array (lambda (x) (* x a1)) a2))
    (if (numberp a2)
	(map-array (lambda (x) (* x a2)) a1)
      (if (= 1 (array-rank a1))
	  (if (= 1 (array-rank a2))
	      (dot-product a1 a2)
	    (mv* (transpose a2) a1))
	(if (= 1 (array-rank a2))
	    (mv* a1 a2)
	  (matrix-multiply a1 a2))))))					     

(defun a/ (a x)
  "a/ A X.  A is an array, X is a real number.  Return a new array with same dimensions as A, with elements divided by X."
  (map-array #'(lambda (y) (/ y x)) a))


(defun dot-product (a1 a2)
  "dot-product V1 V2. Inner product."
  (assert (= (length a1) (length a2)))
  (loop for x across a1 for y across a2 summing (* x y)))



(defun matrix-multiply (a b)
  (dbind (m n) (array-dimensions a)
    (assert (= n (array-dimension b 0)))
    (let ((k (array-dimension b 1)))
      (let ((c (make-array (list m k) :initial-element 0.0 :element-type 'float)))
	(dotimes (i m c)
	  (dotimes (j k)
	    (setf (aref c i j) (loop for l below n summing (* (aref a i l) (aref b l j))))))))))



(defun array-size-multipliers (a)
  "array-size-multipliers A.  Return the multipliers needed to step through the various dimensions of the array.  For example, an array with dimensions 3,5,4,2 would have multipliers 40,8,2,1"
  (loop
      with step = 1
      with mults = nil
      for mult in (cons 1 (reverse (rest (array-dimensions a))))
      do (push (setf step (* step mult)) mults)
      finally (return mults)))
		  





(defun subarray (a inds)
  "subarray A INDS.  Returns an array consisting of all entries of A whose index begins with INDS (which is a list).  E.g., if A has dimensions 2,3,4,3 then subarray(a,(1 2)) will return the 4x3 array a[1,2,0,0],...,a[1,2,3,2].  Note that the new array shares structure with the original one, i.e. if the entries are modified, then the original array will be changed as well."
  (let* ((offset 0)
	 (newdims
	  (loop
	      with indices = inds
	      for dim in (array-dimensions a)
	      for mult in (array-size-multipliers a)
	      for ind = (first indices)

	      if ind
	      do (setf indices (rest indices)
		       offset (+ offset (* mult ind)))
		 
	      else
	      collect dim)))


		 
    (make-array newdims :displaced-to a :displaced-index-offset offset)))
	
(defun set-subarray (a inds new-vals)
  (loop
      for ind from (loop 
		       for i in inds 
		       for mult in (array-size-multipliers a)
		       sum (* i mult))
      for x across new-vals
      do (setf (row-major-aref a ind) x)
      finally (return new-vals)))
  

(defsetf subarray set-subarray)



(defun reshape-array (a dims &key (clone-p nil))
  "reshape-array A DIMS &key (CLONE-P nil).  Reshape array to new dimensions DIMS (which must have the right total number of elements).  If CLONE-P is true, then items are cloned."
  (let ((old-dims (array-dimensions a))
	(dims (if (integerp dims) (list dims) dims)))
    (assert (equal (apply #'* dims) (apply #'* old-dims))
	() "Array of size ~a cannot be converted to array of size ~a!" old-dims dims))
  
  (loop
      with new-a = (make-array dims)
      for i below (array-total-size a)
      do (setf (row-major-aref new-a i)
	   (let ((x (row-major-aref a i)))
	     (if clone-p
		 (clone x)
	       x)))
      finally (return new-a)))


(defun sparsify (v &key test (name-fn #'identity))
  "sparsify V &optional TEST NAME-FN.  Return a vector w each of whose elements is a pair (ind . val), which  means that v(ind) = val, for all the nonzero elements of v.  w is in increasing order of ind.  TEST is used to select which elements are included.  By default, all elements that are nonzero are included.  NAME-FN is applied to ind before printing it.  By default, its the identity function."
  (loop
      with w = (make-array 0 :adjustable t :fill-pointer 0)
      with include? = (or test (lambda (x) (/= x 0)))
      for x across v
      for i from 0
      when (funcall include? x)
      do (vector-push-extend (cons (funcall name-fn i) x) w)
      finally (return w)))


(defun diag (v)
  "diag V.  Return diagonal matrix whose diagonal elements are V."
  (let* ((n (length v))
	 (a (make-array (list n n) :initial-element 0.0)))
    (dotimes (i n a)
      (setf (aref a i i) (aref v i)))))

(defun transpose (a)
  "transpose A.  Matrix transpose of a 2d array."
  (dbind (m n) (array-dimensions a)
    (let ((b (make-array (list n m))))
      (dotimes (i m b)
	(dotimes (j n)
	  (setf (aref b j i) (aref a i j)))))))
	

(defun array-lp-dist (p m1 &optional (m2 nil m2-supp))
  (if (numberp p)
      (let ((s 0))
	(expt
	 (dotimes (i (array-total-size m1) s)
	   (incf s (expt (abs (- (row-major-aref m1 i) 
				 (if m2-supp 
				     (row-major-aref m2 i)
				   0)))
			 p)))
	 (/ 1 p)))
    (progn
      (assert (eq p 'infty))
      (assert (not m2-supp))
      (loop 
	  for i below (array-total-size m1)
	  maximizing (abs (row-major-aref m1 i))))))
      
(defun bsearch (a x &key (test #'my<))
  "bsearch A X &key (TEST #'my<)

TEST is assumed to be a transitive irreflexive ordering.
A is assumed to be an array that is sorted according to TEST.  This function uses binary search, and returns I such that 1) I=length(A) or A[I] >= X 2) A[I-1] < X or I=0."
  (flet ((smaller (x y) (funcall test x y))
	 (not-smaller (x y) (not (funcall test x y)))
	 (middle-ind (i j) (+ i (floor (- j i) 2))))
    (let ((n (length a)))
      
      (cond
       ((not-smaller (aref a 0) x) 0)
       ((smaller (aref a (1- n)) x) n)
       (t (let ((i 0)
		(j (1- n)))
	    (loop
	      (if (= j (1+ i)) 
		  (return j)
		(let ((k (middle-ind i j)))
		  (if (smaller (aref a k) x)
		      (setf i k)
		    (setf j k)))))))))))
		    

(defun make-adjustable-array (&key (initial-contents nil))
  "make-adjustable-array &key (INITIAL-CONTENTS nil).
Return an adjustable array, whose initial contents are the elements of INITIAL-CONTENTS in order."

  (let ((a (make-array 0 :adjustable t :fill-pointer 0)))
    (dolist (c initial-contents a)
      (vector-push-extend c a))))

(defun append-to-adjustable-array (a s)
  "Destrutive operation: add the elements in S to the end of adjustable array A in forward order."
  (map nil #'(lambda (x) (vector-push-extend x a)) s))

(in-package cl-user)

