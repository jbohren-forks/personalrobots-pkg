;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; svd.lisp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(in-package lin-alg)



(defparameter *svd-eps* .0001)
(defparameter *svdfit-tol* .00001)

(defun svd (a)
  "svd A
A: mxn matrix (array) with m>n

Performs a singular value decomposition of A, and returns 3 values:
1. An mxn matrix U
2. An n-vector W
3. An nxn matrix V

such that
1. U'U = I_n
2. V'V = I_n
3. U diag(W) V' = A"

  (utils:dbind (m n) (array-dimensions a)
    (let ((w (make-array n :element-type 'float))
	  (v (make-array (list n n) :element-type 'float))
	  (a (utils:clone a)))
      
      (let (flag i nm c f h s x y z
	    (rv1 (make-array n))
	    (g 0.0)
	    (scale 0.0)
	    (anorm 0.0))
	
	(loop
	    with l = 0
	    for i from 1 to n
	    do (setf l (1+ i)
		     (aref1 rv1 i) (* scale g)
		     g 0
		     s 0
		     scale 0)
	       (when (<= i m)
		 (loop for k from i to m
		     do (incf scale (abs (aref1 a k i))))
		 (unless (zerop scale)
		   (loop for k from i to m
		       do (_f / (aref1 a k i) scale)
			  (incf s (expt (aref1 a k i) 2)))
		   (setf f (aref1 a i i)
			 g (- (sign (sqrt s) f))
			 h (- (* f g) s)
			 (aref1 a i i) (- f g))
		   (loop for j from l to n
		       do (setf s
			    (loop for k from i to m
				sum (* (aref1 a k i) (aref1 a k j)))
			    f (/ s h))
			  (loop for k from i to m
			      do (incf (aref1 a k j) (* f (aref1 a k i)))))
		   (loop for k from i to m
		       do (_f * (aref1 a k i) scale))))
	       (setf (aref1 w i) (* scale g)
		     g 0
		     s 0
		     scale 0)
	       (when (and (<= i m) (not (= i n)))
		 (setf scale (loop for k from l to n summing (abs (aref1 a i k))))
		 (unless (zerop scale)
		   (loop for k from l to n
		       do (_f / (aref1 a i k) scale)
			  (incf s (expt (aref1 a i k) 2)))
		   (setf f (aref1 a i l)
			 g (- (sign (sqrt s) f))
			 h (- (* f g) s)
			 (aref1 a i l) (- f g))
		   (loop for k from l to n
		       do (setf (aref1 rv1 k) (/ (aref1 a i k) h)))
		   (loop for j from l to m
		       do (setf s (loop for k from l to n summing (* (aref1 a j k) (aref1 a i k))))
			  (loop for k from l to n do (incf (aref1 a j k) (* s (aref1 rv1 k)))))
		   (loop for k from l to n do (_f * (aref1 a i k) scale))))
	       (setf anorm (fmax anorm (+ (abs (aref1 w i)) (abs (aref1 rv1 i))))))
	
	(loop for i from n downto 1
	    with l = (1+ n)
	    do (when (< i n)
		 (unless (zerop g)
		   (loop for j from l to n
		       do (setf (aref1 v j i) (/ (/ (aref1 a i j) (aref1 a i l)) g)))
		   (loop for j from l to n 
		       do (setf s (loop for k from l to n summing (* (aref1 a i k) (aref1 v k j))))
			  (loop for k from l to n do (incf (aref1 v k j) (* s (aref1 v k i))))))
		 (loop for j from l to n
		     do (setf (aref1 v i j) 0
			      (aref1 v j i) 0)))
	       (setf (aref1 v i i) 1
		     g (aref1 rv1 i)
		     l i))
	
	(loop for i from (imin m n) downto 1
	    for l = (1+ i)
	    do (setf g (aref1 w i))
	       (loop for j from l to n do (setf (aref1 a i j) 0))
	       (if (not (zerop g))
		   (progn
		     (setf g (/ 1 g))
		     (loop for j from l to n
			 do (setf s (loop for k from l to m summing (* (aref1 a k i) (aref1 a k j)))
				  f (* (/ s (aref1 a i i)) g))
			    (loop for k from i to m 
				do (incf (aref1 a k j) (* f (aref1 a k i)))))
		     (loop for j from i to m do (_f * (aref1 a j i) g)))
		 (loop
		     for j from i to m do (setf (aref1 a j i) 0)))
	       (incf (aref1 a i i)))
	
	(loop 
	    with l = 0
	    for k from n downto 1
	    do (loop for its from 1 to 30
		   do (setf flag 1)
		      (setf l
		      (loop for l from k downto 1
			  do (setf nm (1- l))
			     (when (< (abs (aref1 rv1 l)) (* anorm *svd-eps*))
			       (setf flag 0)
			       (return l))
			     (when (< (abs (aref1 w nm)) (* anorm *svd-eps*))
			       (return l))))
		      (unless (zerop flag)
			(setf c 0
			      s 1)
			(loop for i from l to k
			    do (setf f (* s (aref1 rv1 i))
				     (aref1 rv1 i) (* c (aref1 rv1 i)))
			       (when (< (abs f) (* anorm *svd-eps*))
				 (return nil))
			       (setf g (aref1 w i)
				     h (pythag f g)
				     (aref1 w i) h
				     h (/ 1 h)
				     c (* g h)
				     s (- (* f h)))
			       (loop for j from 1 to m 
				   do (setf y (aref1 a j nm)
					    z (aref1 a j i)
					    (aref1 a j nm) (+ (* z s) (* y c))
					    (aref1 a j i) (- (* z c) (* y s))))))
		      
		      (setf z (aref1 w k))
		      (when (= l k)
			(when (< z 0)
			  (setf (aref1 w k) (- z))
			  (loop for j from 1 to n do (_f * (aref1 v j k) -1)))
			(return nil))
		      
		      (assert (not (= its 30)) nil "no convergence in 30 svd iterations")
		      (setf x (aref1 w l)
			    nm (1- k)
			    y (aref1 w nm)
			    g (aref1 rv1 nm)
			    h (aref1 rv1 k)
			    f (/ (+ (* (- y z) (+ y z)) (* (- g h) (+ g h))) (* 2 h y))
			    g (pythag f 1)
			    f (/ (+ (* (- x z) (+ x z)) (* h (- (/ y (+ f (sign g f))) h))) x)
			    c 1
			    s 1)
		      (loop for j from l to nm
			  do (setf i (1+ j)
				   g (aref1 rv1 i)
				   y (aref1 w i)
				   h (* s g)
				   g (* c g)
				   z (pythag f h)
				   (aref1 rv1 j) z
				   c (/ f z)
				   s (/ h z)
				   f (+ (* x c) (* g s))
				   g (- (* g c) (* x s))
				   h (* y s)
				   y (* c y))
			     (loop for jj from 1 to n
				 do (setf x (aref1 v jj j)
					  z (aref1 v jj i)
					  (aref1 v jj j) (+ (* x c) (* z s))
					  (aref1 v jj i) (- (* z c) (* x s))))
			     (setf z (pythag f h)
				   (aref1 w j) z)
			     (unless (zerop z)
			       (setf z (/ 1 z)
				     c (* f z)
				     s (* h z)))
			     
			     (setf f (+ (* c g) (* s y))
				   x (- (* c y) (* s g)))
			     (loop for jj from 1 to m
				 do (setf y (aref1 a jj j)
					  z (aref1 a jj i)
					  (aref1 a jj j) (+ (* y c) (* z s))
					  (aref1 a jj i) (- (* z c) (* y s)))))
		      
		      (setf (aref1 rv1 l) 0
			    (aref1 rv1 k) f
			    (aref1 w k) x))))
      (values a w v))))




(defun pythag (a b)
  (let ((absa (abs a))
	(absb (abs b)))
    (if (> absa absb)
	(* absa (sqrt (1+ (sqr (/ absb absa)))))
      (if (zerop absb)
	  0
	(* absb (sqrt (1+ (sqr (/ absa absb)))))))))



(defun sign (a b)
  (if (>= b 0)
      (abs a)
    (- (abs a))))


(defun imin (a b)
  (if (< a b) a b))

(defun fmax (a b)
  (if (> a b) a b))
				   

(defun sqr (a)
  (* a a))
				    
			      
(defun sv-backsub (u w v b)
  "Assuming U, W, and V are arrays returned by svd (A) for a square matrix, solve the equation Ax = b."
  (dbind (m n) (array-dimensions u)
    
    (let ((tmp (make-array n))
	  (x (make-array n :element-type 'float))
	  s)
      (loop for j from 1 to n
	  do (setf s 0)
	     (unless (zerop (aref1 w j))
	       (incf s (loop for i from 1 to m summing (* (aref1 b i) (aref1 u i j))))
	       (divf  s (aref1 w j)))
	     (setf (aref1 tmp j) s))
      
      (loop for j from 1 to n
	  finally (return x)
	  do (setf s 0)
	     (incf s (loop for jj from 1 to n summing (* (aref1 tmp jj) (aref1 v j jj))))
	     (setf (aref1 x j) s)))))




(defun svd-fit (x y)
  "svd-fit X Y.  Uses SVD to do a least squares fit.  X is the design matrix - an nxm matrix where n is the number of data points, m is the dimension of the feature space, and n > m.  Y is an n-vector.  Returns an m-vector theta such that X*theta is as close as possible to Y in the L2 norm."
  (dbind (ndata ma) (array-dimensions x)
    (assert (= (length y)  ndata) nil "~a must be a vector of length ~a" y ndata)

    (mvbind (u w v)
	(svd x)
      (let* ((wmax (reduce #'max w))
	     (thresh (* wmax *svdfit-tol*)))
	(loop for j from 1 to ma
	      ;;; my understanding is that w can't be negative here
	    do (assert (>= (aref1 w j) 0) () "Unexpected negative singular value in svdfit")
	       (when (< (aref1 w j) thresh)
		 (setf (aref1 w j) 0)))
	(values (sv-backsub u w v y) (svdvar v w))))))


(defun svdvar (v w &aux (ma (array-dimension v 0)))
  (let ((cvm (make-array (list ma ma) :element-type 'float))
	(wti (map 'vector #'(lambda (x) (if (zerop x) 0 (/ 1 (* x x)))) w)))
    (loop for i from 1 to ma
	do (loop for j from 1 to i
	       for sum = (loop for k from 1 to ma summing (* (aref1 v i k) (aref1 v j k) (aref1 wti k)))
	       do (setf (aref1 cvm j i) (setf (aref1 cvm i j) sum)))
	finally (return cvm))))
    
    
      
    
	  
	  
	  

	
			    
			    
      
      
	  
	  
      

				  

	
	



