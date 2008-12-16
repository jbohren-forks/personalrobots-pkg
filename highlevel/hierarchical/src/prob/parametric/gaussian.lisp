;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; gaussians, truncated, linear guassians
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package prob)

(defparameter *gaussian-tol* .0001)


(defclass <gaussian> (<continuous-prob-dist>)
  ((mean :type (vector float) :accessor gaussian-mean)
   (covariance :type (array float 2) :accessor cov :reader covariance)
   (precision :type array :accessor precision)
   (chol :accessor chol :initarg :chol) ;; matrix to transform iid standardized n-dim gaussian into this one
   (dim :type fixnum :accessor dim)
   (norm-const :accessor norm-const)
   (use-vector :initform t :accessor use-vector))
  (:documentation "<gaussian> (<continuous-prob-dist>)

Initargs
:mean - of type (vector float) or a float (in the one-dimensional case)
:cov - of type (array float 2) or a float (in the one-dimensional case)
:chol - the cholesky decomp. of cov, or ':unknown.  May be omitted, in which case it will be computed.

The distribution is over vectors, except in the case where the mean and cov are provided as individual numbers, in which case the distribution is also over numbers."))

(defmethod initialize-instance :after ((d <gaussian>) &rest args &key mean cov)
  (declare (ignore args))
  (when (typep mean 'number)
    (setf (use-vector d) nil)
    (setf mean (make-array 1 :element-type 'float :initial-contents `(,(float mean))))
    (check-type cov number)
    (setf cov (make-array '(1 1) :element-type 'float :initial-contents `((,(float cov))))))
  (check-type mean (vector float))
  (check-type cov (array float 2))
  (setf (gaussian-mean d) mean
	(cov d) cov
	(precision d) (inverse cov))

  (unless (slot-boundp d 'chol)
    (setf (chol d) (cholesky cov)))
  (let* ((chol (chol d))
	 (dim (length mean))
	 (det (if (arrayp chol)
		  (expt (reduce-set #'* dim :key #'(lambda (i) (aref chol i i))) 2)
		(determinant cov))))
    (setf (norm-const d) (* (expt (* 2 pi) (- (/ dim 2))) (expt det -1/2)))
    (assert (equal (array-dimensions cov) (list dim dim)) nil
      "Covariance ~a was not a ~a by ~:*~a matrix" cov dim)
    (setf (dim d) dim)))


(defmethod sample ((d <gaussian>))
  (with-slots (dim use-vector mean covariance chol) d
    (if use-vector
	(let ((v (a* chol (mapset 'vector #'(lambda (i) (declare (ignore i)) (box-muller)) dim))))
	  (dotimes (i dim v)
	    (incf (aref v i) (aref mean i))))
      (+ (aref mean 0) (* (box-muller) (sqrt (aref covariance 0 0)))))))

(defmethod mean ((d <gaussian>))
  (if (use-vector d)
      (gaussian-mean d)
    (aref (gaussian-mean d) 0)))
  

(defmethod prob ((d <gaussian>) x)
  (unless (use-vector d)
    (setf x (make-array 1 :initial-element x)))
  (let ((diff (a- x (gaussian-mean d))))
    (a* (norm-const d)
	(exp (* -1/2 (a* diff (mv* (precision d) diff)))))))
	       
	       

(defmethod probability ((d <gaussian>) (s <interval>))
  (assert (= (dim d) 1))
  (let* ((mean (aref (gaussian-mean d) 0))
	 (std (sqrt (aref (cov d) 0 0)))
	 (a (my/ (my- (left-bound s) mean) std))
	 (b (my/ (my- (right-bound s) mean) std)))
    (- (lookup-gaussian-cdf b) (lookup-gaussian-cdf a))))

(defvar *gaussian-cdf-table* nil) 

(defun lookup-gaussian-cdf (x)
  ;; Read in the cdf table if necessary
  (orf *gaussian-cdf-table*
       (with-open-file (f "prob/parametric/gaussian-cdf-table" :direction :input)
	 (read f)))
  
  (case x
    (-infty 0.0)
    (infty 1.0)
    (otherwise
     (let ((n (length *gaussian-cdf-table*))
	   (i (bsearch *gaussian-cdf-table* x)))
    
       ;; If x is within the bounds of the table use that.  Otherwise use an e^-x approximation (is this even right?)
       (cond
	((= i 0) (/ (exp (- x (aref *gaussian-cdf-table* 0))) n))
	((= i n) (- 1 (/ (exp (- (slast *gaussian-cdf-table*) x)) n)))
	(t (float (/ i n))))))))
     

(defun box-muller ()
  "polar box-muller sampling procedure for a single N(0,1) variable"
  (let (u v s)
    (repeat-until 
     (and (< 0 s) (< s 1))
     (setf u (1- (* 2 (random 1.0)))
	   v (1- (* 2 (random 1.0)))
	   s (+ (* u u) (* v v))))
    (* u (sqrt (/ (* -2 (log s)) s)))))
		  

(defconstant root-2pi (sqrt (* 2 pi)))

(defun gaussian-conditional-expectation (mean std c d)
  "Expectation of Gaussian with mean MEAN and std STD, conditioned on being in the interval [c,d]."
  (assert (my<= c d) nil "[~a, ~a] is not a valid interval" c d)
  (if (or (infinite mean) (< std *gaussian-tol*))
      mean
    (let ((c (my- c mean))
	  (d (my- d mean)))
      (flet ((nexp1 (x)
	       (myexp (my- (my/ (my* x x) (my* 2 std std))))))
	(my+ mean
	     (let ((mult 1)
		   (n (my* root-2pi (my- (lookup-gaussian-cdf (my/ d std)) (lookup-gaussian-cdf (my/ c std))))))
	       (when (my< c 0)
		 (multf c -1))
	       (when (my< d 0)
		 (multf d -1))
	       (when (my< d c)
		 (rotatef c d)
		 (setf mult -1))
	       (if (my< n *gaussian-tol*)
		   c
		 (my/ (my* mult std (my- (nexp1 c) (nexp1 d))) n))))))))

(defmethod print-object ((d <gaussian>) str)
  (print-unreadable-object (d str :type t :identity nil)
    (format str "Mean: ~a  Cov: ~a" (gaussian-mean d) (cov d))))

(defmethod print-prob-dist (str (d <gaussian>))
  (format str "~a" d))
			    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Truncated Gaussian
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <truncated-gaussian> (<continuous-prob-dist>)
  ((left-end-point :initarg :a :accessor left-end-point)
   (right-end-point :initarg :b :accessor right-end-point)
   (mean :initform nil :initarg :mean :accessor gaussian-mean)
   (original-mean :accessor mu)
   (std :initarg :std :accessor std))
  (:documentation "Class <truncated-gaussian>

Initargs
:a, :b - left and right end points
:mean - mean of the truncated gaussian
:std - standard deviation of the original gaussian (not the truncated one)."))

(defmethod initialize-instance :after ((g <truncated-gaussian>) &rest args)
  (declare (ignore args))
  (with-slots (left-end-point right-end-point std mean) g
    (flet ((new-mean (m)
	     (gaussian-conditional-expectation m std left-end-point right-end-point)))
      (assert (between mean (new-mean left-end-point) (new-mean right-end-point)) nil
	"Currently truncated gaussian requires the mode of the gaussian to be within the bounds [~a, ~a].  For that, the mean ~a has to be in [~a, ~a]."
	left-end-point right-end-point mean (new-mean left-end-point) (new-mean right-end-point))
    
    ;; Use this iterative procedure to figure out what the mean of the original gaussian
    ;; should be.  Seems to converge in examples.
    (let ((m mean))
      (loop
	(let* ((m2 (new-mean m))
	       (d (if (eql m2 mean) 0 (my- m2 mean))))
	  (when (< (abs d) *gaussian-tol*)
	    (setf (mu g) m)
	    (return nil))
	  (decf m d)))))))
	  

(defmethod probability ((g <truncated-gaussian>) (s <interval>))
  (with-slots (original-mean std left-end-point right-end-point) g
    (let ((a (my- (mymax left-end-point (left-bound s)) original-mean))
	  (b (my- (mymin right-end-point (right-bound s)) original-mean)))
      (cond ((my>= a b) 0.0)
	    ((my<= std (my* *gaussian-tol* (my- b a)))
	     (indicator (between original-mean a a)))
	    (t (my/ (my- (lookup-gaussian-cdf (my/ b std)) (lookup-gaussian-cdf (my/ a std)))
		    (my- (lookup-gaussian-cdf (my/ (my- right-end-point original-mean) std))
			 (lookup-gaussian-cdf (my/ (my- left-end-point original-mean) std)))))))))


(defmethod expectation ((g <truncated-gaussian>) rv)
  (assert (eq rv #'identity))
  (gaussian-mean g))

(defmethod conditional-expectation ((g <truncated-gaussian>) (s <interval>) &key (rv #'identity fn-supp))
  (declare (ignore rv))
  (assert (not fn-supp))
  (with-slots (original-mean std) g
    (let ((c (mymax (left-end-point g) (left-bound s)))
	  (d (mymin (right-end-point g) (right-bound s))))
      (assert (my<= c d) nil "Conditioning ~a on zero-probability event [~a, ~a]" g c d)
      (gaussian-conditional-expectation original-mean std c d))))

    

(defmethod print-object ((d <truncated-gaussian>) str)
  (print-unreadable-object (d str :type t :identity nil)
    (format str "Mean: ~a. Std: ~a.  Bounds: [~a, ~a]"
	    (gaussian-mean d) (std d) (left-end-point d) (right-end-point d))))
  
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Linear Gaussian
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <linear-gaussian> (<cond-prob-dist>)
  ((trans :accessor trans :initarg :trans)
   (bias :accessor bias :initarg :bias)
   (cov :accessor cov :initarg :cov)
   (chol :accessor chol)
   (use-vector :accessor use-vector :initform t))
  (:documentation "Class <linear-gaussian> (<cond-prob-dist>)

Initargs
:trans - an mxn matrix where n is the dimension of the input vector and m of the output
:bias - an m-vector 
:cov - an mxm vector


as with <gaussian>, both bias and cov can also be floats, in which case the distribution is over numbers instead of vectors."))

(defmethod initialize-instance :after ((d <linear-gaussian>) &rest args &key cov trans bias)
  (declare (ignore args))
  (when (numberp cov)
    (setf (trans d) (make-array '(1 1))
	  (bias d) `#(,bias)
	  (cov d) (make-array '(1 1))
	  (use-vector d) nil
	  (aref (trans d) 0 0) trans
	  (aref (cov d) 0 0) cov
	  (use-vector d) nil))
    
  (setf (chol d) (cholesky cov)))

(defmethod cond-dist ((d <linear-gaussian>) x)
  (with-slots (bias cov chol trans use-vector) d
    (if use-vector
	(make-instance '<gaussian>
	  :mean (a+ (a* trans x) bias) :cov cov :chol chol)
      (make-instance '<gaussian>
	:mean (+ (* (aref trans 0 0) x) (aref bias 0)) :cov (aref cov 0 0) :chol chol))))

(defmethod transform-dist ((p <gaussian>) (d <linear-gaussian>))
  (with-slots (trans bias cov) d
    (let ((m (a+ bias (a* trans (gaussian-mean p))))
	  (c (a+ cov (a* (a* trans (cov p)) (transpose trans)))))
      (if (use-vector p)
	  (make-instance '<gaussian> :mean m :cov c :chol ':unknown)
	(make-instance '<gaussian> :mean (aref m 0) :cov (aref c 0 0) :chol ':unknown)))))

(defmethod condition-on-dist ((p <gaussian>) (d <linear-gaussian>) y)
  (unless (use-vector p)
    (setf y (make-array 1 :initial-element y)))
  (with-slots (trans bias cov) d
    (let* ((sigma (cov p))
	   (mu (gaussian-mean p))
	   (w (a- y (a+ bias (a* trans mu))))
	   (tr (transpose trans))
	   (kxy (a* sigma tr))
	   (iyy (inverse (a+ cov (a* trans kxy)))))
      (make-instance '<gaussian>
	:mean (a+ mu (a* kxy (a* iyy w)))
	:cov (a- sigma (a* kxy (a* iyy (transpose kxy))))))))
	   
		
      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Mixture of gaussians
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun mixture-of-gaussians (weights means covariances)
  "mixture-of-gaussians WEIGHTS MEANS COVARIANCES"
  (make-mixture-dist 
   (coerce weights 'vector) 
   (map 'vector #'(lambda (m c) (make-instance '<gaussian> :mean m :cov c)) means covariances)))

(defun mixture-of-linear-gaussians (weights trans bias cov)
  "mixture-of-gaussians WEIGHTS TRANS MEANS COVARIANCES"
  (make-instance '<mixture-cond-dist>
    :weights weights
    :cond-dists (map 'vector 
		  #'(lambda (tr b c) 
		      (make-instance '<linear-gaussian>
			:trans tr :bias b :cov c))
		  trans bias cov)))
		  
			