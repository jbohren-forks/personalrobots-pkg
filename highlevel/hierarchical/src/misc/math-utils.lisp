(in-package utils)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; primes (generate nth prime in memoized way)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *primes*)
(defvar *sieve*)

(defun reset-primes ()
  (setf *primes* (make-array 1 :element-type 'fixnum :adjustable t :fill-pointer t :initial-element 2))
  (setf *sieve* (make-array 4 :element-type 'boolean :adjustable t :initial-element nil)))

(reset-primes)


(defun generate-next-prime ()
  (let ((p (slast *primes*)))
    (ensure-sieve-length (* 2 p))
    (let ((k (check-not-null (position nil *sieve* :start (1+ p)))))
      (vector-push-extend k *primes*))))

(defun nth-prime (n)
  (let ((k (length *primes*)))
    (while (<= k n)
      (generate-next-prime)
      (incf k))
    (aref *primes* n)))

(defun ensure-sieve-length (l)
  (adjust-array *sieve* l :initial-element nil)
  (map nil
    #'(lambda (p)
	(let ((q (* 2 p)))
	  (while (< q l)
	    (setf (aref *sieve* q) t)
	    (incf q p))))
    *primes*))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Low-dispersion sequences
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun vdc-generator (&optional (b 0) &aux (base (nth-prime b)))
  "Return a function of zero arguments that can be repeatedly called to generate the van der Corput sequence: 0, 1/2, 1/4, 3/4, 1/8, 5/8,... "
  (let ((a (make-array 1 :element-type 'fixnum :adjustable t :fill-pointer t :initial-element 0))
	(b (make-array 1 :element-type 'float :adjustable t :fill-pointer t :initial-element (/ 1.0 base))))
    #'(lambda ()
	(prog2 
	    (for-loop (i (length b) (length a))
	      (vector-push-extend (expt base (- (1+ i))) b))
	    (loop for x across a for y across b sum (* x y))
	  (advance-array a 1 base)))))


(defun advance-array (a n base)
  (let ((i (position-if #'(lambda (x) (< x (1- base))) a)))
    (cond
     (i (incf (aref a i))
	(fill a 0 :end i))
     (t (fill a 0)
	(vector-push-extend 1 a)
	(repeat (1- n) (vector-push-extend 0 a))))))




(defun n-dimensional-vdc-generator (n &optional (constant-space nil) (b 0) &aux (base (nth-prime b)))
  "Return a function of 0 arguments which, when called repeatedly, returns a dense sequence of vectors from [0,1)^N.  If CONSTANT-SPACE is true, then the return values of repeated calls will modify the same underlying vector, and if not, a fresh vector is returned each time."
  (let ((a (make-array n :element-type 'fixnum :adjustable t :fill-pointer t :initial-element 0))
	(b (make-array n :element-type 'float)))
    #'(lambda ()
	(prog2 
	    (update-array a b base)
	    (if constant-space b (copy-seq b))
	  (advance-array a n base)))))


(defun update-array (a b base)
  (fill b 0.0)
  (let ((k (length a))
	(n (length b))
	(i 0)
	(mult (/ 1.0 base)))
    (while (< i k)
      (dotimes (j n)
	(incf (aref b j) (* mult (aref a i)))
	(incf i))
      (divf mult base))))

	


	       



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code for treating 0,1,...,n-1 as elements of Z/nZ
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  
(defmacro def-mod-op (name (&rest args) &rest body)
  (with-gensyms (modulus)
    (condlet (((stringp (first body)) (docstring (first body)) (actual-body (rest body)))
	      (t (docstring "") (actual-body body)))
	     `(defun ,name (,modulus ,@args)
		,docstring
		(mod ,@actual-body ,modulus)))))

(defmacro def-simple-mod-op (name actual)
  (with-gensyms (args)
    `(def-mod-op ,name (&rest ,args) (apply (function ,actual) ,args))))

(def-simple-mod-op mod+ +)
(def-simple-mod-op mod- -)
(def-simple-mod-op mod* *)
(def-mod-op mod-expt (i j)
  (expt i j))
(def-mod-op mod-inc (i)
  (1+ i))
(def-mod-op mod-dec (i)
  (1- i))
(defun aref-mod (v i)
  "aref-mod VEC I.  Like aref except I is modded by length(V).  For now this is only a reader - cannot be setf-ed."
  (aref v (mod i (length v))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Constants
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *2pi* (* 2 pi))
