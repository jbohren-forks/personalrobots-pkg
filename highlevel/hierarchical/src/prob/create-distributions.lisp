;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; prob/create-distributions.lisp
;; Ways of creating various standard distributions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package prob)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; deterministic distribution
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-deterministic-dist (x)
  "make-deterministic-dist X.  Return a deterministic distribution over X (represented as an alist)."
  `((,x . 1.0)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; uniform distributions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <numbered-set-dist> (<prob-dist>)
  ((numbered-set :type [numbered-set]
		 :reader s
		 :initarg :set)
   (test :accessor test :initarg :test :initform #'equal)
   (no-duplicates :initform nil :initarg :no-duplicates :accessor no-duplicates))
  (:documentation "Represents a uniform distribution over a numbered set"))

(defun make-unif-dist-over-set (s &key (test #'equal))
  "make-unif-dist-over-set SET &key (TEST #'equal)  Return a uniform distribution over set, which must be of type [numbered-set].  The elements of SET are assumed unique according to equality test TEST."
  (check-type s [numbered-set])
  (assert (not (is-empty s)) () "Cannot make uniform distribution over empty set")
  (make-instance '<numbered-set-dist> :set s :test test :no-duplicates t))


(defmethod expectation ((d <numbered-set-dist>) rv &aux (n (size (s d))))
  (/ (sum-over (s d) #'(lambda (x) (evaluate-rv rv x))) n))

(defmethod prob ((d <numbered-set-dist>) x)
  (let ((s (s d))
	(test (test d))
	(p (/ 1 (size (s d)))))
    (if (no-duplicates d)
	(iwhen (member? x s) p) ;; todo signal if x not in set
      (sum-over s #'(lambda (y) (iwhen (funcall test x y) p))))))


(defmethod sample-space ((p <numbered-set-dist>))
  (s p))

(defmethod sample ((p <numbered-set-dist>))
  (sample-uniformly (s p)))

(defmethod sample-uniformly ((p <interval>))
  (let ((a (left-bound p))
	(l (interval-length p)))
    (check-type l real)
    (+ a (* l (random 1.0)))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; empirical distribution given a sequence of samples
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <empirical-dist> (<prob-dist>)
  ((samples :initarg :samples :accessor samples)
   (num-samples :accessor num-samples)
   (empirical-dist-table :initform nil :accessor empirical-dist-table)))

(defmethod initialize-instance :after ((d <empirical-dist>) &rest args &key samples hash-function test)
  (declare (ignore args))
  (setf (num-samples d) (verify-type (size samples) fixnum "an integer"))
  (when test
    (setf (empirical-dist-table d) (collapse-empirical-dist d :hash-function hash-function :test test))))

(defmethod probability (e (d <empirical-dist>))
  (aif (empirical-dist-table d)
      (probability e it)
    (/ (sum-over (samples d) #'(lambda (x) (indicator (member? x e))))
       (num-samples d))))

(defmethod expectation ((d <empirical-dist>) rv)
  (aif (empirical-dist-table d)
      (expectation it rv)
    (/ (sum-over (samples d) #'(lambda (x) (evaluate-rv rv x)))
       (num-samples d))))

(defmethod prob  ((d <empirical-dist>) x)
  (assert (empirical-dist-table d) nil "Prob method only works for empirical distributions that were given a test argument")
  (prob (empirical-dist-table d) x))

(defmethod sample ((d <empirical-dist>))
  (item (random (num-samples d)) (samples d)))
 
(defun empirical-dist (samples &key (test #'equal))
  "empirical-dist SAMPLES &key (TEST #'equal).  SAMPLES must be a finite set of samples.  It may contain duplicates (unlike <numbered-set-dist>).  Returns the corresponding empirical distribution (choose an index at random and return the ith sample)."

  (break "empirical-dist no longer guaranteed to work, due to changes in set equality checking")
  (assert (is-standard-equality-test test))
  (let ((h (make-hash-table :test test))
	(p (/ 1 (size samples))))
    (do-elements (x samples h)
      (setf (gethash x h) (+ p (or (gethash x h) 0))))))

(defun collapse-empirical-dist (d &key hash-function test)
  "collapse-empirical-dist DIST &key HASH-FUNCTION TEST
HASH-FUNCTION maps from sample points to integers.  Either it or test must be provided."

  (let ((table (cond
		(hash-function (make-hash-table* :hash-function hash-function :test #'eql))
		((is-standard-equality-test test) (make-hash-table :test test))))
	(p (/ 1.0 (num-samples d))))
    (do-elements (x (samples d) table)
      (setf (gethash* x table) (+ p (gethash* x table))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dist gotten by composing a random variable with a prob dist
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <rv-dist> (<prob-dist>)
  ((dist :type [prob-dist] :reader dist :initarg :dist)
   (rv :type [random-variable] :reader rv :initarg :rv))
  (:documentation "Distribution gotten by composing a random variable with a prob dist.  Can create using make-rv-dist

Sampling is efficient.  Computing probabilities by default is not so efficient since it just iterates over the sample space of dist and checks when rv equals the given element.  See <invertible-rv-dist> for an efficient version."))

(defun make-rv-dist (rv dist)
  "make-rv-dist RAND-VAR PROB-DIST.  Compose RAND-VAR with PROB-DIST to get a new PROB-DIST of type <rv-dist>."
  (make-instance '<rv-dist> :rv rv :dist dist))

(defmethod prob ((p <rv-dist>) x)
  (let ((total-prob 0))
    (do-sample-points (y prob (dist p) total-prob)
      (when (equal (evaluate-rv (rv p) y) x)
	(incf total-prob prob)))))


(defmethod sample ((p <rv-dist>))
  (evaluate-rv (rv p) (sample (dist p))))

(defmethod expectation ((p <rv-dist>) rv)
  (let ((rv2 (rv p)))
    (expectation (dist p) 
		 #'(lambda (x) 
		     (evaluate-rv rv (evaluate-rv rv2 x))))))

(defmethod sample-iterator ((p <rv-dist>))
  (let ((iter (sample-iterator (dist p)))
	(already-seen nil))
    (labels
	((new-it ()
	   (multiple-value-bind (item prob done?)
	       (funcall iter)
	     (declare (ignore prob))
	     (if done?
		 (values nil nil t)
	       (let ((new-item (evaluate-rv (rv p) item)))
		 (if (member new-item already-seen :test #'equal)
		     (funcall #'new-it)
		   (progn
		     (push new-item already-seen)
		     (values new-item (prob p new-item) nil))))))))
      #'new-it)))

(defmethod sample-space ((p <rv-dist>))
  
  (let ((iter (sample-iterator p))
	(l nil)
	(done? nil))
    (until done?
      (multiple-value-bind (item prob d2)
	  (funcall iter)
	(declare (ignore prob))
	(setf done? d2)
	(unless done? (push item l))))
    l))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; more efficient version of rv dists when the function is invertible
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defclass <invertible-rv-dist> (<rv-dist>)
  ((inverse :type function
	    :reader inv
	    :initarg :inverse))
  (:documentation "A more efficient subclass of rv-dist for rv's with known inverse. Create using make-inverse-rv-dist."))

(defun make-invertible-rv-dist (rv inverse dist)
  "make-invertible-rv-dist RV INVERSE DIST
INVERSE is a function of one argument that either returns the inverse value, or signals a 'rv-inverse-not-defined error."
  (make-instance '<invertible-rv-dist> :rv rv :inverse inverse :dist dist))

(defmethod prob ((p <invertible-rv-dist>) x)
  
  (handler-case
      (let ((y (funcall (inv p) x)))
	(let ((z (evaluate-rv (rv p) y)))
	  
	  ;; to be really sure things are working right, we verify that the 
	  ;; image of the inverse under rv is the same as the original
	  (assert (equal z x) ()
	    "~a not equal to image ~a of its inverse ~a."
	    x z y))
	
	(prob (dist p) y))
    
    
    ;; if inverse not defined, error
    (rv-inverse-not-defined ()
      (error 'element-not-in-domain :x x))))

(defmethod sample-iterator ((p <invertible-rv-dist>))
  ;; use invertibility of rv
  (let ((iter (sample-iterator (dist p))))
    (lambda ()
      (multiple-value-bind (x pr done)
	  (funcall iter)
	(if done
	    (values nil nil t)
	  (values (evaluate-rv (rv p) x) pr nil))))))


(defmethod sample-space ((p <invertible-rv-dist>))
  (let ((rv (rv p)))
    (if (functionp rv)
	(make-image-set (sample-space (dist p)) rv (inv p))
      (call-next-method))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; mixture models
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; class defs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <mixture-prob-dist> (<prob-dist>)
  ((weights :type vector
	    :reader mixture-weights
	    :initarg :weights)
   (dists :type vector
	  :reader dists
	  :reader mixture-components
	  :initarg :dists))
  (:documentation "Subclass of <prob-dist> for mixture distributions.  Create using make-mixture-dist."))

(defclass <mixture-cond-dist> (<cond-prob-dist>)
  ((weights :type simple-vector :reader mixture-weights :initarg :weights)
   (cond-dists :type simple-vector :reader cond-dists :reader mixture-components :initarg :cond-dists)))


(defun make-mixture-dist (weights dists)
  "make-mixture-dist WEIGHT-VECTOR DIST-VECTOR.  Create a mixture distribution over the distributions in DIST-VECTOR using weights in WEIGHT-VECTOR.  Doesn't 'expand out' the new distribution, which means the operations on the returned distribution take a bit longer."
  (let ((n (length weights)))
    (assert (= n (length dists)) () "Weights ~a and dists ~a must have same length" weights dists)
    (assert (is-valid-prob-dist weights)))
  (make-instance '<mixture-prob-dist> :weights weights :dists dists))
    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; methods
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;; 


(defmethod prob ((p <mixture-prob-dist>) y)
  (loop
      for w across (mixture-weights p)
      for d across (dists p)
      sum (* w (prob d y))))

(defmethod sample ((p <mixture-prob-dist>))
  (sample (aref (dists p) (sample (mixture-weights p)))))

(defmethod expectation ((p <mixture-prob-dist>) rv)
  (loop
      for w across (mixture-weights p)
      for d across (dists p)
      sum (* w (expectation d rv))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Mixture cond dist
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod cond-dist ((d <mixture-cond-dist>) x)
  (make-instance '<mixture-prob-dist>
    :weights (mixture-weights d) :dists (map 'vector #'(lambda (cd) (cond-dist cd x)) (mixture-components d))))

(defmethod transform-dist ((d <mixture-prob-dist>) cd)
  (make-instance '<mixture-prob-dist>
    :weights (mixture-weights d) :dists (map 'vector #'(lambda (p) (transform-dist p cd)) (dists d))))

(defmethod transform-dist (d (cd <mixture-cond-dist>))
  (make-instance '<mixture-prob-dist>
    :weights (mixture-weights cd) :dists (map 'vector #'(lambda (c) (transform-dist d c)) (cond-dists cd))))

(defmethod transform-dist ((d <mixture-prob-dist>) (cd <mixture-cond-dist>))
  (let ((w (mixture-weights d))
	(cw (mixture-weights cd))
	(dists (dists d))
	(cdists (cond-dists cd)))
    (let ((comps (make-instance '<prod-set> :sets (list (length w) (length cw))
				:inst-acc (make-list-accessors 2) :iterate-quickly t)))
      (make-instance '<mixture-prob-dist>
	:weights (mapset 'vector #'(lambda (c) (* (aref w (first c)) (aref cw (second c)))) comps)
	:dists (mapset 'vector #'(lambda (c) (transform-dist (aref dists (first c)) (aref cdists (second c)))) comps)))))

(defmethod condition-on-dist ((d <mixture-prob-dist>) cd y)
  (let* ((w (map 'vector #'(lambda (comp w) (* w (prob (transform-dist comp cd) y)))
		 (mixture-components d) (mixture-weights d)))
	 (total (reduce #'+ w)))
    (dotimes (i (length w))
      (divf (aref w i) total))
    
    (make-instance '<mixture-prob-dist>
      :weights w
      :dists (map 'vector #'(lambda (comp) (condition-on-dist comp cd y)) (mixture-components d)))))

(defmethod condition-on-dist (d (cd <mixture-cond-dist>) y)
  (let* ((w (map 'vector #'(lambda (comp w) (* w (prob (transform-dist d comp) y)))
		 (mixture-components d) (mixture-weights d)))
	 (total (reduce #'+ w)))
    (dotimes (i (length w))
      (divf (aref w i) total))
    
    (make-instance '<mixture-prob-dist>
      :weights w 
      :dists (map 'vector #'(lambda (c) (condition-on-dist d c y)) (mixture-components d)))))

(defmethod condition-on-dist ((d <mixture-prob-dist>) (cd <mixture-cond-dist>) y)
  (let ((w (mixture-weights d))
	(cw (mixture-weights cd))
	(dists (mixture-components d))
	(cdists (mixture-components cd)))
    (let* ((comps (make-instance '<prod-set> :sets (list (length w) (length cw))
				 :inst-acc (make-list-accessors 2) :iterate-quickly t))
	   (weights (mapset 'vector
		      #'(lambda (c)
			  (dbind (i j) c
			    (* (aref w i) (aref cw j)
			       (prob (transform-dist (aref dists i) (aref cdists j)) y))))
		      comps))
	   (total (reduce #'+ weights)))
      (dotimes (i (length weights))
	(divf (aref weights i) total))
      (make-instance '<mixture-prob-dist>
	:weights weights
	:dists (mapset 'vector
		 #'(lambda (c)
		     (dbind (i j) c
		       (condition-on-dist (aref dists i) (aref cdists j) y)))
		 comps)))))
		 



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sample with replacement
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <sampling-with-replacement> (<prob-dist>)
  ((dist :type [prob-dist] :reader dist :initarg :dist
	 :documentation "The underlying distribution")
   (n :type fixnum :reader n :initarg :n
      :documentation "Number of samples."))
  (:documentation "A probability distribution that consists of sampling with replacement from some distribution DIST N times.  Results are collected into a vector of length n.

Initargs
:dist
:n"))

(defmethod prob ((p <sampling-with-replacement>) x)
  (let ((n (n p))
	(dist (dist p)))
    (if (and (arrayp x) (= (length x) n))
	(let ((prob 1))
	  (map nil (lambda (y) (multf prob (prob dist y))) x)
	  prob)
      (signal-element-not-in-domain x))))

(defmethod sample ((p <sampling-with-replacement>))
  (with-slots (n dist) p
    (mapset 'vector (lambda (i) (declare (ignore i)) (sample dist)) n)))

;; expectation not implemented for now

(defmethod sample-space ((p <sampling-with-replacement>))
  (apply #'direct-product 'vector (loop repeat (n p) collect (sample-space (dist p)))))
		 

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sample without replacement
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <sampling-without-replacement> (<prob-dist>)
  ((dist :type [prob-dist] :reader dist :initarg :dist
	 :documentation "The underlying distribution")
   (n :type fixnum :reader n :initarg :n
      :documentation "Number of samples")
   (element-type :type symbol :accessor element-type :initarg :element-type :initform 'vector))
  (:documentation "A probability distribution that samples with replacement from DIST N times.  Results are collected into a vector or list of length N depending on element-type

Initargs
:dist
:n
:element-type - defaults to 'vector

Note that sampling is done just by repeatedly trying to sample each element until we get one that's different.  So if the DIST assigns most or all of its probability to some M < N elements, this could go into a long or infinite loop."))

(defmethod initialize-instance :after ((p <sampling-without-replacement>) &rest args)
  ;; sanity check
  (declare (ignore args))
  (let* ((s (sample-space (dist p)))
	 (n (n p))
	 (m (size s)))
    (assert (<= n m) ()
      "Cannot sample without replacement ~a times from set ~a of size ~a"
      n s m)))

(defmethod prob ((p <sampling-without-replacement>) x)
  (let ((dist (dist p))
	(prob 1)
	(tot-prob 1))
    (map nil
      #'(lambda (y)
	  (let ((pr (prob dist y)))
	    (multf prob (/ pr tot-prob))
	    (decf tot-prob pr)))
      x)
    prob))

(defmethod sample ((p <sampling-without-replacement>))
  (loop
      with n = (n p)
      with dist = (dist p)
      with elt-type = (element-type p)
      with a = (ecase elt-type (vector (make-array n)) (list (make-list n)))
	       
      for i below n
      for z = (loop
		  for y = (sample dist)
		  while (find y a :end i :test #'equalp) ;; TODO not right
		  finally (return y))
	
      do (assert z () "Can't sample without replacement from dist ~a which has () as a sample point" dist)
	 (setf (elt a i) z)
	 
      finally (return a)))


(defmethod sample-space ((p <sampling-without-replacement>))
  ;; this is an overestimate of the set, but that shouldn't cause a problem
  (apply #'direct-product 'vector (loop repeat (n p) collect (sample-space (dist p)))))


;; expectation not implemented
      



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; multinomial over a set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <multinomial-dist> (<prob-dist>)
  ((prob-vec :type (vector float) :reader prob-vec :initarg :prob-vec)
   (domain :type [numbered-set] :reader domain :initarg :domain)))

								   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod prob ((p <multinomial-dist>) x)
  (aref (prob-vec p) (item-number x (domain p))))

(defmethod sample ((p <multinomial-dist>))
  (item (sample (prob-vec p)) (domain p)))

(defmethod expectation ((p <multinomial-dist>) rv)
  (let ((dom (domain p)))
    (expectation (prob-vec p) #'(lambda (i) (funcall rv (item i dom))))))

(defun sample-multinomial (domain &rest probs)
  (let ((x (random 1.0))
	(cumsum 0))
    (loop
	for p in probs
	for i from 0
	do (incf cumsum p)
	when (> cumsum x)
	do (return (item i domain)))))

(defmethod sample-space ((p <multinomial-dist>))
  (domain p))

    

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; creation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
 

(defun make-multinomial-dist (v &optional (s (length v)))
  "make-multinomial-dist V &optional NUMBERED-SET
Make a multinomial distribution with probabilities as given by the vector.  V is a vector of nonnegative reals, and may be unnormalized.  If SET is not provided, it defaults to 0,1,..., N-1 where N is the length of the vector.  See also make-sorted-multinomial-dist."
  (let ((z (reduce #'+ v)))
    (make-instance '<multinomial-dist>
      :prob-vec 
      
      (map 'vector 
		  #'(lambda (x) 
		      (assert (>= x 0) nil "Probability vector must be nonnegative.")
		      (/ x z))
		  v)
      :domain s)))

(defun make-boltzmann-dist (v temp &optional (domain (length v)))
  "make-boltzmann-dist VEC TEMPERATURE &optional DOMAIN.  Create boltzmann distribution from the vector of negative energies.  If DOMAIN is not provided, it defaults to 0,1,...,N-1 where N is the length of VEC"
  (let ((l (length v)))
    (assert (> l 0) (v)
      "Attempted to create Boltzmann distribution from empty vector.")
	
    (let ((m (reduce #'max v)))
      (make-multinomial-dist
       (cond
	((eql temp 'infty) (make-array l :element-type 'fixnum :initial-element 1))
	((> temp 0) (map 'vector #'(lambda (x) (exp (/ (- x m) temp))) v))
	(t ;; temperature equals 0
	 (map 'vector #'(lambda (x) (indicator (eql x m))) v)))
       domain))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; tabular distribution of random variables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <tabular-dist> (<prob-dist>)
  ((table :accessor table :initarg table)
   (domains :accessor domains)
   (domain-sizes :accessor domain-sizes)
   (acc :accessor acc)
   (rank :accessor rank)
   (iterate-quickly :accessor iterate-quickly :initform nil :initarg :iterate-quickly))  
  (:documentation "<tabular-dist> (<prob-dist>)
Stores joint instantiations of a set of random variables in a table.  The initial creation iterates over the sample space of the original distribution, but after that the operations should be fast (limited mainly by the speed of hashing the individual variable values).

Initargs

Among the following, either provide dist and vars, or table and (optionally) domains

:dist - a [prob-dist]
:vars - a list of [random-variable] objects
:table - a multidimensional array representing the probability table for joint instantiations
:domains - a vector of sets representing the domains of each component.  Defaults to being 0:n-1 for component i where n is the size of the ith dimension of table.

Optional arguments
:acc - Optional.  For now, must be either the symbol 'list (which is the default), in which case joint instantiations are represented as lists, or a list of symbols, in which case they are represented as association lists over those symbols.
:iterate-quickly - defaults to nil.  If this is true, iteration over the sample space is done more memory-efficiently by reusing the same instantiation.  This works so long as the iteration does not modify or save the instantiation.
"))

(defmethod initialize-instance :after ((d <tabular-dist>) &rest args &key (acc 'list) dist (vars nil v-supp) table domains )
  (assert (xor (and dist v-supp (not domains)) table) nil "Illegal arguments to initialize instance for <tabular dist> : ~a" args)
  (let ((n (if v-supp (length vars) (array-rank table))))
    (setf (rank d) n
	  (acc d) (etypecase acc
		    ((member list) (inst-vars:make-list-accessors n))
		    (list (assert (= (length acc) n)) (inst-vars:make-alist-accessors acc #'equal)))
	  (domains d) (if v-supp
			  (mapset 'vector #'(lambda (i) (declare (ignore i)) (make-instance '<indexed-set> :test #'equalp)) n)
			(or domains (coerce (array-dimensions table) 'vector)))
	  
	  (table d)
	  (or table
	      (let ((a (make-inf-array :rank n :default-val 0.0)))
		(do-sample-points (u p dist)
		  (let* ((vals (mapcar #'(lambda (v) (evaluate-rv v u)) vars))
			 (nums (mapset 'list #'(lambda (v i) (tabular-dist-get-num d v i)) vals n)))
		    (incf (apply #'inf-aref a nums) p)))
		(inf-array-get-table a)))
	  (domain-sizes d) (map 'list #'size (domains d)))))


(defmethod sample-space ((d <tabular-dist>))
  ;; note sample iterator below relies on this using a particular iteration order
  (make-instance 'prod-set:<prod-set>
    :sets (domains d) :inst-acc (acc d) :iterate-quickly (iterate-quickly d)))

(defun tabular-dist-get-num (d item i)
  (let ((tab (aref (domains d) i)))
    (addf tab item)
    (item-number item tab)))

(defmethod prob ((d <tabular-dist>) x &aux (a (acc d)))
  (apply #'aref (table d) (mapset 'list #'(lambda (i) (tabular-dist-get-num d (inst-vars:get-var-val a x i) i)) (rank d))))

(defmethod sample-iterator ((d <tabular-dist>))
  (with-slots (table domains acc) d
    (let ((sample-point-iterator (iterator (sample-space d))) 
	  (index-iterator (iterator 
			   (make-instance 'prod-set:<prod-set>
			     :sets (domain-sizes d)
			     :iterate-quickly t
			     :inst-acc (inst-vars:make-list-accessors (rank d))))))
      #'(lambda ()
	  (mvbind (x done?) (funcall sample-point-iterator)
	    (if done?
		(values -42 -42 t)
	      (let ((ind (funcall index-iterator)))
		;(format t "~&X = ~a, Ind = ~a~&" x ind)
		(values x (apply #'aref table ind) nil))))))))
	      
    

(defmethod sample ((d <tabular-dist>))
  (let ((r (random 1.0))
	(tot 0.0)
	(a (table d))
	(coords (apply #'direct-product 'list (map 'list #'size (domains d)))))
    (do-elements (c coords nil i)
      (when (> (incf tot (apply #'aref a c)) r)
	(return-from sample (item i (sample-space d)))))))

  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; General conditional distributions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <conditional-distribution> (<prob-dist>)
  ((dist :accessor dist :initarg :dist)
   (event :accessor event :initarg :event)))

(defmethod sample ((p <conditional-distribution>))
  ;; By default, just sample from the original distribution until the event holds
  (with-slots (dist event) p
    (loop
      (let ((x (sample dist)))
	(when (funcall event x)
	  (return x))))))