(defpackage probability
  (:documentation "A package for probability distributions and operations on them.  

Main types
----------
[prob-dist] - basic type for probability distributions.  
<prob-dist> - class that can be extended to create new probability distributions.
[cond-prob-dist] - basic type for conditional distributions
[random-variable] - basic type for random variables

Common distributions
----------------------
make-multinomial-dist
make-boltzmann-dist
make-unif-dist-over-set
<empirical-dist>
make-deterministic-dist
standardize-alist-prob-dist
make-rv-dist 
make-invertible-rv-dist
make-mixture-dist
<rv-dist>
<sampling-without-replacement>
<sampling-with-replacement>
<tabular-dist>
<conditional-distribution>

Parametric families
-------------------
<gaussian>
<truncated-gaussian>
<linear-gaussian>
mixture-of-gaussians
mixture-of-linear-gaussians

Basic operations on a probability distribution (which may not be implemented by all of them)
--------------------------------------------------------------------------------------------
sample 
probability
cdf
prob 
expectation 
mean
covariance
sample-iterator
sample-space 
do-sample-points
is-valid-prob-dist
map-prob-dist
updatef
clone-dist
marginalize
condition-on-dist
transform-dist
chain-dists
total-variation-distance

Sampling
--------
sample-uniformly
rejection-sampling


Information theory
------------------
entropy
conditional-entropy
mutual-information
normalized-conditional-mutual-information

Operations on a conditional probability distribution
----------------------------------------------------
cond-dist 
cond-prob 
cond-sample 
cond-exp 
conditional-expectation

Operations on mixtures
----------------------
mixture-weights
mixture-components

Operations on a random variable
-------------------------------
evaluate-rv 


Conditions
----------
invalid-distribution - *may* be signalled by any of the operations when given an invalid distribution (e.g. doesn't sum to 1)
element-not-in-domain - signalled when trying to evaluate probability of an element that isn't in the domain of the distribution
cond-dist-not-defined - signalled when trying to condition on an element for which the conditional dist is not defined.
rv-inverse-not-defined - signalled by inverse functions of random variables for undefined values

Constants
---------
*prob-dist-sum-tol* - valid distributions must sum to within this of 1.0

Debugging
---------
pprint-prob-dist
print-prob-dist
*print-prob-precision*

")
  
  (:nicknames prob)
  (:export <prob-dist>
	   [prob-dist]
	   [cond-prob-dist]
	   <cond-prob-dist>
	   [random-variable]
	   <rv-dist>
	   <sampling-without-replacement>
	   <sampling-with-replacement>
	   <tabular-dist>
	   empirical-dist
	   <empirical-dist>
	   <conditional-distribution>
	   
	   make-multinomial-dist
	   make-boltzmann-dist
	   make-deterministic-dist
	   make-unif-dist-over-set
	   empirical-dist
	   sample-multinomial
	   make-rv-dist
	   make-invertible-rv-dist
	   make-mixture-dist
	   standardize-alist-prob-dist

	   
	   <gaussian>
	   <truncated-gaussian>
	   <linear-gaussian>
	   mixture-of-gaussians
	   mixture-of-linear-gaussians

	   sample
	   prob
	   probability
	   cdf
	   is-valid-prob-dist
	   map-prob-dist
	   updatef
	   clone-dist
	   marginalize
	   condition-on-dist
	   transform-dist
	   chain-dists
	   total-variation-distance

	   entropy
	   conditional-entropy
	   mutual-information
	   normalized-conditional-mutual-information

	   cond-prob
	   cond-dist
	   cond-sample
	   cond-exp
	   conditional-expectation
	   
	   mixture-weights
	   mixture-components

	   sample-uniformly
	   rejection-sampling

	   evaluate-rv
	   expectation
	   mean
	   covariance
	   sample-iterator
	   sample-space
	   do-sample-points
	   invalid-distribution
	   cond-dist-not-defined
	   rv-inverse-not-defined
	   element-not-in-domain
	   
	   pprint-prob-dist
	   print-prob-dist)
  (:use common-lisp
	utils
	lin-alg
	inst-vars
	set))


(in-package probability)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <prob-dist> () ())
(defclass <cond-prob-dist> () ())  
(defclass <continuous-prob-dist> () ())


(defun is-assoc-list (x)
  (and (listp x)
       (every #'consp x)))

(deftype [prob-dist] ()
  "Type for probability distributions.  Can be either

1. A vector of numbers, representing a multinomial distribution.  E.g., #(.2 .5 .3) assigns probability .2 to 0, .5 to 1, and .3 to 2
2. An association list with items of the form (ELT . PROB).  The ELT must all be different (use standardize-alist-prob-dist otherwise)
3. Object of type <prob-dist>"
  `(or <prob-dist> vector (satisfies is-assoc-list) ))


(deftype [cond-prob-dist] ()
  "Type for conditional probability distributions of the form P(Y|X).  Can be either

1.  A function of one argument,  (the values of the variable being conditioned on), which returns a [prob-dist].
2.  Object of type <cond-prob-dist>

If you want to condition on multiple variables, you need to combine them into a single object, such as a list.  Make sure that the combined object has the right behaviour under equalp.

Conditional distributions should never modify the object being conditioned on."
  `(or <cond-prob-dist> function))




(deftype [random-variable] ()
  "Type for random variables, in the measure-theoretic sense of a function defined on a sample space.  The point of having this type is to allow structured representations which allow the basic operations to be done more efficiently.  Right now, can only be a real-valued function."
  'function)


(define-condition invalid-distribution () 
  ((p :initarg :p :reader p)))

(define-condition element-not-in-domain ()
  ((x :initarg :x :reader x)))

(defun signal-element-not-in-domain (x)
  "signal-element-not-in-domain X.  Signal, then return 0."
  (signal 'element-not-in-domain :x x)
  0.0)
	  

(define-condition cond-dist-not-defined ()
  ((dist :initarg :dist :reader dist)
   (x :initarg :x :reader x)))

(define-condition rv-inverse-not-defined ()
  ((y :initarg :y :reader y)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; constants
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *prob-dist-sum-tol* .0001)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations for probability distributions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; See the default method for this below
(defgeneric sample (p)
  (:documentation "sample PROB-DIST.  Return a randomly chosen element sampled according to PROB-DIST.  Can also use a function in place of PROB-DIST, in which case the function is just called and its argument returned.")
  (:method ((p function))
	   (funcall p)))

(defgeneric prob (p x)
  (:documentation "prob PROB-DIST ELT.  Evaluate the probability density of PROB-DIST at ELT.  If the element is not in the set, might throw an element-not-in-domain condition, and then return 0 (i.e. you only need to worry about handling the condition if you want to override this behaviour). See also the more general function probability.

Note that, strictly speaking, density is not an intrinsic property of a probability distribution, and depends on the base measure.  But for now, we assume that every distribution has a 'default' base measure w.r.t. which this function is defined: counting measure for discrete distributions and Lebesgue measure for absolutely continuous distributions in R^n.  
"))


(defgeneric probability (p s)
  (:documentation "probability PROB-DIST EVENT.  This function will typically be implemented only for certain types of events, for a given prob-dist type, though for finite sets, there's always a method that just calls prob.")
  (:method (p s)
	   (if (numberp (size s))
	       (sum-over s #'(lambda (x) (prob p x)))
	     (call-next-method))))

(defun cdf (p x &optional (right-open t))
  "cdf P X &optional (RIGHT-OPEN t) A special case of the probability generic function that returns the probability of the interval between -infty and X."
  (probability p (make-instance '<interval> :a '-infty :left-open t :b x :right-open right-open)))

;; See the default method for this below
(defgeneric expectation (p rv)
  (:documentation "expectation PROB-DIST &key (RANDOM-VAR #'identity).  Compute the expectation of RANDOM-VAR."))

(defgeneric mean (p)
  (:documentation "mean P.  Shorthand for (expectation P #'identity).")
  (:method (p) (expectation p #'identity)))

(defgeneric covariance (p)
  (:documentation "covariance PROB-DIST.  Compute the variance (or covariance matrix if RV is vector-valued) of the identity random variable w.r.t PROB-DIST."))

(defgeneric sample-space (p)
  (:documentation "Return a [set], representing the sample space of this distribution.")
  (:method (p) (declare (ignore p)) 'unknown))

(defgeneric sample-iterator (p)
  (:documentation "sample-iterator PROB-DIST.  Returns a function F.  Calling F returns 1) the next sample point, if it exists, or an arbitrary value otherwise 2) The probability of the item, if it exists, or nil otherwise 3) T if there were no more elements.  The first and second return values only make sense if the third one is NIL.  A standard way to use sample-iterator is via do-sample-points.  Default method just uses iterator over sample-space.")
  (:method ((p t))
	   (let ((iter (iterator (sample-space p))))
	     (lambda ()
	       (multiple-value-bind (item done?)
		   (funcall iter)
		 (if done?
		     (values nil nil t)
		   (values item (prob p item) nil)))))))

(defmacro do-sample-points ((item-var prob-var p &optional result-form) &body body)
  "macro do-sample-points (ITEM-VAR PROB-VAR PROB-DIST &optional RESULT-FORM) &body BODY.  Loop over the elements of the sample space of PROB-DIST.  During BODY, ITEM-VAR is bound to successive sample points, and PROB-VAR to the corresponding probability.  If RESULT-FORM is provided, this is the return value.  Otherwise, return NIL.

Note : this is not very efficient for hashtables - use map-prob-dist instead."
  (with-gensyms (iter done)
    `(let ((,iter (sample-iterator ,p)))
       (loop
	 (mvbind (,item-var ,prob-var ,done)
	     (funcall ,iter)
	   (declare (ignorable ,item-var ,prob-var))
	   (when ,done
	     (return ,result-form))
	   ,@body)))))


(defgeneric sample-uniformly (s)
  (:documentation "sample uniformly from set S.  For finite sets this is according to counting measure, and for manifolds, according to the corresponding Lebesgue measure.")
  (:method (s)
	   ;; Default method
	   (check-type s [numbered-set])
	   (item (random (size s)) s)))

(defmethod sample (d)
  (let ((r (random 1.0))
	(tot 0))
    (do-sample-points (u p d)
      (when (> (incf tot p) r)
	(return-from sample u)))))

(defmethod expectation (d rv)
  (let ((sum 0.0))
    (do-sample-points (u p d sum)
      (incf sum (* p (evaluate-rv rv u))))))
      


(defvar *validity-tol* .0001)
(defgeneric is-valid-prob-dist (p)
  (:documentation "is-valid-prob-dist P.  Return true iff P is a valid probability distribution, i.e. that it is positive and sums to 1 over its domain.")
  (:method ((p t))
	   (let ((sum 0.0))
	     (do-sample-points (x prob p (< (abs-diff sum 1)))
	       (unless (and (>= prob 0) (<= prob 1))
		 (return nil))))))


(defgeneric map-prob-dist (fn dist)
  (:documentation "map-prob-dist FN DIST.  FN is a function of two arguments : an item and its probability.  DIST is the probability distribution.  Iterates over sample space of distribution applying FN to each element and its probability.  Returns nil.  Default method just calls do-sample-points.

Not as general as do-sample-points.  Mainly used for hash table distributions, for efficiency.")

  (:method (fn dist)
	   (do-sample-points (x p dist nil)
	     (funcall fn x p))))
  
	      
		  



(defgeneric update-dist (p1 p2 eta)
  (:documentation "update-dist DIST1 DIST2 ETA.  Not implemented for all distributions.  Destructively update DIST1 to (1-ETA)*DIST1 + ETA*DIST2 and return it.  Used by updatef, which is the function that should be called by outside code."))

(defmacro updatef (p1 p2 eta)
  "macro updatef P1 P2 ETA.  Update distribution P1 to ETA*P2 + (1-ETA)*P1.  Not implemented by all distributions."
  `(_f update-dist ,p1 ,p2 ,eta))

(defgeneric clone-dist (p)
  (:documentation "clone-dist P.  The generic function clone can be applied to probability distributions, but may not always behave as desired.  For example, given an association list, clone will clone both the keys and values, whereas, if the alist is being thought of as a probability distribution, it is not necessary to clone either.  The behaviour of clone-dist is
1) For alists, just create a new alist structure, but don't clone the keys or values
"))

(defgeneric marginalize (p vars &key result-type)
  (:documentation "marginalize PROB-DIST VARS &key (RESULT-TYPE 'list).  Returns the marginal distribution of the random variables designated by the elements of list VARS.  RESULT-TYPE must either be 'list, 'vector, 'single (only allowed if there's a single variable), or an instantiation accessor (see help inst-vars).  RESULT-TYPE defaults to 'list.  It is also allowed to have VARS to be a single (nonlist) random variable, in which case RESULT-TYPE defaults to 'single.  

This operation will be implemented only for certain combinations of PROB-DIST and random variables.  Note that the VARS are designators for random variables - they can be objects that aren't actually [random-variable]s themselves, but refer to one in this context, in a manner depending on the specific PROB-DIST type.  For example, given a class of distributions over R^n, we may choose to allow integer i to denote the ith coordinate.  The one fixed case is that a function always refers to itself.")
  (:method :around (p vars &key (result-type nil result-supp) &aux (var-was-list (listp vars)) (vars (designated-list vars)) (n (length vars)))
	   (unless result-supp
	     (setf result-type (if var-was-list 'list 'single)))
	   (call-next-method 
	    p vars
	    :result-type
	    (case result-type
	      (list (inst-vars:make-list-accessors n))
	      (vector (inst-vars:make-vec-accessors n))
	      (otherwise result-type))))
  (:method (p vars &key result-type)
	   (make-rv-dist 
	    #'(lambda (u) 
		(if (eq result-type 'single)
		    (evaluate-rv (first vars) u)
		  (let ((inst (inst-vars:create-inst result-type)))
		    (do-elements (v vars inst i)
		      (inst-vars:set-var-val result-type inst i (evaluate-rv v u))))))
	    p)))


(defgeneric condition-on-dist (dist cond-dist v)
  (:documentation "condition-on-dist DIST COND-DIST V.  Suppose X is sampled from DIST, Y is sampled from COND-DIST given X, and observed to equal V.  This function returns the distribution over X given this evidence.

Default method assumes X's are compared using #'equal.")
  (:method (d cond-dist v)
	   (let ((h (make-hash-table :test #'equal))
		 (sum 0))
	     (do-sample-points (u p d)
	       (let ((s (* p (cond-prob cond-dist v u))))
		 (unless (zerop s)
		   (incf sum s)
		   (setf (gethash u h) s))))
	     (assert (> sum 0) nil "Conditioning ~a and ~a on impossible outcome ~a" d cond-dist v)
	     (maphash 
	      #'(lambda (k v)
		  (setf (gethash k h) (/ v sum)))
	      h)
	     h)))

(defgeneric chain-dists (dist cond-dist &key inst-acc)
  (:documentation "chain-dists DIST COND-DIST &key (INST-ACC 'list).  Return a distribution over X,Y corresponding to sampling X from DIST, then sampling Y from COND-DIST given X.  INST-ACC must either be the symbol 'list or an instantiation accessor (see help inst-vars).")
  (:method (dist cond-dist &key (inst-acc 'list))
	   (assert (eq inst-acc 'list) nil "For now, can only use lists in chain-dists")
	   (let ((l nil))
	     (do-sample-points (x p dist l)
	       (do-sample-points (y q (cond-dist cond-dist x))
		 (push (cons (list x y) (* p q)) l))))))

(defgeneric transform-dist (dist cond-dist)
  (:documentation "transform-dist DIST COND-DIST.  Consider sampling X from DIST and Y from COND-DIST given X.  Returns the marginal distribution of Y in this situation.")
  (:method (dist cond-dist)
	   (let ((h (make-hash-table :test #'equal)))
	     (do-sample-points (x p dist h)
	       (do-sample-points (y q (cond-dist cond-dist x))
		 (setf (gethash y h) (+ (* p q) (or (gethash y h) 0.0))))))))



(defgeneric total-variation-distance (d1 d2)
  (:documentation "total-variation-distance D1 D2.  Return the total variation distance between probability distributions D1 and D2, which is the sup over all sets S of |D1(S) - D2(S)|.")
  (:method (d1 d2)
	   ;; TODO this is only correct for discrete distributions
	   (let ((s 0))
	     (do-sample-points (x p d1 (/ s 2))
	       (incf s (abs-diff p (prob d2 x)))))))


(defun pprint-prob-dist (&rest args)
  (bind-pprint-args (str d) args
    (print-prob-dist str d)))


(defvar *print-prob-precision* 4)

(defgeneric print-prob-dist (str d)
  (:method (str d)
	   (if (eq (sample-space d) 'unknown)
	       (print-unreadable-object (d str :type t))
	     (let ((first t))
	       (pprint-logical-block (str nil :prefix "[" :suffix "]")
		 (do-sample-points (x p d)
		   (when (> p 0)
		     (if first
			 (setf first nil)
		       (format str ", "))
		     (format str "(~w : ~6,vf)" x *print-prob-precision* p)
		     (pprint-newline :fill str)
		     (pprint-pop))))))))


(set-pprint-dispatch '<prob-dist> #'pprint-prob-dist)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations for conditional probability distributions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defgeneric cond-dist (p x)
  (:documentation "cond-dist P X.  Returns a [prob-dist] representing the conditional distribution given X.  May throw a cond-dist-not-defined condition.

Should never modify the object being conditioned on, for example in generating the possible values of the new variable.")
  (:method ((p function) x)
	   (handler-case
	       (funcall p x)
	     (cond-dist-not-defined ()
	       (error 'cond-dist-not-defined :x x :cond-dist p)))))


(defgeneric cond-prob (p y x)
  (:documentation "cond-prob P Y X.  Returns the conditional probability of Y given X.  May throw an element-not-in-domain or cond-dist-not-defined condition if either Y or X is illegal.  By default, just call cond-dist then evaluate the result on Y.

Implementors should never modify X.")
  (:method ((p t) y x)
	   (prob (cond-dist p x) y)))

(defun cond-sample (p x)
  "cond-sample P X.  Sample from P(Y|X).  Might throw a cond-dist-not-defined condition.

Implementors should never modify the object X."
  (sample (cond-dist p x)))


(defgeneric cond-exp (p rv &optional force-eval)
  (:documentation "cond-exp COND-DIST RV &optional (FORCE-EVAL nil).  Suppose COND-DIST is a conditional distribution P(Y|X), and RV is a random variable on Y.  Returns a random variable f on X, where f(x) = E(RV(Y)|X=x).  This operation occurs frequently in Bellman equations for Markov decision processes.  Default operation just returns a function that does the expectation whenever it's called on some specific X.  Subclasses might take advantage of structure in P and RV.  

Setting FORCE-EVAL true indicates a preference for doing as much computation as possible when this method is called, so that carrying out operations on the returned random variable is fast.  Methods may or may not take this into account.

Might throw a cond-dist-not-defined condition.")
  (:method (p rv &optional (force-eval nil))
	   (assert (not force-eval) () "default method for cond-exp does not support FORCE-EVAL right now.")
	   (lambda (x) (expectation (cond-dist p x) rv))))

(defgeneric conditional-expectation (p event &key rv)
  (:documentation "conditional-expectation PROB-DIST EVENT &key (RV #'identity).  Conditional expectation of RV given EVENT."))


(defun make-cond-dist-from-matrix (a)
  "make-cond-dist-from-matrix A.  Treat A as a 'transition matrix' and return the corresponding conditional distribution."
  (let ((num-cols (array-dimension a 1))
	(last-row (1- (array-dimension a 0))))
    (lambda (i)
      (check-type i fixnum)
      (assert (between i 0 last-row))
      (make-array num-cols :displaced-to a :displaced-index-offset (* i num-cols)))))

    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; random variables
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric evaluate-rv (rv x)
  (:documentation "evaluate-rv RV X.  Evaluate this random variable at sample point X."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sampling
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun rejection-sampling (p event)
  "rejection-sampling D EVENT.  This is the simple version of rejection sampling, where you sample repeatedly from D until you get something that belongs to EVENT, and return that.  D can be a [prob-dist] or sampling function."
  (loop
    (let ((x (sample p)))
      (when (member? x event)
	(return x)))))