(defpackage direct-product-set
  (:documentation "Package direct-product-set (prod-set)

Contains operations related to objects of type <prod-set>.

Types
-----
<direct-product-set>
<prod-set>


Operations
----------
direct-product (also in package set)
sets
inst-acc
make-subspace
fast-product

Other symbols
-------------
not-used
")
  (:export
   <direct-product-set>
   <prod-set>
   direct-product
   sets
   inst-acc
   make-subspace
   fast-product
   not-used
   )
  (:nicknames prod-set)
    (:use
   cl
   set
   inst-vars
   utils))

(in-package prod-set)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Class definitions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <direct-product-set> (<set>)
  ((sets :reader sets :writer set-sets :type vector)
   (dim :writer set-dim :reader dim :type fixnum)
   (inst-accessor :writer set-inst-acc :reader inst-acc :initarg :inst-acc))
  (:documentation "<direct-product-set> (<set>)

Represents the direct product of a finite number of sets.  Uses space and time proportional to the individual set sizes and dimension rather than the set of joint instantiations.

Initargs
:sets - a vector of the component sets.
:inst-acc - an instantiation accessor to represent elements of the product set (see inst-vars package).

It is an error for any of the values in the sets to be the symbol 'uninstantiated"))



(defclass <prod-set> (<direct-product-set> <numbered-set>)
  ((size :writer set-size :reader prod-set-size :initform ':unknown)
   (sizes :reader sizes :writer set-sizes :type vector)
   (iterate-quickly :initarg :iterate-quickly :initform nil :reader iterate-quickly)
   (size-products :writer set-size-prods :reader size-prods :type (simple-array * 1)
		  :documentation "For example, if the successive dimensions have cardinality 3, 4, 2, 5 then this would be the array #(40 10 5 1)")
)
  (:documentation "A subclass of <direct-product-set> where the components are finite, allowing item numbering, iteration, etc. for sets of instantiations to a finite set of variables.  

Required initargs same as <direct-product-set>
:sets 
:inst-acc 

Optional
:iterate-quickly - nil by default.  If t, then iteration operations (iterator, do-elements, mapset) will work in a more efficient way by not creating a new object each time. This will work ok so long as the calling code does not modify the returned objects, or expect their values to persist (e.g. by saving them in a lexical closure).

"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; constructor
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod initialize-instance :after ((ps <direct-product-set>) &rest args &key sets)
  (declare (ignore args))
  (set-sets (coerce sets 'vector) ps)
  (let ((dim (length sets)))
    (set-dim dim ps)
    (set-equality-test
     #'(lambda (x y)
	 (each dim #'(lambda (i) (funcall (equality-test (aref (sets ps) i)) (get-comp ps x i) (get-comp ps y i)))))
     ps)))

  

(defmethod initialize-instance :after ((ps <prod-set>) &rest args &key sets)
  (declare (ignore args))
  (let ((sizes (map 'vector #'size sets))
	(dim (dim ps)))
    
    (assert (every #'numberp sizes))
    (set-sizes sizes ps)
    (let ((size-prods 
	   (reverse
	    (let ((cumulative-prod 1))
	      (map 'vector
		   (lambda (x)
		     (prog1 
			 cumulative-prod
		       (_f * cumulative-prod x)))
		   (reverse sizes))))))

      (set-size 
       (if (> dim 0)
	   (* (aref sizes 0) (aref size-prods 0))
	   1)
       ps)
      (set-size-prods size-prods ps))))

(defun fast-product (sets)
  "fast-product SETS
Return the direct product of the SETS.  Elements are represented as vectors.  Iteration is fast, i.e., a new vector is not allocated each time.  So don't use this set if you ever want to be able to refer simultaneously to two different elements of it ."
  (make-instance '<prod-set> :sets sets :iterate-quickly t :inst-acc (make-list-accessors (length sets))))


(defmethod clone ((s <prod-set>))
  (make-instance '<prod-set> :sets (sets s)
		 :inst-acc (inst-acc s)))


(defmethod print-object ((s <direct-product-set>) str)
  (print-unreadable-object (s str :type t :identity nil)
    (format str "with ~D components" (length (sets s)))))
    
(defun pprint-prod-set (str s)
  (pprint-logical-block (str (coerce (sets s) 'list) 
			     :prefix "[Product set with components "
			     :suffix "]")

    (loop
       (format str "~W" (pprint-pop))
       (pprint-exit-if-list-exhausted)
       (format str ", "))))

(set-pprint-dispatch '<direct-product-set> #'pprint-prod-set 1)

(defun direct-product (result-type &rest sets)
  "direct-product RESULT-TYPE &rest SETS.

RESULT-TYPE - either 'list, 'vector, or a list
SETS - a list of sets

This function is a constructor for the <prod-set> or <direct-product> class (you can also use make-instance).  If RESULT-TYPE is 'list (resp 'vector) the elements are lists (resp vectors).  If it is a list (which must be of the same length as SETS), elements are association lists, in which keys are compared using #'equal.  The :iterate-quickly flag is set to nil.  If all the sets have known finite size, <prod-set> is used, otherwise <direct-product-set> is used.

For example, if SETS is '(3 (foo bar)) and RESULT-TYPE is 'list, then a typical element would be '(1 bar).  If RESULT-TYPE is '(qux oof), then a typical element would be ((qux . 1) (oof . bar))."
  
  (let ((d (length sets)))
    (make-instance 
     (if (every (fn (numberp size)) sets) '<prod-set> '<direct-product-set>)
     :inst-acc (etypecase result-type
		 ((member list) (inst-vars:make-list-accessors d))
		 ((member vector) (inst-vars:make-vec-accessors d))
		 (list (inst-vars:make-alist-accessors result-type #'equal)))
     :sets sets)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; subtypes of item-not-in-set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; jawolfe - not sure if this is right, but it seems to fix the error...
;#+sbcl (defmethod sb-mop:validate-superclass ((class SB-PCL::standard-class) (super SB-PCL::condition-class)) t)

(define-condition <invalid-inst> (item-not-in-set)
  ((reason :initarg :reason :reader reason))
  (:documentation "Class <invalid-inst> (item-not-in-set)
Represents an explanation for an item not to be in a product set because of it not being a well-formed instantiation of the sort required by that product set."))


(defmethod get-explanation-string ((c <invalid-inst>))
  (format nil "the instantiation is invalid for reason ~a" (reason c)))


(define-condition <invalid-component> (item-not-in-set)
  ((comp :initarg :comp :reader comp)
   (comp-set :initarg :comp-set :reader comp-set)
   (reason :initarg :reason :reader reason))
  (:documentation "Class <invalid-component> (item-not-in-set)
Represents an explanation for an item not to be in a product set because one of its components is not a member of the corresponding component set."))

(defmethod get-explanation-string ((c <invalid-component>))
  (format nil "component ~a is not a member of set ~a, the reason for *that* being ~a"
	  (comp c) (comp-set c) (get-explanation-string (reason c))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; accessing instantiations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declaim (inline get-comp set-comp make-empty-element))

(defun get-comp (ps x i)
  "get-comp PRODUCT-SET INSTANTIATION VAR-NUM."
  (get-var-val (inst-acc ps) x i))

(defun set-comp (ps x i val)
  "set-comp PRODUCT-SET INSTANTIATION VAR-NUM VALUE"
  (set-var-val (inst-acc ps) x i val))

(defun make-blank-inst (ps)
  "make-blank-inst PRODUCT-SET"
  (create-inst (inst-acc ps)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations from set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (x (ps <direct-product-set>))
  (flet ((not-member (reason-class &rest reason-args)
	   (return-from member?
	     (values nil (apply #'make-instance reason-class reason-args)))))
    (aif (detect-invalid-inst (inst-acc ps) x)
	 (not-member '<invalid-inst>
		     :item x :set ps :reason it)
	 (loop
	     for i below (dim ps)
	     for s across (sets ps)
	     for comp = (get-comp ps x i)

	     do (multiple-value-bind (comp-is-member? reason)
		    (member? comp s)
		  (unless comp-is-member?
		    (not-member '<invalid-component>
				:item x :set ps
				:comp comp :comp-set s
				:reason reason)))
	     finally (return t)))))
		
			

(defmethod item-number (x (ps <prod-set>))
  (assert (known-finite ps))
  (loop
      for i below (dim ps)
      for prod across (size-prods ps)
      for s across (sets ps)
      for y = (get-comp ps x i)
		   
      sum (* prod (item-number y s))))

(defmethod item (num (ps <prod-set>))
  ;; TODO signal condition when num out of bounds
  (assert (known-finite ps))
  (let ((y num)
	(x (make-blank-inst ps)))
    (loop
	for i below (dim ps)
	for s across (sets ps)
	for p across (size-prods ps)
	do (multiple-value-bind (q r)
	       (floor y p)
	     (setf y r)
	     (set-comp ps x i (item q s)))
	finally (return x))))


(defmethod size ((s <prod-set>) &optional (constant-time nil))
  (declare (ignore constant-time))
  (prod-set-size s))

(defmethod iterator ((s <prod-set>))
  (assert (known-finite s))
  (if (iterate-quickly s)
      
      ;; if iterating quickly, do a more efficient version that uses the same
      ;; object each time
      (let ((sizes (sizes s))
	    (dim (dim s))
	    (sets (sets s))
	    (total-size (prod-set-size s))
	    (acc (inst-acc s)))
	(assert (every #'numberp sizes) ()
	  "Iterator cannot be defined because the coordinate sizes ~a of ~a are not all numbers."
	  sizes s)
	
	(if (> total-size 0)
	    
	    ;; if set is nonempty
	    (let ((current-ind 0)
		  (inst (item 0 s))
		  (iterators (map 'vector #'iterator sets)))
	  
	      ;; start each iterator off
	      (map nil #'funcall iterators)

	  
	      (if (some #'zerop sizes)
		  ;; if the set is empty, the iterator is done immediately
		  (lambda ()
		    (iterator-done))
	    
		;; Otherwise
		(lambda ()
	      
		  ;; the prog1 is to step current-ind each time
		  (multiple-value-prog1
		  
		      ;; deal with init and finish cases
		      (if (eql current-ind 0)
			  (iterator-not-done inst)
			(if (>= current-ind total-size)
			    (iterator-done)
		      
			  ;; main case
			  (loop
			
			  ;;; start at end and work way back
			      for i from (1- dim) downto 0
						   
			      do (multiple-value-bind (val done?)
				     (funcall (aref iterators i))
			       
			       ;;; if this iterator cannot be stepped, reset this coord to beginning 
				   (if done?
				       (let ((iter (iterator (aref sets i))))
					 (setf (aref iterators i) iter)
					 (multiple-value-bind (val done?)
					     (funcall iter)
					   (assert (not done?) () "iterator unexpectedly finished for set ~a"
						   (aref sets i))
					   (set-var-val acc inst i val)))
				 
				     ;; otherwise, step this iterator and end loop
				     (return 
				       (progn
					 (set-var-val acc inst i val)
					 (iterator-not-done inst)))))
			     
			      finally (assert () () "Iteration error for prod set ~a, current item ~a" s inst))))
		    (incf current-ind)))))
	
	  ;; if set is empty
	  (lambda () (values nil t))))
  
    ;; if the iterate-quickly flag is off, just use the default method of set
    (call-next-method)))
    
  
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; subspaces
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun make-subspace (s vars &optional (iterate-quickly nil))
  "make-subspace S VAR-NAMES &optional (ITERATE-QUICKLY nil)

S is a product set.
VAR-NAMES is a list of variables names.

Create a representation of the set of instantiations to the variables in VARS.  The elements of this set will actually be full joint instantiations from S, but the variables not in VARS will be ignored, or have their value set to 'not-used.

ITERATE-QUICKLY has the same meaning as when creating new <prod-set> objects."
  (let* ((sets (sets s))
	 (acc (inst-acc s))
	 (names (var-names acc))
	 (var-nums (mapcar (lambda (n) 
			     (item-number n names))
			   vars)))
      (make-instance '<prod-set>
	:sets (map 'vector (lambda (v) (aref sets v)) var-nums)
	:inst-acc (make-subinst-accessors (inst-acc s) vars)
	:iterate-quickly iterate-quickly)))


(defun make-subinst-accessors (inst-acc vars)
  "make-subinst-accessors INST-ACC VARS.  
INST-ACC - instantiation accessor
VARS - list of variable ids

Given an accessor for overall joint instantiations, we want to make a sub-instantiation accessor for a subset of the variables that can take in instantiations to the entire set of variables, but treat them as if they were projected down to the subset."
  
  (let ((n (num-vars inst-acc))
	(index (map 'vector #'(lambda (v) (get-var-num inst-acc v)) vars)))
    (let ((creator #'(lambda () 
		     (let ((a (create-inst inst-acc)))
		       (dotimes (i n)
			 (set-var-val inst-acc a i 'not-used))
		       (map nil 
			 #'(lambda (j) 
			     (set-var-val inst-acc a j 'uninstantiated))
			 index)
		       a)))
	  (readers (map 'vector
		     #'(lambda (j)
			 #'(lambda (a) 
			     (get-var-val inst-acc a j)))
		     index))
	  (writers (map 'vector
		     #'(lambda (j)
			 #'(lambda (val a)
			     (set-var-val inst-acc a j val)))
		     index))
	  (invalid-inst-detector 
	   #'(lambda (i) 
	       (or (detect-invalid-inst inst-acc i)
		   (dotimes (j n)
		     (unless (member? j index)
		       (let ((val (get-var-val inst-acc i j)))
			 (unless (eq val 'not-used)
			   (return (list 'unused-variable-has-value j val)))))))))
		   
	  (var-num (lambda (v) (position v vars :test #'equal))))
      
      (make-inst-var-accessors
       :creator creator
       :readers readers
       :writers writers
       :var-num var-num
       :invalid-inst-detector invalid-inst-detector
       :var-names vars))))
			   
	 



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Cumulative product of list elements
;; i.e. given a list (n1 n2 .. nk) returns
;; (1 n1 n1*n2 ... n1*n2*...*nk)
(defun cumprod (l)
  (if (null l)
      '(1)
    (cons 1
	  (mapcar 
	   #'(lambda (x) (* x (first l)))
	   (cumprod (rest l))))))

(defun known-finite (s)
  (numberp (prod-set-size s)))
	  
  
(in-package common-lisp-user)    











