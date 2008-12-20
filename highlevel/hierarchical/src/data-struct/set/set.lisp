(defpackage set
  (:documentation "Package for operations related to sets.

Set types, creation
- [set]
- [numbered-set]
- <set>
- <numbered-set>
- <prod-set>
- <direct-product-set>
- <indexed-set>
- <directory-set>
- powerset
- make-image-set
- <image-set>
- <filtered-set>
- objects-satisfying
- <recursive-enumeration>
- subsets-of-size
- <subsets-of-size>
- <tree-set>
- random-cycle
- random-permutation
- ndlet
- ndlet-fail
- ndlet*
- ndunion
- permutations
- bijections
- complement-set
- <powerset>
- int-range
- <interval>

Symbols naming sets
- natural-numbers
- real-numbers

Exported Operations
- member? 
- membership-predicate 
- equality-test 
- set-equality-test
- iterator
- size 
- is-empty
- item-number
- item
- last-item
- do-elements
- do-sets
- add
- addf
- mapset
- to-list
- to-vector
- filter
- each
- any
- find-element
- reduce-set
- sum-over
- subset
- binary-intersection
- intersect
- intersects
- <implicit-intersection>
- implicit-intersection
- implicit-union
- union-sets
- binary-union
- <implicit-union>
- disjoint-union
- disjoint-union-list
- disjoint-union-of-sets
- unite
- set-eq
- symmetric-difference
- direct-product
- pprint-set
- base-set
- vdc-sequence
- product-vdc-sequence


Operations for children
- iterator-done
- iterator-not-done

Operations on specific set types
- left-bound, right-bound, left-open, right-open, make-closed-interval, interval-length (intervals)

Conditions
- item-not-in-set
- index-out-of-bounds

Operations on conditions
- get-explanation-string
- get-full-explanation-string

Other symbols
- fail

")
  (:use common-lisp
	utils)
  (:export [set]
	   [numbered-set]
	   <set>
	   <numbered-set>
	   <indexed-set>
	   <directory-set>
	   <filtered-set>
	   objects-satisfying
	   make-image-set
	   <image-set>
	   <recursive-enumeration>
	   subsets-of-size
	   <subsets-of-size>
	   <tree-set>
	   random-cycle
	   random-permutation
	   ndlet
	   ndlet-fail
	   ndunion
	   ndlet*
	   permutations
	   bijections
	   complement-set
	   <prod-set>
	   <direct-product-set>
	   <powerset>
	   int-range
	   <interval>
	   
	   natural-numbers
	   real-numbers
	   member?
	   membership-predicate
	   equality-test
	   set-equality-test
	   iterator
	   iterator-not-done
	   iterator-done
	   item-number
	   item
	   last-item
	   do-elements
	   do-sets
	   mapset
	   to-list
	   to-vector
	   filter
	   each
	   any
	   find-element
	   reduce-set
	   sum-over
	   subset

	   binary-intersection
	   intersect
	   intersects
	   <implicit-intersection>
	   binary-union
	   implicit-union
	   union-sets
	   <implicit-union>
	   disjoint-union
	   disjoint-union-list
	   disjoint-union-of-sets
	   unite
	   set-eq
	   symmetric-difference
	   direct-product
	   pprint-set
	   base-set
	   vdc-sequence
	   product-vdc-sequence
	   
	   add
	   addf
	   powerset
	   size
	   is-empty
	   item-not-in-set
	   index-out-of-bounds
	   get-explanation-string
	   get-full-explanation-string
	   
	   left-bound
	   right-bound
	   left-open
	   right-open
	   make-closed-interval
	   interval-length
	   
	   fail
	   ))

(in-package set)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; type defs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defgeneric equality-test (s)
  (:documentation "A symbol naming a globally defined function that takes in X and Y and returns a boolean.  In the special case of eq, equalp, equal, or eql, it can also be the corresponding function object.  For hash table sets, this is the equality test of the table.  For other sets that aren't classes, this is always #'equal.  For sets that are classes (and subclasses of <set>), this defaults to #'equal, but you can change it using the :equality-test initarg.")
  (:method (s) (declare (ignore s)) #'equal))

(defclass <set> ()
  ((name :type string :reader name :initarg :name :initform "")
   (equality-test :initarg :equality-test :reader equality-test :initform #'equal :writer set-equality-test))
  (:documentation "Abstract class for sets."))

(deftype [set] ()
  "Type for sets.  Can be one of
1. A numbered set (see [numbered-set])
2. Object of type <set>
3. The symbol 'real-numbers
4. The symbol t (denoting the universal set)"
  `(or <set> [numbered-set] (eql 'real-numbers) (eql t)))


(defclass <numbered-set> (<set>)
  ()
  (:documentation "Abstract class for numbered sets."))

(deftype [numbered-set] ()
  "Type for numbered sets, i.e., sets in which the elements are numbered, starting from 0.  Iterating over a numbered sets always returns the elements in increasing order of number.  Can be one of
1. A sequence, signifying the set of its elements.  Operations on such sets implicitly assume the members of the sequence are all different, and #'equal is used as the equality test.
2. A cons of an equality test function and a vector.  This is like 1 except with the given equality test.
3. A fixnum n, signifying the set {0,1,...,n-1}
4. A hashtable from items to numbers
5. 'natural-numbers, signifying the set {0,1,2,...}
6. Object of type <numbered-set>"
  `(or cons null vector fixnum hash-table gen-hash-table <numbered-set> (eql 'natural-numbers)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; conditions and related types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define-condition item-not-in-set ()
  ((item :initarg :item :reader get-item)
   (s :initarg :set :reader get-set))
  (:report (lambda (c s) (format s "~a not in set ~a for reason ~a" (get-item c) (get-set c) (get-explanation-string c))))
  (:documentation "Objects of type item-not-in-set play two roles.  First, when calling the function member?, they are returned as a secondary value that denotes the reason why an item is not a member of a set.  Second, certain functions (e.g., item-number) require certain set membership relations to hold.  When they do not, those functions may signal a condition of type item-not-in-set. 

Subclasses should implement get-explanation-string."))


(define-condition index-out-of-bounds ()
  ((ind :reader ind :initarg :ind)
   (s :initarg :set :reader get-set)
   (max-ind :reader max-ind :initarg :max-ind))
  (:report (lambda (c s) (format s "Index ~a out of bounds for set ~a (max allowed is ~a)" (ind c) (get-set c) (max-ind c))))
  (:documentation "Signalled when trying to find an element with a nonexistent index in a [numbered-set]."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; generic functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric member? (item s)
  (:documentation "member? ITEM SET.  Return T iff ITEM or something that satisfies the equality test of SET with it.  If returning nil, may return a second value, which is an object of type item-not-in-set that denotes the reason ITEM is not a member of SET.  There is currently an :around method at the top-level which, if the item is not a member and the reason is nil, makes an item-not-in-set object that includes the item and the set (this is a temporary solution - eventually, the second return value should be mandatory).")
  (:method :around (item s)
	   (multiple-value-bind (present? reason)
	       (call-next-method)
	     (values present?
		     (if (or present? reason)
			 reason
		       ;; if not present, and a reason is not provided, make a default one
		       ;; TODO this is a bit hacky
		       (make-instance 'item-not-in-set :item item :set s))))))




(defun membership-predicate (s)
  #'(lambda (x) (member? x s)))

(defgeneric iterator (s)
  (:documentation "iterator SET.  Returns a function F.  Calling F returns 1) the next item of F, if it exists, or an arbitrary value otherwise 2) T if there were no more elements.  The first return value only makes sense if the second one is NIL.  A standard way to use iterator is via do-elements or mapset.  There is a default method for iterator that just uses item-number

Also works for functions - just returns the function back.")
  (:method ((s function)) s))

(defgeneric item-number (item s)
  (:documentation "item-number ITEM NUMBERED-SET.  Return the number of item in the set.  Uses (equality-test SET) for equality checking. Signals an 'item-not-in-set error if item is not found."))

(defgeneric item (num s)
  (:documentation "item NUM NUMBERED-SET. Return the item with this number.  An around method for the abstract class ensures that the number is a valid index for this set, or signals an 'index-out-of-bounds error otherwise.")
  (:method :around (num s)
	   (let ((size (size s t)))
	     (if (or (symbolp size)
		     (and (numberp size) (between2 num 0 size)))
		 (call-next-method)
	       (error 'index-out-of-bounds :set s :ind num :max-ind (1- size))))))
	     

(defgeneric size (s &optional constant-time)
  (:documentation "size SET &optional (CONSTANT-TIME nil).  If CONSTANT-TIME is nil, either returns the number of items in the set, the symbol ':unknown, or the symbol 'infty.  If CONSTANT-TIME is t, is guaranteed to run in constant time (if it can't compute the size in constant time, it just returns ':unknown.")
  (:method ((s t) &optional (constant-time nil)) (declare (ignore constant-time)) ':unknown))

(declaim (inline last-item))
(defun last-item (s)
  "last-item SET.  If it's a finite numbered set, return the item with the highest number."
  (item (1- (size s)) s))

(defgeneric is-empty (s)
  (:documentation "is-empty SET.  The top-level method just checks if the size equals 0, but for certain types of set, it may be much easier to check whether the set is empty than to actually count the number of elements.")
  (:method ((s t)) (zerop (size s))))

(defgeneric add (s item &optional pos)
  (:documentation "add S ITEM &optional (POS nil).  If ITEM doesn't exist in the set S, add it and return the new set.  If not, return the original.  Might destructively modify the original set.  If POS is nil, then the item numbers in the set may be changed arbitrarily.  If POS is t, then none of the existing item numbers are changed, and if the item is being added newly, it will be added to the end (i.e. with the highest item number).  If this is not possible (e.g. with a sorted set representation), an assert should happen."))


(defgeneric get-explanation-string (c)
  (:documentation "get-explanation-string ITEM-NOT-IN-SET-CONDITION.  Return a string that explains verbally why an item is not in a set.  It should be assumed to come after a string of the form 'X was not a member of S, the reason being ' and followed by a period.  See also get-full-explanation-string.")
  (:method (c)
	   (if (symbolp (get-item c))
	       "unspecified (the item is a symbol.  If a symbol with the same name appears to be in the set, then perhaps there's a package issue)"
	     "unspecified")))

(defun get-full-explanation-string (c)
  "get-full-explanation-string ITEM-NOT-IN-SET-CONDITION.  Return a string that explains why an item is not in a set.  The string will be a full sentence of the form 'X was not a member of S. The reason: '.  Calls get-explanation-string."
  (format nil "~a was not a member of ~a.  Reason: ~a"
	  (get-item c) (get-set c) (get-explanation-string c))) 
  
(defgeneric vdc-sequence (s &optional constant-space base)
  (:documentation "Assumes S can be viewed as a submanifold of R^n.  Returns a function of no arguments, based on the Van der Corput sequence with the given base, such that repeated calls to it generate a dense sequence in S.  If CONSTANT-SPACE is t, repeated calls will modify the same underlying vector object, while if it is nil (the default), each call will return a fresh vector.  The second return value is the new base."))

(defun product-vdc-sequence (result-type sets base &optional (constant-space nil))
  (assert (eql result-type 'vector))
  (let ((functions
	 (mapset 'list #'(lambda (s) 
		     (mvbind (f b) (vdc-sequence s constant-space base)
		       (setf base b)
		       f))
		 sets))
	(v (make-array (length sets))))
    (values
     #'(lambda ()
	 (map-into v #'(lambda (f) (funcall f)) functions)
	 (if constant-space v (copy-seq v)))
     base)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations for use by children
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro iterator-not-done (item)
  `(values ,item nil))

(defmacro iterator-done ()
  `(values nil t))
	    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations defined here for external use
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    

(defmacro do-elements ((var s &optional result-form num-var) &body body)
  "macro do-elements (VAR SET &optional RESULT-FORM NUM-VAR) &body BODY.  Loop over the elements of a set in order.  During BODY, VAR is bound to successive elements of the set.  If NUM-VAR is provided, then during BODY, it is bound to the index of the corresponding element.  If RESULT-FORM is provided, this is the return value.  Otherwise, return NIL."
  `(do-iterator (iterator ,s) (,var ,result-form ,num-var) ,@body))

(defmacro do-sets (vars (&optional (result-form nil)  (num-var (gensym))) &body body)
  "do-sets ((VAR1 SET1) ... (VARn SETn)) (&optional RESULT-FORM NUM-VAR) &body BODY

Like do, except each variable iterates through the corresponding set.  Stops as soon as one of the sets gets over."
  (let ((var-names (mapcar #'first vars))
	(sets (mapcar #'second vars))
	(done? (gensym))
	(iter-vars (mapcar #'(lambda (x) (declare (ignore x)) (gensym)) vars)))
    `(let ,(append (mapcar #'(lambda (v s) `(,v (iterator ,s))) iter-vars sets)
	    var-names
	    `((,num-var 0) ,done?))
       (declare (ignorable ,num-var))
	 (loop
	 ,@(mapcan #'(lambda (v iter)
		       `((mvsetq (,v ,done?) (funcall ,iter))
			 (when ,done? (return))))
		   var-names iter-vars)
	 ,@body
	   (incf ,num-var))
       ,result-form)))

(defmethod item (num s)
  (do-elements (x s (error 'index-out-of-bounds :set s :ind num :max-ind (1- i)) i)
    (when (eq i num )
      (return-from item x)))
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; mapset
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric mapset (result-type fn set1 &rest sets)
  (:documentation "mapset RESULT-TYPE FN SET1 &rest SETS.  Apply FN to each element of SET.  If RESULT-TYPE is 'list, return the results in a list, and if its 'vector, return the results in a vector. If its a list of the form '(vector type), return the results in a vector where the elements are of that type.  If RESULT-TYPE is :same, return a set of the same type as SET1 (not always implemented, and the exact meaning of this will depend on the subtype of SET1).  In the vector case, the vector is guaranteed to be simple."))

(defmethod mapset ((result-type (eql 'list)) fn set1 &rest sets)
  (apply #'map-iterator-to-list fn (mapcar #'iterator (cons set1 sets))))

(defun map-into-vector (v f &rest sets)
  (let* ((iterators (mapcar #'iterator sets))
	 (items (make-list (length sets))))
    (loop
      (map-into items
		#'(lambda (iter)
		    (mvbind (next done) (funcall iter)
		      (when done (return-from map-into-vector v))
		      next))
		iterators)
      (vector-push-extend (apply f items) v))))

(defmethod mapset ((result-type (eql 'vector)) fn set1 &rest sets)
  (apply #'map-into-vector (make-array 0 :adjustable t :fill-pointer 0) fn set1 sets))

(defmethod mapset ((result-type list) fn set1 &rest sets)
  (assert (eq (first result-type) 'vector))
  (apply #'map-into-vector (make-array 0 :element-type (second result-type) :adjustable t :fill-pointer 0) fn set1 sets))

(defgeneric to-list (s)
  (:method (s) (map-iterator 'list #'identity (iterator s))))

(defgeneric to-vector (s)
  (:documentation "make a vector of the elements of S.")
  (:method (s) (mapset 'vector #'identity s)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; filter
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric filter (result-type s pred)
  (:documentation "filter RESULT-TYPE SET PREDICATE.  Return a new set containing the elements of SET that satisfy PREDICATE.  RESULT must be either ':list or ':implicit or ':vector or ':equal-hash-table"))


(defmethod filter ((result-type (eql ':list)) s pred)
  (let ((l nil))
    (do-elements (x s l)
      (when (funcall pred x) (push x l)))))

(defmethod filter ((result-type (eql ':vector)) s pred)
  (let ((v (make-array 0 :adjustable t :fill-pointer 0)))
    (do-elements (x s v)
      (when (funcall pred x) (vector-push-extend x v)))))

(defmethod filter ((result-type (eql ':implicit)) s pred)
  (make-instance '<filtered-set> :base-set s :predicate pred)) 

(defmethod filter ((result-type (eql ':equal-hash-table)) s pred)
  (let ((h (make-hash-table :test #'equal)))
    (do-elements (x s h)
      (when (funcall pred x) (setf (gethash x h) t)))))

(defun each (s pred)
  "each SET PREDICATE.  Iterates through SET.  If a member is encountered that does not satisfy PREDICATE, immediately return nil.  At the end, return t."
  (do-elements (x s t)
    (unless (funcall pred x)
      (return-from each nil))))

(defun any (s pred)
  "any SET PREDICATE.  Iterate through SET.  If a member is encountered that satisfies PREDICATE, immediately return t.  At the end, return nil."
  (do-elements (x s nil)
    (when (funcall pred x)
      (return-from any t))))

(defun find-element (s pred)
  "find-element SET PREDICATE.  If there exists an element of SET satisfying PREDICATE, returns 1) one such element 2) t.  Otherwise, returns nil and nil."
  (do-elements (x s (values nil nil))
    (when (funcall pred x)
      (return-from find-element (values x t)))))

(defgeneric reduce-set (fn s &key initial-value key)
  (:documentation "reduce-set FUNCTION SET &key INITIAL-VALUE (KEY #'identity)

Works just like reduce, except goes through elements of SET in order.
If initial-value is provided, starts off with answer = initial-value, then repeatedly sets answer to equal value of binary operation FUNCTION on answer and key value of next element and returns final value of answer.
If initial value is not provided:
- if set is empty, calls function once with zero arguments and returns value.
- Otherwise, starts off with answer being key value of first element.  Then repeatedly sets answer to equal value of applying  FUNCTION on current answer and key value of next element, and returns final value of answer.")
  (:method (fn s &key initial-value (key #'identity))
	   (if initial-value
	       (let ((acc initial-value))
		 (do-elements (x s acc)
		   (setf acc (funcall fn acc (funcall key x)))))
	     (if (is-empty s)
		 (funcall fn)
	       (let (acc)
		 (do-elements (x s acc i)
		   (let ((y (funcall key x)))
		     (setf acc (if (zerop i) y (funcall fn acc y))))))))))

(defgeneric sum-over (s fn)
  (:documentation "sum-over  SET FUNCTION.  Repeatedly apply FUNCTION to each member of SET and sum the results.")
  (:method (s fn)
	   (let ((total 0))
	     (do-elements (x s total)
	       (_f my+ total (funcall fn x)))))
  (:method ((s sequence) fn)
	   (reduce #'my+ s :initial-value 0 :key fn)))


  
  

(defgeneric subset (s1 s2)
  (:documentation "subset SET1 SET2.  Is SET1 a subset of SET2?")
  (:method ((s1 <numbered-set>) (s2 <numbered-set>))
	   (let ((size1 (size s1 t))
		 (size2 (size s2 t)))
	     (and (or (symbolp size1) (symbolp size2) (my<= size1 size2))
		  (call-next-method s1 s2))))
  (:method (s1 (s2 null))
    (is-empty s1))
  (:method (s1 s2)
	   (each s1 #'(lambda (x) (member? x s2)))))

	   

(defmethod iterator ((s t))
  (assert (typep s '[numbered-set]) nil
    "Can't iterate over ~a as it's not a [numbered-set]" s)
  (let ((ind 0)
	(size (size s)))
    (lambda ()
      (if (< ind size)
	  (let ((item (item ind s)))
	    (incf ind)
	    (iterator-not-done item))
	(iterator-done)))))

	  

(defmacro addf (s item &optional (pos nil))
  "Macro addf SET ITEM &optional (POS nil).  Add ITEM to SET if it doesn't already exist."
  `(_f add ,s ,item ,pos))





(defgeneric binary-intersection (s1 s2)
  (:documentation "binary-intersection S1 S2.  Return a set that is the intersection of S1 and S2.  Nondestructive."))


(def-symmetric-method binary-intersection ((s1 null) s2) (declare (ignore s2)) nil)
(def-symmetric-method binary-intersection ((s1 <numbered-set>) s2)
    (filter ':list s1
	    #'(lambda (x)
		(member? x s2))))   

(defaggregator intersect binary-intersection t)

(defgeneric intersects (s1 s2)
  (:documentation "intersects S1 S2.  Do S1 and S2 have nonempty intersection?")
  (:method (s1 s2) (not (is-empty (binary-intersection s1 s2)))))





(defgeneric binary-union (s1 s2)
  (:documentation "binary-union S1 S2.  Return a new set that is the union of S1 and S2.  Nondestructive."))

(def-symmetric-method binary-union ((s1 null) s2) s2)
(def-symmetric-method binary-union ((s1 (eql t)) s2) (declare (ignore s2)) t)
(defmethod binary-union (s1 s2)
  (implicit-union s1 s2))

(defaggregator unite binary-union nil)



(defgeneric set-eq (s1 s2)
  (:documentation "set-eq S1 S2.  Are S1 and S2 equal as sets?")
  (:method :around (s1 s2)
	   (or (eq s1 s2)
	       (call-next-method)))
  (:method ((s1 <numbered-set>) s2)
	   (and (or (eq (size s1) (size s2)) (eq (size s1) ':unknown) (eq (size s2) ':unknown))
		(do-elements (x s1 t)
		  (unless (member? x s2)
		    (return nil)))))
  (:method (s1 (s2 <numbered-set>)) (set-eq s2 s1))
  (:method (s1 s2) (and (subset s1 s2) (subset s2 s1))))

(def-symmetric-method set-eq ((s1 symbol) s2) (eq s1 s2))


(defgeneric symmetric-difference (s1 s2)
  (:documentation "symmetric-difference S1 S2.  Return 3 values : 1) the set of elements of S1 not in S2. 2) The set of elements of S2 not in S1 3) Elements in both sets")
  (:method (s1 s2)
	   (values
	    (filter ':list s1 (lambda (x) (not (member? x s2))))
	    (filter ':list s2 (lambda (x) (not (member? x s1))))
	    (intersect s1 s2))))
	   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; argmax and argmin
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun argext (s identity-element op key-fn)
  "argext SET IDENTITY-ELT OP KEY-FUNCTION
primitive operation for operations like argmax and argmin.  

SET - a set
IDENTITY-ELT - identity element of operation
OP - a function of two arguments
KEY-FN - how to get a value from each element of the set (usually #'identity)

Let E be the first element with the extreme value of key (i.e. its value VAL is such such that (op VAL OTHER-VAL) is true for all values OTHER-VAL of elements in the set).
argext returns
1) The index of E
2) The key value
3) The element
4) List of all indices having this value"


  (let ((best nil)
	(best-val identity-element)
	(all-best nil)
	(best-elt nil))
    (flet ((new-best (i val e)
	     (setf best i
		   best-val val
		   best-elt e
		   all-best (list i))))

      (do-elements (x s (values best best-val best-elt all-best) i)
	(let ((val (funcall key-fn x)))
	  (if best
	      (cond
	       ((eql val best-val) (push i all-best))
	       ((funcall op val best-val) (new-best i val x)))
	    (new-best i val x)))))))

(defun argmax (s &key (key #'identity) (tiebreak-keys nil))
  "argmax NUMBERED-SET &key (KEY #'identity) (TIEBREAK-KEYS nil).  Return four values : 1) index of first elt with max key val 2) the val 3) the first element achieving this max 4) list of all indices with this val.  The keys may be real numbers, '-infty, or '-infty."
  (if tiebreak-keys
      (argext s '-infty
	      #'(lambda (x y)
		  (loop
		      for a in x
		      for b in y
		      when (my> a b)
		      return t
		      when (my< a b)
		      return nil))
	      #'(lambda (x)
		  (cons (funcall key x) (mapcar #'(lambda (f) (funcall f x)) tiebreak-keys))))
			     
    (argext s '-infty #'my> key)))

(defun argmin (s &key (key #'identity))
  "argmin NUMBERED-SET &key (KEY #'identity).  Return three values : 1) index of first elt with min key val 2) the key val 3) the first element achieving this min 4) list of all indices with this val.  The keys may be real numbers, '-infty, or '-infty."
  (argext s 'infty #'my< key))

(defun maximizing-element (s &optional (key #'identity))
  "maximizing-element S &optional (KEY #'identity).  Return just the maximizing item from the call to argmax."
  (mvbind (ind val item) (argmax s :key key)
    (declare (ignore val))
    (assert ind nil "Couldn't find maximizing element of ~a, probably because the set is empty." s)
    item))

(defun minimizing-element (s &optional (key #'identity))
  "minimizing-element S &optional (KEY #'identity).  Return just the minimizing item from the call to argmin."
  (mvbind (ind val item) (argmin s :key key)
    (declare (ignore val))
    (assert ind nil "Couldn't find minimizing element of ~a, probably because the set is empty." s)
    item))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; print functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((s <set>) str)
  (print-unreadable-object (s str :type t :identity nil)
    (aif (name s)
	(write-string it str))))


(defun pprint-set (&rest args)
  "pprint-set STREAM SET 
or 
pprint-set SET (where STREAM defaults to t)"
  (condlet
   (((= (length args) 1) (str t) (s (first args)))
    ((> (length args) 1) (str (first args)) (s (second args))))
   (pprint-logical-block (str nil :prefix "{" :suffix "}")
     (do-elements (x s nil i)
       (unless (zerop i)
	 (format str ", "))
       (pprint-newline :fill str)
       ;; (pprint-pop)
       (write x :stream str)
       ))))
      

; (set-pprint-dispatch '(and <numbered-set> (not <image-set>)) #'pprint-set)
  
  

(in-package common-lisp-user)
	







