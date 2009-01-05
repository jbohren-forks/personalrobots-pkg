;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; data-struct/set/create-sets.lisp
;; Ways of creating new sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package set)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Image of a set under a function
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <image-set> (<numbered-set>)
  ((f :initarg :f
      :reader f)
   (f-inv :initarg :f-inv
	  :reader f-inv
	  :initform nil)
   (s :initarg :s
      :accessor s
      :reader base-set
      :type [set])))

(defun make-image-set (s f &optional (f-inv nil))
  "make-image-set SET FUNCTION &optional (F-INV nil).  Returns a new set which is the image of SET under FUNCTION.  The new set will not be stored in memory; rather, its members will be computed as needed by applying FUNCTION to elements of SET.  F-INV, if provided, must be the bijective inverse of FUNCTION.  It is optional, but some operations, such as member? and item-number, may be slower if it is not provided (iterator, item, and size will be as fast, though).

NOTE: The iterator, size, and item-number methods, and anything else that's based on them (e.g. do-elements) only work correctly when f is 1-1."
  (make-instance '<image-set> :s s :f f :f-inv f-inv))

(setf (documentation 'base-set 'function)
  "base-set IMAGE-SET.  Return the original set that this IMAGE-SET is is based on.")

(defmethod member? (item (is <image-set>))
  (let ((s (s is))
	(test (equality-test is)))
    (aif (f-inv is)   
	 (let ((x (funcall it item)))
	   (and (member? x s)
		(funcall test item (funcall (f is) x))))
      (do-elements (x is nil)
	(when (funcall test x item)
	  (return t))))))
  
(defmethod iterator ((is <image-set>))
  (let ((iter (iterator (s is)))
	(f (f is)))
    (lambda ()
      (multiple-value-bind (item done?)
	  (funcall iter)
	(if done?
	    (values nil t)
	  (values (funcall f item) nil))))))

(defmethod item-number (item (is <image-set>))
  (let ((s (s is))
	(test (equality-test is)))
    (aif (f-inv is)
	(item-number (funcall (f-inv is) item) (s is))
      (progn
	(do-elements (x s nil i)
	  (when (funcall test x item)
	    (return-from item-number i)))
	(error 'item-not-in-set :item item :set is)))))

(defmethod item (num (is <image-set>))
  (funcall (f is) (item num (s is))))

(defmethod size ((is <image-set>) &optional (constant-time nil))
  (size (s is) constant-time))

(defmethod print-object ((s <image-set>) str)
  (print-unreadable-object (s str :type nil :identity nil) 
    (format str "Image of ~w" (base-set s))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; filtered set - "all X that satisfy F"
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <filtered-set> (<set>)
  ((base-set :initarg :base-set :accessor base-set :initform t)
   (predicate :initarg :predicate :accessor predicate)
   (cached-size :accessor cached-size  :initform ':unknown))
  (:documentation "Class <filtered-set> (<set>)

Initargs
:base-set - a set.  Defaults to t
:predicate - a predicate (boolean function) of one argument
:store-cached-size - cache size upon creation?  Defaults to nil.

Implicitly represents the set of elements of base-set that satisfy predicate.  The operations defined for filtered-sets are member? and iterator (and derived operations such as do-elements and mapset).  Note that a call to the iterator could run forever if the base set is infinite and there are infinitely items that don't satisfy the predicate.

Can create using constructor, or the functions filter or objects-satisfying."))

(defmethod initialize-instance :after ((s <filtered-set>) &rest args &key (store-cached-size nil))
  (declare (ignore args))
  (when store-cached-size
    (setf (cached-size s)
      (sum-over s (constantly 1)))))

(defun objects-satisfying (pred)
  (make-instance '<filtered-set> :predicate pred))

(defmethod member? (x (s <filtered-set>))
  (and (member? x (base-set s)) (funcall (predicate s) x)))

(defmethod iterator ((s <filtered-set>))
  (let ((iter (iterator (base-set s)))
	(pred (predicate s)))
    #'(lambda ()
	(loop
	  (mvbind (item done?) (funcall iter)
	    (if done?
		(return (iterator-done))
	      (when (funcall pred item)
		(return (iterator-not-done item)))))))))

(defmethod size ((s <filtered-set>) &optional (constant-time nil))
  (let ((cs (cached-size s)))
    (if (or (numberp cs) constant-time (not (numberp (size (base-set s)))))
	cs
      (sum-over s (constantly 1)))))

(defmethod vdc-sequence ((s <filtered-set>) &optional (constant-space nil) (base 0))
  (let ((f (vdc-sequence (base-set s) constant-space base))
	(pred (predicate s)))
    #'(lambda ()
	(loop
	   (let ((x (funcall f)))
	     (when (funcall pred x)
	       (return x)))))))


	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; powerset of a set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <powerset> (<set>)
  ((base-set :initarg :base-set :reader base-set)
   (iteration-order :initarg :iteration-order :reader iteration-order :initform ':default))
  (:documentation "Class <powerset>

:base-set
:iteration-order - optional.  The default is to iterate by 'counting in binary'.  Can also be ':increasing-size."))
  

(defun powerset (s)
  "powerset S.  Return the set of all subsets of S.  The powerset is not stored explicitly.  The only supported operations are size,  member?, clone, and print-object and iteration, which represents subsets as lists."
  (make-instance '<powerset> :base-set s))



(defmethod size ((s <powerset>) &optional (constant-time nil))
  (let ((s1 (size (base-set s) constant-time)))
    (if (symbolp s1) s1 (expt 2 s1))))
      


(defmethod iterator ((s <powerset>) &aux (b (base-set s)) (n (size b)))
  (ecase (iteration-order s)
    (:default  (let ((a (make-array n :element-type 'bit :initial-element 0)))
		 #'(lambda ()
		     (if a
			 (prog1
			     (iterator-not-done 
			      (let ((l nil))
				(do-elements (x b l i)
				  (unless (zerop (bit a i) )
				    (push x l)))))
			   (aif (position 0 a)
				(dotimes (j (1+ it))
				  (setf (bit a j) (- 1 (bit a j))))
				(setf a nil)))
		       (iterator-done)))))
    (:increasing-size (iterator (disjoint-union-of-sets (mapset 'list #'(lambda (k) (make-instance '<subsets-of-size> :base-set b :size k)) (1+ n)))))))
		      
	      
(defmethod member? (x (s <powerset>))
  (subset x (base-set s)))

(defmethod clone ((s <powerset>))
  s)

(defmethod print-object ((s <powerset>) str)
  (if *print-readably*
      (format str "#.(powerset ~W)" (base-set s))
    (print-unreadable-object (s str :type t)
      (format str "of ~W" (base-set s)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; implicit and disjoint union
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <implicit-union> (<set>)
  ((sets :initarg :sets :accessor sets :reader union-sets))
  (:documentation "Represents an implicit union of sets.

Iteration is slow especially for sets with nonstandard equality tests since we have to maintain a table of previously seen items.

Initargs
:sets"))

(defmethod initialize-instance :after ((s <implicit-union>) &rest args &key sets)
  (declare (ignore args))
  (unless (is-empty sets)
    (set-equality-test (equality-test (item 0 sets)) s)))

(defmethod iterator ((s <implicit-union>) &aux (test (equality-test s)))
  (let ((already-seen (if (is-standard-equality-test test) (make-hash-table :test #'equal) nil))
	(iter (iterator (sets s)))
	(set-iter nil))
    #'(lambda ()
	(loop
	   unless set-iter
	   do (mvbind (s done) (funcall iter)
		(if done
		    (return (iterator-done))
		    (setq set-iter (iterator s))))
	     
	   do (mvbind (item done) (funcall set-iter)
		(if done
		    (setq set-iter nil)
		    (etypecase already-seen
		      (hash-table (unless (hash-table-has-key already-seen item)
				    (setf (gethash item already-seen) t)
				    (return (iterator-not-done item))))
		      (list (unless (assoc item already-seen :test test)
			      (push (cons item t) already-seen)
			      (return (iterator-not-done item)))))))))))
	
      

(defun implicit-union (&rest sets)
  "implicit-union &rest SETS"
  (make-instance '<implicit-union> :sets sets))

(defmethod member? (x (us <implicit-union>))
  (any (sets us) #'(lambda (s) (member? x s))))

(defmethod subset ((s1 <implicit-union>) s2)
  (each (sets s1) #'(lambda (s) (subset s s2))))


(defmethod is-empty ((s <implicit-union>))
  (each (sets s) #'is-empty))

(def-symmetric-method intersects ((us <implicit-union>) s2)
  (any (sets us) #'(lambda (s) (intersects s s2))))

(def-symmetric-method binary-intersection ((s1 <implicit-union>) s2)
  (make-instance '<implicit-union> :sets (mapset 'list #'(lambda (s) (binary-intersection s s2)) (sets s1))))



(defclass <disjoint-union> (<implicit-union> <numbered-set>)
  ((sets :initarg :sets :accessor sets)
   )
  (:documentation
   "Represents a disjoint union of sets.  Rather than explicitly list the members of all the sets, just holds references to the underlying sets themselves.  The disjointness assumption is needed to do iteration and numbering efficiently.  The underlying sets should not be modified once this object is created.

Initargs
:sets - set of <numbered-sets>"))

(defun disjoint-union (&rest sets)
  "disjoint-union &rest SETS. The most efficient union operation, assuming SETS are disjoint.  The created set is space efficient since it maintains reference to the original sets.  So they should not be modified once this is created."
  (make-instance '<disjoint-union> :sets sets))

(defun disjoint-union-list (l)
  "disjoint-union-list SETS.  Like disjoint-union, except takes a single list argument.  Deprecated, as it has been replaced by disjoint-union-of-sets."
  (make-instance '<disjoint-union> :sets l))

(defun disjoint-union-of-sets (s)
  "disjoint-union-of-sets SETS.  Disjoint union of the sets (see disjoint-union)."
  (make-instance '<disjoint-union> :sets s))


(defmethod item (n (ds <disjoint-union>))
  (if (eq (size ds) ':unknown)
      (call-next-method)
    (do-elements (s (sets ds))
      (let ((m (size s)))
	(when (< n m)
	  (return (item n s)))
	(decf n m)))))

(defmethod item-number (x (ds <disjoint-union>))
  (let ((m 0))
    (do-elements (s (sets ds))
      (when (member? x s)
	(return (+ m (item-number x s))))
      (incf m (size s)))))

(defmethod size ((ds <disjoint-union>) &optional (constant-time nil))
  (let ((sizes (mapset 'list #'(lambda (s) (size s constant-time)) (sets ds))))
    (if (member 'infty sizes)
	'infty
      (if (member ':unknown sizes)
	  ':unknown
	(reduce #'+ sizes)))))

(defmethod iterator ((ds <disjoint-union>))
  (let ((set-iter (iterator (sets ds)))
	(current-iter nil))
    #'(lambda ()
	(block iterator
	  (flet ((get-next-iterator ()
		   (mvbind (next-set done?) (funcall set-iter)
		     (if done? (return-from iterator (iterator-done)) (iterator next-set)))))
	    (orf current-iter (get-next-iterator))
	    (loop
	      (mvbind (item done?) (funcall current-iter)
		(unless done? (return (iterator-not-done item)))
		(setf current-iter (get-next-iterator)))))))))



(defun intersect-ds (ds s)
  (apply #'disjoint-union 
	 (mapset 'list #'(lambda (comp) (intersect comp s)) (sets ds))))

(defmethod binary-intersection ((ds <disjoint-union>) s)
  (intersect-ds ds s))

(defmethod binary-intersection (s (ds <disjoint-union>))
  (intersect-ds ds s))

(defmethod print-object ((s <disjoint-union>) str)
  (print-unreadable-object (s str :type nil :identity nil)
    (format str "Union of ~w" (sets s))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; implicit intersection
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <implicit-intersection> (<set>)
  ((sets :initarg :sets :accessor sets))
  (:documentation "<implicit-intersection>

Initargs
:sets - set of <set> objects"))

(defun implicit-intersection (&rest sets)
  (make-instance '<implicit-intersection> :sets sets))

(defmethod member? (x (s <implicit-intersection>))
  (each (sets s) #'(lambda (comp) (member? x comp))))
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; subsets of size k
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun subsets-of-size (s k)
  "subsets-of-size SET K.  Return set of subsets of size K, all represented as lists.  Represents extensionally, so takes |S|^K time and space.  See also <subsets-of-size>."
  (if (zerop k)
      '(())
    (if (> k (size s))
	nil
      (mapcon
       #'(lambda (l)
	   (dsbind (initial . remaining) l
	     (mapcar #'(lambda (l2) (cons initial l2))
		     (subsets-of-size remaining (1- k)))))
       (if (listp s) s (mapset 'list #'identity s))))))


(defclass <subsets-of-size> (<numbered-set>)
  ((base-set :initarg :base-set :accessor base-set)
   (k :initarg :size :accessor k))
  (:documentation "<subsets-of-size>

Initargs
:base-set - a <numbered-set>
:size

Implicit representation of the set of all size-k subsets of the base-set.  The subsets are represented as lists.  See also subsets-of-size function."))


(defmethod iterator ((s <subsets-of-size>) &aux (base-set (base-set s)) (k (k s)) (n (size base-set)))
  
  ;; Maintain a stack of items (L I M) denoting that items in list L have already been chosen
  ;; and we still need to pick I items from positions M,...,N-1
  (let ((stack (list (list nil k 0))))
    
    #'(lambda ()
	(loop
	  (if stack

	      (let ((item (pop stack)))
		(dbind (l i m) item
		  (if (zerop i)
		      
		      ;; If I = 0, we have constructed a subset of size K, so return it
		      (return (iterator-not-done l))
		    
		    ;; Otherwise, add all possible ways of picking the next item to the stack
		    (for-loop (next (- n i) (1- m) -1 #'=)
		      (push (list (cons (item next base-set) l) (1- i) (1+ next)) stack)))))
	    
	    ;; If stack is empty, iteration is done
	    (return (iterator-done)))))))
		      
		
		
	    
  

		
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;  permutation
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun permutation (s a)
  "permutation SET A a-inv.  A is an array of size N where set has size N, and represent a permutation of 0:n-1.  Element 0 of the permuted set is element A(0), and so on."
  (make-image-set
   (size s) #'(lambda (i) (item (aref a i) s))))
   

(defun random-cycle (s)
  "return a random cyclic permutation of set s"
  (let ((n (size s)))
    (let ((k (random n))
	  (a (make-array n :element-type 'fixnum)))
      (let ((m (- n k)))
	(dotimes (i m)
	  (setf (aref a i) (+ i k)))
	(for-loop (i m n)
		  (setf (aref a i) (- i m))))
      (permutation s a))))

(defun random-permutation (s)
  "Return a random permutation of set S.  Uses the Knuth shuffle."
  (let* ((n (size s))
	 (a (make-array n :element-type 'fixnum)))
    (dotimes (i n)
      (setf (aref a i) i))
    (dotimes (i (1- n))
      (let ((j (random (- n i))))
	(psetf (aref a i) (aref a (+ j i))
	       (aref a (+ j i)) (aref a i))))
    (permutation s a)))
    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; nondeterministic-choice sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro ndlet (bindings &body body)
  "ndlet ((VAR1 SET1) ... (VAR_n SET_n)) &body BODY
Looks like a let, except the rhs of each binding is a set.  Expands to a form that returns the set of all items obtained by choosing a value for each variable from its set, then evaluating the body.  The code assumes that no two bindings can result in the same element.

Based on make-image-set, but it doesn't specify an inverse function.  Thus, if certain operations will be called a lot (member?, item-number), it may be better to directly use make-image-set.

The set is represented implicitly, and asking for the Nth element requires generating the first N elements each time.

See also ndlet-fail and ndunion.
"
  (if (= 1 (length bindings))
      (let ((var (caar bindings)))
	`(make-image-set ,(cadar bindings) #'(lambda (,var) ,@body)))
    (with-gensyms (x)
      `(make-image-set 
	(direct-product 'list ,@(mapcar #'second bindings))
	#'(lambda (,x)
	    (dsbind ,(mapcar #'first bindings) ,x
	      ,@body))))))

(defmacro ndunion (bindings &body body)
  "ndunion ((VAR1 SET1) ... (VAR_n SET_n)) &body BODY
Like ndlet, except that the BODY must now evaluate to a set (rather than an element) for each choice of bindings, and the ndunion form returns the union of these sets.  Assumes the sets corresponding to different bindings are disjoint.

Based on make-image-set, but it doesn't specify an inverse function.  Thus, if certain operations will be called a lot (member?, item-number), it may be better to directly use make-image-set together with disjoint-union.

The set is represented implicitly, and asking for the Nth element requires generating the first N elements each time.

See also ndlet and ndlet-fail.
"
  (assert (eq (length bindings) 1) nil "ndunion only implemented at the moment for a single binding.")
  (let ((var (caar bindings)))
    `(make-instance '<disjoint-union>
       :sets (mapset 'list  #'(lambda (,var) ,@body) ,(cadar bindings)))))

(defmacro ndlet-fail (bindings &body body)
  "Like ndlet, with the extension that in the particular case when the body evaluates to the symbol 'fail (in the set package), the element is not included in the set.

The set is represented implicitly, and asking for the Nth element requires generating the first N elements each time.

See also ndlet and ndunion.
"
  `(filter ':implicit (ndlet ,bindings ,@body) #'(lambda (x) (not (eq x 'fail)))))


(defmacro ndlet* (bindings &body body)
  (labels ((ndlet-helper (bindings)
	     (cond
	      ((null bindings) body)
	      ((cdr bindings) `(ndunion ,(list (car bindings)) ,(ndlet-helper (cdr bindings))))
	      (t `(ndlet ,bindings ,@body)))))
    (if bindings
	(ndlet-helper bindings)
      (cons 'progn body))))
     



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; permutations of finite sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun int-permutations (n &key (iterate-quickly nil))
  (if (zerop n)
      (list #())
    (let ((a (when iterate-quickly (mapset 'vector #'identity n))))
      (ndlet ((x (make-instance '<prod-set> :sets (let ((b (make-array (1- n) :element-type 'fixnum))) (loop for i from n downto 2 for j from 0 do (setf (aref b j) i) finally (return b))) 
				:inst-acc (inst-vars:make-vec-accessors (1- n))
				:iterate-quickly t)))
	(if iterate-quickly 
	    (dotimes (i n) (setf (aref a i) i))
	  (setf a (mapset 'vector #'identity n)))
	(do-elements (y x a i)
	  (rotatef (aref a i) (aref a (+ i y))))))))
    
    
    

(defun permutations (s &key (iterate-quickly nil))
  "permutations SET (ITERATE-QUICKLY nil)
Set of permutations of finite set SET."
  (let ((size (size s)))
    (check-type size integer)
    (ndlet ((perm (int-permutations size :iterate-quickly iterate-quickly)))
      (permutation s perm))))



(defun bijections (s1 s2 &key (iterate-quickly nil))
  "bijections S1 S2 &key (ITERATE-QUICKLY nil).  Return set of bijections from S1 to S2.  Bijections are currently represented as functions, but more options may be added later."
  (let ((n1 (size s1))
	(n2 (size s2)))
    (check-type n1 integer)
    (check-type n2 integer)
    (when (eq n1 n2)
      (ndlet ((perm (int-permutations n1 :iterate-quickly iterate-quickly)))
	#'(lambda (x)
	    (item (aref perm (item-number x s1)) s2))))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; complements
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <complement-set> (<set>)
  ((base-set :accessor base-set :initform t :initarg :base-set)
   (s :accessor s :initarg :s))
  (:documentation "Complement of a set represented implicitly.  Create using complement-set."))

(defun complement-set (s &key (result-type ':implicit) (base-set t))
  "complement-set S &key (RESULT-TYPE ':implicit) (BASE-SET t) 
RESULT-TYPE can be ':implicit or ':list"
  (if (and (eq base-set t) (member s '(t nil)))
      (not s)
    (ecase result-type
      (:implicit (make-instance '<complement-set> :base-set base-set :s s))
      (:list (filter ':list base-set #'(lambda (x) (not (member? x s))))))))

(defmethod member? (x (s <complement-set>))
  (and (member? x (base-set s))
       (not (member? x (s s)))))

(defmethod iterator ((cs <complement-set>))
  (let ((iter (iterator (base-set cs)))
	(s (s cs)))
    #'(lambda ()
	(loop
	  (mvbind (item done?) (funcall iter)
	    (if done?
		(return (iterator-done))
	      (unless (member? item s)
		(return (iterator-not-done item)))))))))
	      
	   

(defmethod clone ((s <complement-set>))
  (make-instance '<complement-set> :base-set (clone (base-set s)) :s (clone (s s))))


(defmethod print-object ((s <complement-set>) str)
  (print-unreadable-object (s str :type nil :identity nil)
    (format str "Complement of ~a" (s s))
    (let ((base-set (base-set s)))
      (unless (eq base-set t)
	(format str " in ~a" (base-set s))))))

(defmethod set-eq ((s <complement-set>) (s2 <complement-set>))
  (and (set-eq (base-set s) (base-set s2))
       (set-eq (s s) (s s2))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; integer ranges
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <int-range> (<numbered-set>)
  ((low :accessor low :initarg :low)
   (high :accessor high :initarg :high)))
  

(defun int-range (low high)
  "int-range LOW HIGH.  Return an object representing the set of integers LOW, LOW+1,..., HIGH-1.  Could be empty if HIGH <= LOW."
  (make-instance '<int-range> :low low :high high))

(defmethod member? (item (x <int-range>))
  (and (typep item 'integer)
       (>= item (low x))
       (< item (high x))))

(defmethod clone ((s <int-range>))
  (make-instance '<int-range> :low (low s) :high (high s)))

(defmethod size ((s <int-range>) &optional (constant-time nil))
  (declare (ignore constant-time))
  (max (- (high s) (low s)) 0))

(defmethod item (n (s <int-range>))
  (+ (low s) n))

(defmethod item-number (x (s <int-range>))
  (if (member? x s)
      (- x (low s))
    (error 'item-not-in-set :item x :set s)))



