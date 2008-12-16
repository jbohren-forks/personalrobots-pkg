(in-package utils)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hash tables with general hash functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct gen-hash-table
  table
  hash-fn)
  

(defun make-hash-table* (&rest args &key (hash-fn #'identity hsupp) (test #'eql t-supp))
  "make-hash-table* &rest ARGS
Portable version of hash tables with nonstandard hash functions

ARGS are the arguments to hash table, with two additional keywords.

HASH-FN : a function of one argument that returns an object.  Defaults to #'identity.
TEST: a function of two arguments that must be one of #'equal, #'equalp, #'eql, or #'eq.  Defaults to #'eql.

The assumption will be that given two objects X and Y that are 'the same', their images under HASH-FN are equal w.r.t TEST."
  
  (assert (and args (is-standard-equality-test test)))
  (flet ((remove-if-necessary (args supp key)
	   (if supp
	       (aif (position key args)
		   (if (zerop it)
		       (cddr args)
		     (progn (setf (cdr (nthcdr (1- it) args)) (nthcdr (+ it 2) args)) args))
		 args)
	     args)))
	   
    (let ((actual-args (copy-list args)))
      (setf actual-args (remove-if-necessary actual-args hsupp ':hash-fn)
	    actual-args (remove-if-necessary actual-args t-supp ':test))
      (make-gen-hash-table :table (apply #'make-hash-table :test test actual-args) :hash-fn hash-fn))))

(defgeneric gethash* (key ht)
  (:documentation "gethash* KEY TABLE
To be used with tables created with make-hash-table*.  Also works for regular hash tables.")
  (:method (key (ht hash-table)) (gethash key ht))
  (:method (key (ht gen-hash-table))
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (let ((p (gethash (funcall hash-fn key) table)))
	       (if p
		   (values (cdr p) t)
		 (values nil nil))))))


(defgeneric sethash* (key ht val)
  (:method (key (ht hash-table) val)
	   (setf (gethash key ht) val))
  (:method (key (ht gen-hash-table) val)
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (let* ((k (funcall hash-fn key))
		    (p (gethash k table)))
	       (if p
		   (setf (cdr p) val)
		 (setf (gethash k table) (cons key val)))
	       val))))

(defgeneric hash-table-test* (ht)
  (:documentation "hash-table-test* GEN-HASH-TABLE.  Works for generalized hash tables.  Also, unlike hash-table-test, returns a function rather than a symbol.")
  (:method ((ht hash-table)) (symbol-function (hash-table-test ht)))
  (:method ((ht gen-hash-table)) (with-struct (gen-hash-table- table hash-fn) ht
				   (let ((test (symbol-function (hash-table-test table))))
				     #'(lambda (x y) (funcall test (funcall hash-fn x) (funcall hash-fn y)))))))
  
(defgeneric hash-table-count* (ht)
  (:method ((ht hash-table))
	   (hash-table-count ht))
  (:method ((ht gen-hash-table))
	   (hash-table-count (gen-hash-table-table ht))))

(defgeneric remhash* (key ht)
  (:method (key (ht hash-table))
	   (remhash key ht))
  (:method (key (ht gen-hash-table))
	   (with-struct (gen-hash-table- table hash-fn) ht
	     (remhash (funcall hash-fn key) table))))

(defgeneric hash-table-has-key (h k)
  (:documentation "hash-table-has-key HASHTABLE X.  Does the HASHTABLE contain (an item that satisfies the table's test together with) X?  Works with generalized hash tables also.")
  (:method ((h hash-table) x)
	   (mvbind (y pres)
	       (gethash x h)
	     (declare (ignore y))
	     pres))
  (:method ((h gen-hash-table) x)
	   (hash-table-has-key (gen-hash-table-table h) (funcall (gen-hash-table-hash-fn h) x))))

(defsetf gethash* sethash*)

(defgeneric hash-keys (h)
  (:documentation "hash-keys HASHTABLE.  Return list of keys in some order.  Works for generalized hash tables.")
  (:method ((h hash-table))
	   (loop for k being each hash-key in h collecting k))
  (:method ((h gen-hash-table))
	   (loop for v being each hash-value in (gen-hash-table-table h)
	       collect (car v))))

(defgeneric maphash* (fn h)
  (:documentation "maphash* FN H.  Like maphash* but works for generalized hash tables.")
  (:method (fn (h hash-table))
	   (maphash fn h))
  (:method (fn (h gen-hash-table))
	   (loop
	       for v being each hash-value in (gen-hash-table-table h)
	       do (funcall fn (car v) (cdr v)))))


(defmacro do-hash ((k v h) &rest body)
  `(maphash* #'(lambda (,k ,v) (declare (ignorable ,k ,v)) ,@body) ,h))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Miscellaneous
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun eql-hash-table (h1 h2)
  "eql-hashtable H1 H2.  Return t iff two hashtables have the eql values associated to each key.  Note that a key that maps to the value nil is considered to be the same as a nonexistent key."
  (let* ((keylist nil))
    (maphash (lambda (k v) (declare (ignore v)) (setf keylist (cons k keylist))) h1)
    (maphash (lambda (k v) (declare (ignore v)) (setf keylist (cons k keylist))) h2)
    (loop
	for k in keylist

	do (if (not (eql (gethash k h1) (gethash k h2)))
	     (return nil))
	
	finally (return t))))



(defun alist-to-hash (l &key (test #'eql))
  "alist-to-hash L &key (TEST #'eql).  Convert an alist to a hashtable.  No cloning happens."
  (let ((h (make-hash-table :test test)))
    (dolist (x l h)
      (setf (gethash (car x) h) (cdr x)))))


(defun hash-to-alist (h)
  "hash-to-alist H.  Convert a hashtable to an alist.  No cloning done."
  (let ((l nil))
    (maphash 
     #'(lambda (k v) (push (cons k v) l)) 
     h)
    l))

(defun print-hash-table-readably (h str)
  "print-hash-table-readably HASHTABLE STREAM.  Print the hashtable to the stream readably, i.e., such that using read (with the standard readtable) on that stream will yield a hashtable that is the same as HASHTABLE, under the assumption that the print-object methods for the keys and values in HASHTABLE respect the *print-readably* variable.

The hashtable test is also preserved.  Other optimization features such as the rehash-threshold, etc. may not.  The printing is done by converting the table to an alist.

If STREAM is nil, a string containing the output is returned.  Otherwise, nil is returned and the output is performed as a side-effect."

  (with-standard-io-syntax
    (let (
	#+allegro (excl:*print-nickname* t)
         )
        
	(format str "#.(utils:alist-to-hash '")
	(write (hash-to-alist h) :stream str)
	(format str " :test #'~a)"
		(ecase (hash-table-test h)
		  (eq "eq")
		  (eql "eql")
		  (equal "equal")
		  (equalp "equalp"))))))

(defun hash-table-select (h test)
  "hash-table-select H TEST returns list of keys of items in H whose value satisfies TEST"
  (loop
      for k being each hash-key in h using (hash-value v)
      if (funcall test v)
      collect k))

(defun copy-hash-table (h &key (value-copy-fn nil))
  "copy-hash-table H &key (VALUE-COPY-FN nil).  Create a new hash table of same type as H.  The keys are copied over and the values have VALUE-COPY-FN applied to them first, if it is provided."
  (let ((h2 (make-hash-table :test (hash-table-test h) :size (hash-table-count h))))
    (maphash
     (if value-copy-fn
	 #'(lambda (k v)
	     (setf (gethash k h2) (funcall value-copy-fn v)))
       #'(lambda (k v) (setf (gethash k h2) v)))
     h)
    h2))   


(defun pprint-hash (&rest args)
  (bind-pprint-args (s h) args
    (etypecase h
      (gen-hash-table (pprint-hash s (gen-hash-table-table h)))
      (hash-table
       (pprint-logical-block (s (hash-keys h)
				:prefix "["
				:suffix 
				(let ((not-shown 
				       (if *print-length*
					   (max 0 (- (hash-table-count h) *print-length*))
					 0)))
				  (if (> not-shown 0)
				      (format nil " and ~R other item~:*~P not shown here.]" not-shown)
				    "]")))

	 (loop
	   (let ((k (pprint-pop)))
	     (format s "~a : ~a" k (gethash k h))
	     (pprint-exit-if-list-exhausted)
	     (pprint-newline :mandatory s))))
       (values)))))

