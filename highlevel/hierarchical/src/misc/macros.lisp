(in-package utils)


(defmacro with-gensyms (syms &body body)
  "set the value of each symbol in SYMS to a unique gensym"
  `(let ,(mapcar #'(lambda (s)
		     `(,s (gensym)))
	  syms)
     ,@body))


(defmacro eval-now (&body body)
  "macro EVAL-NOW.  Expands to (eval-when (:compile-toplevel :load-toplevel :execute) ,@body)"
  `(eval-when (:compile-toplevel :load-toplevel :execute) ,@body))

(defmacro with-outfile ((f filename) &body body)
  "with-outfile (VAR FILENAME) &body BODY.  Call body with VAR bound to a stream that writes to FILENAME.  A shorthand for with-open-file that does output and overwrites the file if it exists already."
  `(with-open-file (,f ,filename :direction :output :if-exists :supersede) ,@body))

(defun mappend (fn &rest lsts)
  (apply #'append (apply #'mapcar fn lsts)))

(defmacro while (test &rest body)
  "while TEST &rest BODY
A looping construct.  TEST is evaluated *before* each iteration, and if it is false, the construct terminates."
  `(loop
       while ,test
       do ,@ (or body '(()))))

(defmacro until (test &body body)
  "until TEST &rest BODY
Looping construct equivalent to while (not TEST) . BODY.  See also repeat-until."
  `(while (not ,test) ,@body))

(defmacro repeat-until (test &rest body)
  "repeat-until TEST &rest BODY
Looping construct.  TEST is evaluated *after* each iteration (unlike while and until), and if it is true, the construct terminates."
  `(loop
     ,@body
     (when ,test (return))))

(defmacro repeat (n &rest body)
  "repeat N &rest BODY.  Execute BODY N times."
  (let ((i (gensym)))
    `(dotimes (,i ,n)
       (declare (ignorable ,i))
       ,@body)))
     


(defmacro defvars (&rest args)
  "defvars &rest VAR-NAMES.  Define a set of variables without assigning a value."
  `(progn
     ,@(mapcar #'(lambda (v) `(defvar ,v)) args)))

(defmacro iwhen (test &rest body)
  "iwhen TEST &rest BODY.  If TEST evaluates to true, return result of evaluating BODY.  Otherwise, evaluates to 0.0.  See also iunless, indicator."
  `(if ,test
       (progn ,@body)
     0.0))

(defmacro iunless (test &rest body)
  "iunless TEST &rest BODY.  If TEST evaluates to false, evaluates BODY. Otherwise, just returns 0.0.  See also iwhen, indicator."
  `(if ,test
       0.0
     (progn ,@body)))

(defmacro for-loop ((var i j &optional (inc 1) (test '#'>=)) &body body)
  "for (VAR I J &optional (INC 1) (TEST #'>=)) &body BODY.

Repeatedly executes BODY with VAR bound to I,I+INC,...  The loop is stopped before executing the body when TEST returns true given VAR and J."
  (with-gensyms (last increment)
    `(let ((,last ,j)
	   (,increment ,inc))
       (do ((,var ,i (incf ,var ,increment)))
	   ((funcall ,test ,var ,last) nil)
	 ,@body))))



(defmacro rand-fn-hist (f n &key (inc (ceiling n 100)) (test '#'equal) (print nil))
  "rand-fn-hist F N &key INC TEST calls the form F N times, printing a . for progress every INC steps, and displays (and returns) the results as a hash-table using TEST (default #'equalp)"
  `(loop
       with val = nil
       with h = (make-hash-table :test ,test)
		
       for i from 1 to ,n
		  
       if (zerop (mod i ,inc))
       do (format t ".")
		  
       do (setf val ,f)
       do (setf (gethash val h)
	    (1+ (gethash val h 0)))
	 
       finally (when ,print
		 (fresh-line)
		 (pprint-hash h))
	       (return h)))


(defmacro with-pprint-dispatch ((type function &optional (priority 0) ) &body body)
  "with-pprint-dispatch (TYPE FUNCTION &optional (PRIORITY 0)) &body BODY

Makes a copy of the current pprint dispatch table, adds a pprint method FUNCTION for TYPE with the given priority, and evaluates BODY.  Afterwards, the dispatch table is restored to its old state."

  (let ((old-table (gensym)))
    `(let ((,old-table (copy-pprint-dispatch)))
       (unwind-protect 
	   (progn
	     (set-pprint-dispatch ,type ,function ,priority)
	     ,@body)
	 (setf *print-pprint-dispatch* ,old-table)))))

  
(defmacro bind-pprint-args ((str obj) args &body body)
  "bind-pprint-args (STR OBJ) ARGS &rest BODY

STR, OBJ : unevaluated symbols
ARGS : a list (evaluated)


If ARGS has length 1, bind STR to t, OBJ to (FIRST ARGS).  Otherwise, bind STR to (first args) and OBJ to (second args) ARGS.  Then evaluate BDOY in this lexical context."
  
  (let ((x (gensym)))
    `(let ((,x ,args))
       (condlet
	(((= 1 (length ,x)) (,str t) (,obj (first ,x)))
	 (t (,str (first ,x)) (,obj (second ,x))))
	,@body))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Symbols
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun build-symbol-name (&rest args)
  "build-symbol-name S1 ... Sn.  Each Si is a string or symbol.  Concatenate them into a single long string (which can then be turned into a symbol using intern or find-symbol."
  (apply #'concatenate 'string (mapcar (lambda (x) (if (symbolp x) (symbol-name x) x)) args)))

(defun intern-compound-symbol (&rest args)
  "intern-compound-symbol S1 ... Sn.  Interns the result of build-symbol-name applied to the S."
  (intern (apply #'build-symbol-name args)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; abbreviations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro abbrev (x y)
  (let ((args (gensym))
	(doc-string (format nil "~a : Abbreviation macro for ~a" x y)))

    `(defmacro ,x (&rest ,args)
       ,doc-string
       `(,',y ,@,args))))

(defmacro mvbind (vars form &body body)
  `(multiple-value-bind ,vars ,form ,@body))
(abbrev mvsetq multiple-value-setq)
(abbrev unbind-slot slot-makunbound)
(defmacro dsbind (pattern form &body body)
  `(destructuring-bind ,pattern ,form ,@body))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; destructuring
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun dbind-ex (binds body)
  (if (null binds)
      `(progn ,@body)
    `(let ,(mapcar #'(lambda (b)
		       (if (consp (car b))
			   (car b)
			 b))
	    binds)
       ,(dbind-ex (mapcan #'(lambda (b)
			      (if (consp (car b))
				  (cdr b)))
			  binds)
		  body))))


(defun destruc (pat seq &optional (atom? #'atom) (n 0))
  (labels ((true-list (x)
	     (or (null x)
		 (and (consp x)
		      (true-list (cdr x))))))
    (assert (true-list pat))
    (if (null pat)
	nil
      (let ((rest (cond ((funcall atom? pat) pat)
			((eq (car pat) '&rest) (cadr pat))
			((eq (car pat) '&body) (cadr pat))
			(t nil))))
	(if rest
	    `((,rest (subseq ,seq ,n)))
	  (let ((p (car pat))
		(rec (destruc (cdr pat) seq atom? (1+ n))))
	    (if (funcall atom? p)
		(cons `(,p (elt ,seq ,n)) rec)
	      (let ((var (gensym)))
		(cons (cons `(,var (elt ,seq ,n))
			    (destruc p var atom?))
		      rec)))))))))


(defmacro dbind (pat seq &body body)
  "macro dbind PAT SEQ BODY.  Like destructuring-bind except SEQ may not be any combination of lists and vectors.  The format of PAT is as before regardless.  See also dsbind."
    (let ((gseq (gensym)))
      `(let ((,gseq ,seq))
	 ,(dbind-ex (destruc pat gseq #'atom) body))))

(defmacro with-struct ((name &rest fields) s &body body)
  "with-struct (CONC-NAME . FIELDS) S &rest BODY

Example:
with-struct (foo- bar baz) s ...
is equivalent to
let ((bar (foo-bar s)) (baz (foo-baz s)))...

Note that despite the name, this is not like with-accessors or with-slots in that setf-ing bar above would not change the value of the corresponding slot in s."

  (let ((gs (gensym)))
    `(let ((,gs ,s))
       (let ,(mapcar #'(lambda (f)
			 `(,f (,(intern-compound-symbol name f) ,gs)))
	      fields)
	 ,@body))))
       

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; assertions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro verify-type (x typespec &optional (msg (format nil "~a" typespec)))
  "verify-type X TYPESPEC &optional MSG.   Shorthand for check-type X typespec msg followed by returning x."
  (let ((y (gensym)))
    `(let ((,y ,x))
       (check-type ,y ,typespec ,msg)
       ,y)))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; condlet
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro condlet (clauses &body body)
  "condlet CLAUSES &rest BODY.  CLAUSES is a list of which each member is a conditional binding or otherwise clause.  There can be at most one otherwise clause and it must be the last clause.  Each conditional binding is a list where the first element is a test and the remaining elements are the bindings to be made if the test succeeds.   Each clause must bind the same set of variables.  If one of the tests succeeds, the corresponding bindings are made, and the body evaluated.  If none of the tests suceeds, the otherwise clause, if any, is evaluated instead of the body."
  (labels ((condlet-clause (vars cl bodfn)
	     `(,(car cl) (let ,(condlet-binds vars cl)
			   (,bodfn ,@(mapcar #'cdr vars)))))
	   
	   (condlet-binds (vars cl)
	     (mapcar #'(lambda (bindform)
			 (if (consp bindform)
			     (cons (cdr (assoc (car bindform) vars))
				   (cdr bindform))))
		     (cdr cl))))
	   
    (let* ((var-names (mapcar #'car (cdr (first clauses))))
	   (otherwise-clause? (eql (caar (last clauses)) 'otherwise))
	   (actual-clauses (if otherwise-clause? (butlast clauses) clauses)))
      (assert (every (lambda (cl) (equal var-names (mapcar #'car (cdr cl))))
		     actual-clauses)
	  nil "All non-otherwise-clauses in condlet must have same variables.")
      (let ((bodfn (gensym))
	    (vars (mapcar (lambda (v) (cons v (gensym)))
			  var-names)))
	`(labels ((,bodfn ,(mapcar #'car vars)
		    ,@body))
	   (cond 
	    ,@(mapcar (lambda (cl) (condlet-clause vars cl bodfn))
		      actual-clauses)
	    ,@(when otherwise-clause? `((t (progn ,@(cdar (last clauses))))))))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; setf macros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro change-if-necessary (place new-val &optional (test '#'equal))
  "change-if-necessary PLACE NEW-VAL &optional (TEST #'equal).
If PLACE satisfies the test with NEW-VAL, do nothing and return nil.  Otherwise set it to the new-val and return it."
  (let ((v (gensym)))
    `(let ((,v ,new-val))
       (unless (funcall ,test ,place ,v)
	 (setf ,place ,v)))))
  

(defmacro _f (op place &rest args)
  "_f OP PLACE &rest ARGS.  A correct (i.e. multiple-evaluation-avoiding) version of (setf PLACE (apply OP PLACE ARGS))"
  (multiple-value-bind (vars forms var set access)
      (get-setf-expansion place)
    `(let* (,@(mapcar #'list vars forms)
	    (, (car var) (,op ,access ,@args)))
       ,set)))

(defun avg (x y weight)
  "avg X Y WEIGHT.  Assumes WEIGHT is between 0 and 1.  Returns WEIGHT*Y + (1-WEIGHT)*X.
X,Y can be extended reals."
  (my+ (my* y weight) (my* (- 1 weight) x)))

(defmacro avgf (place new-val weight)
  "avgf PLACE NEW-VAL W.  Replace PLACE with W*NEW-VAL + (1-W)*PLACE."
  `(_f avg ,place ,new-val ,weight))

(defmacro toggle (place)
  `(_f not ,place))

(defmacro adjustf (place &rest args)
  "adjustf PLACE &rest ARGS.  Applies adjust-array to PLACE with ARGS and stores the new array in PLACE."
  `(_f adjust-array ,place ,@args))

(defmacro multf (place &rest args)
  "multf PLACE &rest ARGS.  Multiplies PLACE by the ARGS and stores the result back in place."
  `(_f my* ,place ,@args))

(defmacro divf (place div)
  "divf PLACE DIV.  Divides PLACE by DIV and stores the result back in place."
  `(_f / ,place ,div))


(defmacro maxf (place &rest args)
  "maxf PLACE &rest ARGS.  Setf PLACE to (mymax PLACE . args)."
  `(_f mymax ,place ,@args))

(defmacro minf (place &rest args)
  "minf PLACE &rest ARGS.  Setf PLACE to (mymax place . args)."
  `(_f mymin ,place ,@args))

(defmacro orf (place &rest args)
  "orf PLACE &rest ARGS.  Setf PLACE to (or PLACE . ARGS)."
  `(_f or ,place ,@args))

(defun my-delete (place item &rest args)
  (apply #'delete item place args))

(defun my-adjoin (place item &rest args)
  (apply #'adjoin item place args))

(defmacro deletef (place item &rest args)
  "deletef PLACE ITEM &rest ARGS.  Setf place to (delete item place . args)"
  `(_f my-delete ,place ,item ,@args))
  
(defmacro adjoinf (place item &rest args)
  "adjoinf PLACE ITEM &rest ARGS.  Setf place to (adjoin ITEM PLACE . ARGS)"
  `(_f my-adjoin ,place ,item ,@args))


(defmacro set-unless-key-exists (h k v)
  "macro set-unless-key-exists H K V
H and K are evaluated.  If hashtable H has an entry for key K, then that value is returned.  Otherwise, V is evaluated and an entry with key K and value V is added to H, and V is returned.

Since V is only evaluated if necessary, this macro is convenient for tables that are used to cache results of expensive computations."
  (with-gensyms (h2 k2 val pres)
    `(let ((,h2 ,h)
	   (,k2 ,k))
       (mvbind (,val ,pres) (gethash ,k2 ,h2)
	 (if ,pres ,val (setf (gethash ,k2 ,h2) ,v))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Memoization
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro define-memoized-reader (fname reader writer computer)
  "define-memoized-reader FNAME READER WRITER COMPUTER

FNAME (unevaluated): a symbol
READER, WRITER, COMPUTER (evaluated): functions that are defined in the context of the call to define-memoized-reader

Expands to code that creates a function FNAME that works as follows, given arguments ARGS:
1) Apply reader to ARGS
2) If value is not ':unassigned, return it
3) Else, apply computer to ARGS, and let V be the return value
4) Return the value of applying writer to V . ARGS"
  (let ((x (gensym))
	(args (gensym)))
    `(defun ,fname (&rest ,args)
       (let ((,x (apply ,reader ,args)))
	 (if (eq ,x ':unassigned)
	     (apply ,writer (apply ,computer ,args) ,args)
	   ,x)))))

(defmacro compute-memoized ((reader &rest a) &rest args)
  "compute-memoized (READER WRITER COMPUTER) &rest ARGS

READER, WRITER, COMPUTER: unevaluated symbols that name lexically available functions.

ARGS - argument list (each one evaluated once).

Expands to code that works as follows:
Apply READER to the args.  
If it returns ':unassigned, call COMPUTER on the args, and let V be the return value.  Call writer on V . ARGS and return V.
Else, return the value of READER.

As one special case, WRITER and COMPUTER may be left out if their symbol names are obtained by prefixing set- and compute- respectively to the symbol name of READER."

  (condlet ((a (writer (first a)) (computer (second a)))
	    (t (writer (intern-compound-symbol "SET-" reader)) (computer (intern-compound-symbol "COMPUTE-" reader))))
	   (with-gensyms (arg-list v)
	     `(let* ((,arg-list (list ,@args))
		     (,v (apply #',reader ,arg-list)))
		(if (eql ,v ':unassigned)
		    (let ((,v (apply #',computer ,arg-list)))
		      (prog1 ,v (apply #',writer ,v ,arg-list)))
		  ,v)))))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; anaphoric macros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro aif (test-form then-form &optional else-form)
  `(let ((it ,test-form))
     (if it ,then-form ,else-form)))

(defmacro awhen (test-form &body body)
  `(aif ,test-form
	(progn ,@body)))

(defmacro awhile (expr &body body)
  `(do ((it ,expr ,expr))
       ((not it))
     ,@body))

(defmacro aand (&rest args)
  (cond ((null args) t)
	((null (cdr args)) (car args))
	(t `(aif ,(car args) (aand ,@(cdr args))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; CLOS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro def-symmetric-method (name (a1 a2 &rest remaining) &body body)
  "def-symmetric-method NAME (ARG1 ARG2 &rest REMAINING) &rest BODY

Expands to code that defines two methods.  The first is obtained by replacing def-symmetric-method with defmethod.  The second is obtained by reversing the first two args"
  `(progn
     (defmethod ,name (,a1 ,a2 ,@remaining) ,@body)
     (defmethod ,name (,a2 ,a1 ,@remaining) ,@body)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; testing macros
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *lhs*)
(defvar *rhs*)

(defmacro do-tests (msg &rest args)
  (let ((x (gensym))
	(y (gensym)))
    `(progn
       (format t "~&~a" ,msg)
       ,@(loop
	     with a = args
	     while a
		
	     collect `(let ((,x ,(first a))
			    (,y ,(second a)))
			
			(let ((,x (if (eq ,y t) (to-boolean ,x) ,x)))
			(unless (equalp ,y ,x)
			  (setf *lhs* ,x
				*rhs* ,y)
			  (assert () ()
			    "~a equalled ~a instead of ~a"
			    ',(first a) ,x ,(second a)))
			(format t ".")
			))
	     do (setf a (cddr a)))
       (format t "check"))))

(defmacro do-boolean-tests (msg &rest args)
  `(do-tests ,msg
     ,@(loop
	   with a = args
	   while a
	   collect `(to-boolean ,(first a))
	   collect (second a)
	   do (setf a (cddr a)))))

(defmacro do-rand-tests (msg &rest args)
  (let ((x (gensym)))
    `(progn
       (format t "~&~a." ,msg)
       ,@(loop
	     with a = args
	     while a
		
	     collect `(let ((,x ,(first a)))
			
			(unless ,x
			  (setf *lhs* ,x)
			  (warn "~a did not succeed (it's a randomized test, so it may be due to chance)"
				',(first a)))
			(format t ".")
			)
	     do (setf a (cdr a)))
       (format t "check"))))


(defmacro tests (msg &body args)
  (with-gensyms (x y)
    `(progn
       (format t "~&~a" ,msg)
       ,@(mapcar 
	  #'(lambda (a)
	      (destructuring-bind (lhs &optional (rhs nil rhs-supp) (test '(function equalp) test-supp)) a
		(when (and (not test-supp) (consp rhs) (eq (first rhs) 'function))
		  (setf test-supp t
			test rhs
			rhs-supp nil))
		(let ((name (if (and (consp test) (eq (first test) 'function) (symbolp (second test)))
				(second test)
			      nil)))
		  (if rhs-supp
		      `(let ((,x ,lhs)
			     (,y ,rhs))
			 (unless (funcall ,test ,x ,y)
			   (setf *lhs* ,x
				 *rhs* ,y)
			   (cerror "continue with remaining tests"
				   "~a did not equal ~a when evaluating~&~a" ,x ,y ',(first a)))
			 (write-char #\. t)
			 (force-output t))
		    `(let ((,x ,lhs))
		       (unless (funcall ,test ,x)
			 (cerror "continue with remaining tests"
				 "~a, which evaluated to ~a, did not satisfy test~:[~; ~:*~a~]" ',(first a) ,x ',name))
		       (write-char #\. t)
		       (force-output t))))))
	  args)
       (format t "check"))))
			   
		    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; generalized iteration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro do-iterator (iter (var &optional result-form num-var) &body body)
  "macro do-iterator ITERATOR (VAR &optional RESULT-FORM NUM-VAR) &body BODY

ITERATOR - a function (evaluated)
VAR - a symbol (not evaluated)
RESULT-FORM - a form (not evaluated)
NUM-VAR a symbol (not evaluated)
BODY - a list of forms enclosed by an implicit progn

A macro for writing other iteration macros.  Repeatedly calls the function ITERATOR, which must be a function that returns two values.  If the second value is non-nil, stop and return RESULT-FORM.  Otherwise, evaluate BODY with VAR bound to the first return value of the call to ITERATOR.  If NUM-VAR is provided, then during BODY it is bound to an integer representing the number of times (not including this one) the BODY has been evaluated so far."
  (with-gensyms (done iterator)
    `(loop
	 with (,var ,done)
	 with ,iterator = ,iter
         ,@(awhen num-var `(for ,num-var from 0))
	 do (multiple-value-setq (,var ,done)
	      (funcall ,iterator))
	 when ,done do (return ,result-form)
	 ,@(awhen body `(do ,@body)))))

(defun map-iterator-to-list (fn &rest iters &aux (n (length iters)))
  "map-iterator-to-list FUNCTION &rest ITERS.  Each ITER is a function that returns two values - NEXT-ITEM and DONE.  Until one of the iterators' DONE return value is true, repeatedly apply FUNCTION to the ITEMs and collect the results into a list."
  
  (let ((head nil)
	(tail nil)
	(items (make-list n)))
    (loop
      (map-into items
		#'(lambda (iter)
		    (mvbind (next done?) (funcall iter)
		      (when done? (return-from map-iterator-to-list head))
		      next))
		iters)
      (let ((new-pair (cons (apply fn items) nil)))
	(if tail
	    (setf (cdr tail) new-pair
		  tail (cdr tail))
	  (setf head new-pair
		tail new-pair))))))
		     
		
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Reading
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun define-read-dispatch (char table fn)
  "define-read-dispatch CHAR READTABLE FN

Causes the dispatching read macro for CHAR in READTABLE to work by reading the next thing off the stream, then applying FN to the result."
  (set-dispatch-macro-character 
   #\# char
   #'(lambda (str subchar arg)
       (declare (ignore subchar arg))
       (funcall fn (read str t nil t)))
   table))
				


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; alist functions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmacro make-alist-function ((&rest args) &body body)
  (let ((l (gensym)))
    `#'(lambda (,l)
	 (let ,(mapcar #'(lambda (a) `(,a (cdr (check-not-null (assoc ',a ,l) "The key ~a in the alist ~a" ',a ,l)))) args)
	   ,@body))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug out
;; see also the old version debug-print, etc in utils.lisp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *debug-indent* 0)
(defun debug-indent ()
  (format *query-io* "~&~V,T" *debug-indent*))
(defvar *debug-topic-parents* (make-hash-table :test #'eq))

(defun define-debug-topic (topic parent)
  (setf (gethash topic *debug-topic-parents*) parent))
  

(defmacro with-debug-indent (&body body)
  `(let ((*debug-indent* (+ 3 *debug-indent*)))
     ,@body))

(defvar *debug-stream* t)
(defvar *debug-topics* (make-hash-table :test #'equal))

(defun set-debug-level (topic level &rest args)
  (setf (gethash topic *debug-topics*) level)
  (awhen args (apply #'set-debug-level it)))

(defun reset-debug-level (&rest args)
  (setf *debug-topics* (make-hash-table :test #'equal))
  (awhen args (apply #'set-debug-level it)))

(defmacro debug-out (topic level cond str &rest args)
  "debug-out TOPIC LEVEL CONDITION FORMAT-STRING &rest ARGS

When CONDITION is true, see if the level of this TOPIC exceeds LEVEL, and if so, print debugging message to *debug-stream* using format string and args (which are only evaluated in this case).
The TOPICs are arranged in a forest (using define-debug-topic) and the level of the TOPIC is found by moving up the tree until a topic is found for which the level has been set using set-debug-level."
  `(when (and ,cond (topic-level-exceeds ',topic ,level))
     (format *debug-stream* ,str ,@args)))
  

(defun get-topic-level (topic)
  (or (gethash topic *debug-topics*)
      (aif (gethash topic *debug-topic-parents*)
	   (get-topic-level it)
	   '-infty)))      

(defun topic-level-exceeds (topic level)
  (my> (get-topic-level topic) level))

