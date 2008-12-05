;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; utils.lisp
;; 
;; General-purpose utilities
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(in-package utils)


(defun get-lambda-list (fn)
  "get-lambda-list FUNCTION.  Return the lambda-list of the function.  There's no CL function that does this but most implementations provide a function.  Will signal an error if not using one of the supported implementations (allegro, clisp, cmu, corman, gcl, lispworks, lucid, sbcl)"
  #+allegro (excl:arglist fn)
  #+clisp (sys::arglist fn)
  #+cmu (values (let ((st (kernel:%function-arglist fn)))
                  (if (stringp st) (read-from-string st)
                      (eval:interpreted-function-arglist fn))))
  #+cormanlisp (ccl:function-lambda-list
                (typecase fn (symbol (fdefinition fn)) (t fn)))
  #+gcl (let ((fn (etypecase fn
                    (symbol fn)
                    (function (si:compiled-function-name fn)))))
          (get fn 'si:debug))
  #+lispworks (lw:function-lambda-list fn)
  #+lucid (lcl:arglist fn)
  #+sbcl (let ((st (sb-introspect:function-arglist fn)))
	   (if (stringp st) (read-from-string st)
	       st))
; jawolfe                      #+ignore(eval:interpreted-function-arglist fn))))
  #-(or allegro clisp cmu cormanlisp gcl lispworks lucid sbcl)
  (error 'not-implemented :proc (list 'arglist fn))) 



(declaim (inline indicator))
(defun indicator (x &optional (v 1))
  "indicator X &optional (V 1).  Returns V if X is true and 0 otherwise.  See also iwhen and iunless."
  (if x v 0))

(declaim (inline xor nxor))
(defun xor (x y)
  "xor X Y.  Treats X and Y as generalized booleans and returns their logical XOR."
  (if x (not y) y))

(defun nxor (x y)
  "nxor X Y.  Treats X and Y as generalized booleans and returns the negation of their logical XOR, so either they must both be true or neither."
  (if x y (not y)))



(defun below (hi)
  "below N.  Return the list (0 1 2 ... N-1)"
  (loop for x below hi collecting x))

(defun consec (low hi &optional (step 1))
  "consec LOW HI &optional (STEP 1).  Returns the list (low low+step ... low+k*step) where k is the last one that is <= HI"
  (loop for x from low to hi by step collecting x))


(defun number-sequence (s)
  "number-sequence S.  given a sequence s, return a vector of pairs (i . s(i)) (for debugging purposes)"
  (map 'vector #'cons (consec 0 (length s)) s))


(defun read-object-from-file (name)
  "read-object-from-file FILENAME.  Reads a single object from the file and returns it."
  (with-open-file (f name :direction :input)
    (read f)))


(defvar *pprint-num-decimal-places* 3)

(defun pprint-float (str obj)
  (let* ((x (round (* obj (expt 10 *pprint-num-decimal-places*))))
	 (j (dotimes (i *pprint-num-decimal-places* *pprint-num-decimal-places*)
	      (when (not (divisible-by x (expt 10 (1+ i))))
		(return i)))))

    (format str "~,vf" (max 1 (- *pprint-num-decimal-places* j)) obj)))


(set-pprint-dispatch 'float #'pprint-float)


(defun pprint-matrix (s a &optional (w 8) (d 2))
  "pprint-matrix STREAM MATRIX &optional (WIDTH 8) (DECIMAL-PLACES 2)
Matrix must be a 2d array of real numbers.  Prints to stream STREAM.  No newline is inserted at the beginning or end, and the current column is taken as the left margin.  Any number that's too wide to fit is printed as ###."
  (dbind (m n) (array-dimensions a)
    (pprint-logical-block (s nil)
      (dotimes (i m)
	(dotimes (j n)
	  (format s "~v,v,0,vF " w d #\# (aref a i j)))
	(unless (= i (1- m))
	  (pprint-newline :mandatory s))))))
	
      



	


(defun classify-bin (x l u num-bins &aux (y (- x l)) (m (- u l)))
  "classify-bin X L U NUM-BINS divides [L,U] into NUM-BINS equal-length bins and classifies X into one of them (or asserts if it doesn't fall within the bounds).  All the bins except the first are open on the left, and all are closed on the right."
  (assert (and (>= y 0) (<= y m)) () "value ~a out of bounds [0,~a] in classify-bin" y m)
  (check-type num-bins (integer 1) "a positive integer")
  (if (= y 0)
      0
    (1- (ceiling (* y (/ num-bins m))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug printing
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defvar *debug-level* 0)
(defvar *debug-indent* 0)
(defun debug-indent ()
  (format *query-io* "~&~V,T" *debug-indent*))

(defun string-replace-all (s1 s2 s3)
  ;; Quite inefficient, and only used in the debugging code below
  (let ((n (length s2)))
    (loop
      (let ((i (search s2 s1)))
	(if i
	    (setf s1 (concatenate 'string (subseq s1 0 i) s3 (subseq s1 (+ i n))))
	  (return-from string-replace-all s1))))))

(defun debug-print (l format-string &rest args)
  "debug-print L FORMAT-STRING &REST ARGS.  If *debug-level* exceeds L (which is viewed as having been provided iff the first argument is a number), apply format to *query-io* and the remaining arguments.  The formatting respects the current *debug-indent* level (see with-debug-indent).  Always return the last argument."
  (when (aand l (< it *debug-level*))
    (debug-indent)
    (let ((str *query-io*))
      (pprint-logical-block (str nil)
	(apply #'format str (string-replace-all (string-replace-all format-string "~&" "~:@_") "~%" "~:@_") args)
	;; We use a pprint block to enforce the indent.  In turn, this means we have to replace unconditional newlines with
	;; mandatory conditional newlines.  
	)
      (force-output str)))

  (awhen args (slast it)))

  

(defun debug-prompt (&rest args)
  "debug-prompt &optional (L 0) &rest ARGS.  If *debug-level* exceeds L (which is viewed as having been provided iff the first argument is a number or nil), prompt user to press enter.  If ARGS are provided, first apply format to *query-io* and them.  The formatting respects the current *debug-indent* level (see with-debug-indent)."
  (condlet
   (((typep (first args) '(or number null)) (l (first args)) (format-args (rest args)))
    (t (l 0) (format-args args)))
   (when (aand l (< it *debug-level*))
     (let ((str *query-io*))
       (debug-indent)
       (pprint-logical-block (str nil)
	 (awhen format-args
	   ;; We use a pprint block to enforce the indent.  In turn, this means we have to replace unconditional newlines with
	   ;; mandatory conditional newlines.  
	   (apply #'format str (string-replace-all (string-replace-all (car it) "~&" "~:@_") "~%" "~:@_") (cdr it))))
       (debug-indent)
       (format str "Press enter to continue or q to quit. ")
       (when (equalp (read-line t) "q")
	 (abort))))
   (awhen format-args (slast it))))


(defmacro when-debugging (&rest args)
  "macro when-debugging [LEVEL] &body BODY

LEVEL  : a fixnum.  Defaults to 0.
BODY : list of forms

When *debug-level* exceeds LEVEL, execute the body."
  (condlet
   (((numberp (first args)) (level (first args)) (body (rest args)))
    (t (level 0) (body args)))
   
   `(when (aand ,level (< it *debug-level*)) ,@body)))

(defmacro with-debug-indent (&body body)
  `(let ((*debug-indent* (+ 3 *debug-indent*)))
     ,@body))
     

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; miscellaneous
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defun wait-until (condition interval)
  "wait-until CONDITION INTERVAL.  Calls the function CONDITION with no arguments, and if it returns false, go to sleep for INTERVAL seconds and repeat this process until it returns a non-nil value, which is then returned by wait-until."
  (loop
      for ret-val = (funcall condition)
      until ret-val
	    
      do (sleep interval)
	 
      finally (return ret-val)))
  

(defun force-format (str &rest args)
  "force-format STREAM &rest ARGS.  Call format to STREAM using ARGS, then call force-output to STREAM.  Used on streams that buffer output, to ensure that the output appears immediately."
  (apply #'format str args)
  (finish-output str))


(defparameter *default-prompt-msg* "~&Press enter to continue. ")
(defun prompt (&optional msg)
  "prompt &optional MSG.  Print the prompt message to *query-io*, then do a read-line on this stream and return the result.If message is not provided, *default-prompt-msg* is used"
  (format *query-io* (or msg *default-prompt-msg*))
  (read-line *query-io*))


(defun make-evenly-spaced-intervals (min max num-true)
  "make-evenly-spaced-intervals MIN MAX NUM-TRUE.  

Suppose there is a counter variable V that goes from MIN to MAX.  We want a predicate P which is true at a set of NUM-TRUE evenly spaced integers, such that the gap between each successive pair is as large as possible and, the last one is MAX.  Works exactly for NUM-TRUE >= 0 and <= MAX-MIN.  For NUM-TRUE > MAX-MIN, just saves MAX-MIN times.  Treats NUM-TRUE = nil as 0.

Useful, for example, when saving a history of a given length for an algorithm."
  
  (let ((span (- max min)))
    (cond
     ((or (null num-true) (= num-true 0)) (constantly nil))
     ((> num-true (1+ span)) (constantly t))
     ((= num-true 1) (lambda (i) (eql i max)))
     (t (let ((inc (floor span (1- num-true))))
	  (lambda (i) (eql 0 (mod (- max i) inc))))))))



(defun round-decimal (x d)
  "round-decimal X D.  Round X to D decimal places.  For convenience, if X is not a number, just return it."
  (if (numberp x)
      (let ((base (expt 10 d)))
	(float (/ (round (* base x)) base)))
    x))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; extended real arithmetic
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declaim (inline between between2 my<= my< my+ ; my*
		 infinite mymax mymin myinv my/ myexp))

(deftype extended-real ()
  "An extended real number is either a real number, or one of the symbols 'utils:infty or 'utils:-infty.  The operations my<, my+, etc. operate on extended reals."
  `(or (member infty -infty)
       real))

(define-condition multiply-zero-by-infinity (error)
  ())

(define-condition add-infinity-and-minus-infinity (error)
  ())

(defun my<= (a b)
  "my<= A B.  <=, but also allows A and B to be 'infty or '-infty"
  (check-type a extended-real)
  (check-type b extended-real)
  (or (eq a '-infty)
      (eq b 'infty)
      (and (numberp a)
	   (numberp b)
	   (<= a b))))

(defun my< (a b)
  "my< A B.  < but also allows A and B to be 'infty or '-infty"
  (check-type a extended-real)
  (check-type b extended-real)
  (not (my<= b a)))

(defun my>= (a b)
  "my>= A B.  >= but also allows A and B to be 'infty or '-infty"
  (check-type a extended-real)
  (check-type b extended-real)
  (my<= b a))

(defun my= (a b)
  (if (symbolp a)
      (eq a b)
    (and (numberp a) (numberp b) (= a b))))

(defun my> (a b)
  "my> A B. > but also allows 'infy and '-infty"
  (check-type a extended-real)
  (check-type b extended-real)
  (not (my<= a b)))

(defun mymax (&rest args)
  "mymax &rest args.  Like MAX, but allows multiple args, which may be 'infty or '-infty"
  (let ((current-max '-infty))
    (dolist (x args current-max)
      (check-type x extended-real)
      (when (my> x current-max)
	(setf current-max x))
      (when (eq current-max 'infty) (return current-max)))))

(defun mymin (&rest args)
  "mymin &rest args.  Like MIN, but allows multiple args, which may be 'infty or '-infty"
  (let ((current-min 'infty))
    (dolist (x args current-min)
      (check-type x extended-real)
      (when (my< x current-min)
	(setf current-min x))
      (when (eq current-min '-infty) (return current-min)))))

(defun infinite (x)
  (member x '(infty -infty)))

(defun my+ (&rest args)
  "my+ &args.  Like +, but allows args to be 'infty or '-infty.  Attempting to add '-infty and 'infty results in an error."
  (labels 
      ((invalid () (error 'add-infinity-and-minus-infinity))
       (helper (a b)    
	 (case a
	   (infty (if (eq b '-infty) (invalid) 'infty))
	   (-infty (if (eq b 'infty) (invalid) '-infty))
	   (otherwise (if (infinite b) b (+ a b))))))
    
    (let ((sum 0))
      (dolist (a args sum)
	(setf sum (helper sum a))))))
      

(defun my- (&rest args)
  "Works like -.  For more than one argument, A,B,C,... returns ((A-B)-C)-... For a single argument A, returns -A."
  (dsbind (f . r) args
    (if r
	(apply #'my+ f (mapcar #'(lambda (x) (my* x -1)) r))
      (case f
	(-infty 'infty)
	(infty '-infty)
	(otherwise (- f))))))

(defun my* (&rest args)
  "my* A B.  Like * but allows args to be 'infty or '-infty.  You can't multiply 0 by +- infty, and in this case an error of type 'multiply-zero-by-infinity is signalled.  Note, though, that for some reason, signalling this error is quite inefficient, so if this is going to happen often in an inner loop, it might help to have the calling code do the check instead."
  (dbind (a b &rest c) args
    (if c
	(my* a (apply #'my* b c))
	(condlet
	    (((infinite b) (a b) (b a))
	     (t (a a) (b b)))
	  (if (infinite a)
	      (if (eq b 0)  
		  (error 'multiply-zero-by-infinity)
		  (if (eq (my> a 0) (my> b 0)) 'infty '-infty))
	      (* a b))))))

(defun myinv (a)
  (etypecase a
    ((member 0) 'infty)
    (real (/ 1 a))
    ((member infty -infty) 0)))

(defun my/ (a b)
  (my* a (myinv b)))
    

(defun myexp (a)
  (etypecase a
    (real (exp a))
    ((member infty) 'infty)
    ((member -infty) 0)))


	
	
(defun between (a b c)
  "between A B C.  Check if a is >= b and <= c.  Note that any of the values may be 'infty or '-infty"
  (and (my<= b a) (my<= a c)))

(defun between2 (a b c)
  "between2 A B C.  Check if a is >= b and < c.  Note that any of the values may be 'infty or '-infty"
  (and (my<= b a) (my< a c)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; miscellaneous
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun default-comparator (x y &optional (tol 0.0))
  (cond
    ((< (abs-diff x y) tol) '=)
    ((< x y) '<)
    (t '>)))

(declaim (inline divisible-by))
(defun divisible-by (m n)
  "divisible-by M N.  M and N must be integers.  Returns t iff M is divisible by N."
  (zerop (mod m n)))

(declaim (inline to-boolean))

(defun to-boolean (x)
  "to-boolean X.  nil if X is nil and t otherwise."
  (when x t))



(declaim (inline bit-true bit-false))

(defun bit-true (x)
  "Does the bit X equal 1?"
  (ecase x
    (1 t)
    (0 nil)))

(defun bit-false (x)
  "Does the bit X equal 0?"
  (ecase x
    (1 nil)
    (0 t)))

(defun map-iterator (result-type f iter)
  "Like map, except uses an iterator function (see do-iterator)."
  (assert (eq result-type 'list))
  (let ((l nil)
	(tail nil))
    (loop
       (mvbind (next done?) (funcall iter)
	 (if done?
	     (return l)
	     (let ((v (list (funcall f next))))
	       (if l
		   (setf (cdr tail) v
			 tail (cdr tail))
		   (setf l v
			 tail l))))))))


(declaim (inline abs-diff squared-diff))

(defun abs-diff (x y)
  "abs-diff X Y.  |X-Y|."
  (abs (- x y)))

(defun squared-diff (x y)
  "squared-diff X Y. (X-Y)^2"
  (expt (- x y) 2))


#+sbcl (defun ld (x)
	 "shorthand for load in sbcl that muffles style warnings"
	 (handler-bind ((style-warning #'muffle-warning)) (load x)))

(declaim (inline set-if-unbound))

(defun set-if-unbound (slot-name object value)
  "set-if-unbound SLOT-NAME OBJECT VALUE.  If the slot named SLOT-NAME of OBJECT is unbound, set its value to VALUE."
  (unless (slot-boundp object slot-name)
    (setf (slot-value object slot-name) value)))

(defun symbol< (s1 s2)
  (string< (symbol-name s1) (symbol-name s2)))


(declaim (inline fill-format fill-format-nl))

(defun fill-format (ch num str before after &rest args)
  "fill-format CHAR NUM STR BEFORE AFTER &rest ARGS"
  (let ((middle (map 'string (constantly ch) (below num))))
    (format str (concatenate 'string before middle after) args)))

(defun fill-format-nl (ch num &optional (str t))
  "fill-format-nl CHAR NUM &optional (STREAM t).  Expands to code that prints NUM copies of CHAR on a newline to STREAM."
  (fill-format ch num str "~&" ""))

(declaim (inline check-not-null))
(defun check-not-null (x &rest args)
  "check-not-null X MSG &rest ARGS.  Check that X is not null and return it."
  (let ((val x))
    (assert val () (concatenate 'string (apply #'format nil (or args '("object"))) " was unexpectedly nil."))
    val))


(defmacro undefmethod (name &rest args)
  (let ((arg (first args)))
    (typecase arg
      (cons (udm name nil arg))
      (t (udm name (list arg) (second args))))))

(defun udm (name qual specs)
  (let ((classes (mapcar #'(lambda (s) `(find-class ',s)) specs)))
    `(remove-method (symbol-function ',name)
		    (find-method (symbol-function ',name)
				 ',qual
				 (list ,@classes)))))



(defun print-string-unquoted (str s)
  (map nil
    #'(lambda (c) (write-char c str))
    s))

(defmacro without-quotes (&rest body)
  "without-quotes &rest BODY.  Execute the BODY, with the current pprint dispatch table temporarily replaced by one that doesn't print quotes or package names when printing symbols or strings."
  `(let ((*print-pprint-dispatch* (copy-pprint-dispatch)))
     (set-pprint-dispatch 'string #'print-string-unquoted)
     (set-pprint-dispatch 'symbol #'(lambda (str s) (print-string-unquoted str (symbol-name s))))
     ,@body))


(defun pprint-tabulated-pair (str x y width &optional (newline t))
  "pprint-tabulated-pair STREAM X Y WIDTH &optional (NEWLINE t).
Should be called with a pprint-logical-block.  Prints X followed by a colon, followed by Y at a tabstop WIDTH away from the start of X, all to STREAM.  If NEWLINE, then a mandatory newline is printed first."
  
  (when newline
    (pprint-newline :mandatory str))
  (without-quotes
   (format str "~W:~2,V:@T~W" x width y)))

  

(defvar *mixins* (make-hash-table :test #'equal))

(defun get-mixin-class (&rest parents)
  (let ((new-class-name (gensym)))
    (mvbind (c present) (gethash parents *mixins*)
      (if present
	  c
	(setf (gethash parents *mixins*)
	  (eval `(defclass ,new-class-name (,@parents) ()))
	  ;; Ugh... unfortunately, there's no way in ANSI CL to define
	  ;; classes with names not known at compile time.  A cleaner way
	  ;; would be to use the MOP and ensure-class.
	  )))))
    

(define-condition exact-class-error (error)
  ((instance :initarg :instance :reader instance)
   (class-list :initarg :class-list :reader class-list))
  (:report (lambda (c s) 
	     (format s "Class name of ~a was not in the allowed list ~a"
		     (instance c) (class-list c)))))

(defun check-exact-class (x name)
  "check-exact-class INSTANCE CLASS-NAMES
INSTANCE - an object.
CLASS-NAMES -  Either a list of symbols, or a symbol, which is treated as a single-element list.

If the name of (class-of INSTANCE) belongs to the given list of names, nothing happens.  Otherwise, an error is signalled, of type exact-class-error.

The macro is useful to protect against situations when the standard semantics of method combination doesn't quite do the right thing.  For example, suppose a generic function named foo is defined, and a method is defined for foo for instances of class bar.  Later, a class named baz is defined, but the programmer forgets to define a method for foo.  Code that calls foo on instances of baz will, rather than failing, silently use the method from bar.  Using check-exact-class in the method for foo for bar can be used to guard against this possibility."
  
  (let ((class-list 
	 (etypecase name
	   (symbol (list name))
	   (list name))))
    (unless (member (class-name (class-of x)) class-list)
      (error 'exact-class-error :instance x :class-list class-list))))


(defun var-names (lambda-list)
  (mapcar #'(lambda (x) (etypecase x (list (first x)) (symbol x)))
	  (remove-if #'(lambda (x) (member x '(&key &aux &rest &optional))) lambda-list)))

(defmacro defstub (name arg-list &rest args)
  (condlet
   (((stringp (first args)) (doc (first args)) (body (rest args)))
    (t (doc "") (body args)))
   `(defun ,name ,arg-list ,doc (declare (ignorable ,@(var-names arg-list))) ,@body)))

(defun current-time-str ()
  "Return string representing current time in hh:mm:ss format"
  (mvbind (s m h) (get-decoded-time)
    (format nil "~D:~2,'0D:~2,'0D" h m s)))

(defmacro results-in-error (error-type &rest body)
  "Macro results-in-error ERROR-TYPE &rest BODY.
If BODY results in an error of type ERROR-TYPE (not evaluated), return t.  If it runs without errors return nil.  If it throws any other errors, they are passed along."
  `(handler-case
       (progn
	 ,@body
	 nil)
     (,error-type ()
       t)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; package params
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *package-params* (make-hash-table))

(defmacro declare-params (&rest args)
  "declare-params (VAR1 VAL1 &optional DOC1) ... (VARn VALn &optional DOCn)

Intended for use within test script files.
Creates variables (as if with defvar) having the given initial values.
Also, sets up an entry in the database, so print-params and restore-params will do the right thing.  Returns t iff this is the first time declare-params was called within this package, nil otherwise."
  (let ((val-vars (loop repeat (length args) collect (gensym)))
	(var-names (mapcar #'first args))
	(var-vals (mapcar #'second args))
	(var-doc (mapcar #'(lambda (x) (or (third x) "")) args)))
    `(let ,(mapcar #'list val-vars var-vals)
       ,@(mapcar #'(lambda (var val doc) `(defvar ,var ,val ,doc)) var-names val-vars var-doc)
       (prog1
	   (not (hash-table-has-key *package-params* *package*))
	 (setf (gethash *package* *package-params*)
	   (mapcar #'list ',var-names (list ,@val-vars) ',var-doc))))))

(defun print-params (&key (stream t) (package *package*))
  "print-params &key (STREAM t) (PACKAGE *package*)
Print current values of the package parameters created by declare-params."
  (let* ((vars (gethash (find-package package) *package-params*))
	 (field-width (+ 2 (loop for v in vars maximizing (length (symbol-name (car v))))))
	 (def-start (+ field-width 20))
	 (doc-start (+ field-width 40)))
    (if vars
	(progn
	  (format stream "Name~0,V@TValue~V,TDefault~V,TDocumentation" field-width def-start doc-start)
	  (dolist (v vars)
	    (format stream "~&~(~a~)~0,V@T~a~V,T~a~V,T~a" (car v) field-width (symbol-value (car v)) def-start (second v) doc-start (third v))))
      (format stream "No parameters found for ~a" *package*))))

(defun reset-params ()
  "reset-params.  Resets all parameters declared by declare-params within this package to their default value.  See also print-params."
  (dolist (param (gethash *package* *package-params*))
    (setf (symbol-value (first param)) (second param))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Infinite arrays
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (inf-array (:constructor create-inf-array))
  "An inf-array (infinite array) represents an array in which the coordinates can be unboundedly large integers.  The storage requirements at any point are S*2^R where R is the rank, and S is the maximum value of the product of the coordinates of any item in the array which does not equal (or has not equalled at some point) the default value.  Operations are make-inf-array, inf-aref, (setf inf-aref), and inf-array-get-table"
  table
  default-val
  bounds)

(defun make-inf-array (&key (rank nil) (default-val nil))
  "make-inf-array (RANK nil) (DEFAULT-VAL nil)

Make an inf-array with the given rank and default value."
  (check-type rank number)
  (create-inf-array :table (make-array (make-list rank :initial-element 0) :initial-element default-val) 
		    :default-val default-val
		    :bounds (make-list rank :initial-element 0)
		    ))

(defmethod print-object  ((a inf-array) str)
  (let ((dims (array-dimensions (inf-array-table a))))
    (if (every #'zerop dims)
	(print-unreadable-object (a str :type t :identity nil)
	  (format str "Dimensions: ~a Default value: ~a" dims (inf-array-default-val a)))
      (call-next-method))))

(defun inf-aref (a &rest coords)
  "inf-aref INF-ARRAY &rest COORDS.  Like AREF except with inf-arrays.  Can also be setf-ed."
  (let ((array (inf-array-table a)))
    (if (every #'< coords (array-dimensions array))
	(apply #'aref array coords)
      (dolist (coord coords (inf-array-default-val a))
	(check-type coord (integer 0 *))))))

(defun (setf inf-aref) (v a &rest coords)
  (let* ((dims (array-dimensions (inf-array-table a)))
	 (array
	  (if (every #'< coords dims)
	      (inf-array-table a)
	    (setf (inf-array-table a)
	      (adjust-array
	       (inf-array-table a)
	       (mapcar #'(lambda (c d) (if (< c d) d (1+ (* 2 c)))) coords dims)
	       :initial-element (inf-array-default-val a))))))
    (let ((b (inf-array-bounds a))
	  (i -1))
      (dolist (c coords)
	(maxf (nth (incf i) b) (1+ c))))
    (setf (apply #'aref array coords) v)))

(defun inf-array-get-table (a)
  "inf-array-get-table A.  Return a finite array having the property that for any indices out of the bounds of this finite array, the value equals the default.  Returns a reference to the original object, so that modifications to the returned object will affect future return values of (inf-aref A ...) calls."
  (inf-array-table a))




(in-package cl-user)


  




