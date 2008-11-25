(defpackage prop-logic
  (:documentation "Package for representing formulas in propositional logic.

Formulas are representing using the [formula] type.  Elementary propositions are represented by symbols or lists (denoting predicates), and t and nil represent the logical constants TRUE and FALSE.  More complex formulae are built out of these using conjoin, disjoin, or negate.  A propositional state (aka possible world aka model), using the closed world assumption, is represented by a list of propositions that are true in that state.

Basic creation and access
-------------------------
conjoin
conjoin-set
disjoin
disjoin-set
negate
conjuncts
disjuncts
negatee
literal-prop
standardize-prop
standardize-literal
prop-symbol
prop-args

Types
-----
is-state
is-formula
[formula]
is-dnf-clause
is-dnf-formula
proposition
literal
negation
conjunction
disjunction
compound-formula

General logic
-------------
holds

Substitution 
------------
bind

Parametrized sets of propositions
---------------------------------
make-prop-set

DNF
---
dnf-or
dnf-and
dnf-not
dnf-implies
dnf-consistent
make-state-dnf
*compound-formula-type*

DNF sets
--------
<dnf-set>
make-dnf-set
formula
props
size-must-exceed

Debugging
---------
pprint-dnf
with-dnf-pprint

   ")
  

  (:export
   conjoin
   conjoin-set
   disjoin
   disjoin-set
   negate
   conjuncts
   disjuncts
   negatee
   literal-prop
   standardize-literal
   standardize-prop
   prop-symbol
   prop-args
   
   is-state
   is-formula
   [formula]
   proposition
   literal
   negation
   conjunction
   disjunction
   compound-formula

   holds
   
   bind
   
   make-prop-set

   dnf-or
   dnf-and
   dnf-not
   is-dnf-clause
   is-dnf-formula
   dnf-implies
   dnf-consistent
   make-state-dnf
   *compound-formula-type*
   
   make-dnf-set
   <dnf-set>
   formula
   props
   size-must-exceed
   
   pprint-dnf
   with-dnf-pprint
   )
  (:use
   cl
   utils
   inst-vars
   prod-set
   set))
   

(in-package prop-logic)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; various basic types of formulae
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(deftype symbolic-proposition ()
  "A symbolic proposition is a symbol that is not a boolean"
  '(and symbol (not boolean)))

(deftype list-proposition ()
  "A list proposition is a list whose first element is not 'and, 'or, or 'not"
  '(cons (and symbol (not (member and or not)))))

(deftype proposition ()
  "A proposition is a list whose first element is a symbol that is not 'and, 'or, or 'not"
  ;; Note symbolic props no longer allowed
  '(or list-proposition))

(deftype literal ()
  "A literal is a proposition or negated proposition"
  '(or proposition (cons (eql not) (cons proposition null))))

(deftype conjunction ()
  "A conjunction is a cons of 'and and a set of formulae (note type checker will not recursively check well-formedness)."
  '(cons (eql and)))

(deftype disjunction ()
  "A conjunction is a cons of 'or and a set of formulae (note type checker will not recursively check well-formedness)."
  '(cons (eql or)))

(deftype negation ()
  "A negation is a list of the form (not FORMULA) (note type checker will not recursively check well-formedness)"
  '(cons (eql not)))

(deftype compound-formula ()
  "A compound formula is a conjunction, disjunction, or negation (note type checker will not recursively check well-formedness)"
  '(or conjunction disjunction negation))

(deftype formula ()
  "A formula is a boolean, proposition, or compound formula (note type checker may not recursively check well-formedness)"
  '(or boolean proposition compound-formula))
		   


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic creation and access
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(declaim (inline conjoin conjoin-set disjoin disjoin-set negate negatee
		 conjuncts disjuncts prop-symbol prop-args literal-prop
		 standardize-literal standardize-prop))

(defun conjoin (&rest args)
  "conjoin &rest FORMULAE.  Return a formula representing the logical and of the FORMULAE."
  (cons 'and args))

(defun conjoin-set (s)
  "conjoin-set S.  Return a formula representing the logical and of the formulae in set S."
  (cons 'and s))

(defun disjoin (&rest args)
  "disjoin &rest FORMULAE.  Return a formula representing the logical or of the FORMULAE."
  (cons 'or args))

(defun disjoin-set (s)
  "disjoin-set S.  Return a formula representing the logical or of the formulae in set S."
  (cons 'or s))

(defun negate (f)
  "negate FORMULA.  Return the negation of FORMULA, and also do simplification to remove double negatives."
  ;; don't do typecase for speed
  (cond
   ((member f '(t nil)) (not f))
   ((eq (car f) 'not) (second f))
   (t (list 'not f))))

(defun negatee (x)
  "negatee X.  For a negation, return the thing that's being negated (answer undefined if not a negation)."
  (second x))

(defun conjuncts (x)
  "conjuncts X.  If X is a conjunction, return the set of conjuncts, otherwise just return a list containing X."
  (typecase x
    (conjunction (cdr x))
    (otherwise (list x))))

(defun disjuncts (x)
  "disjuncts X.  If X is a disjunction, return the set of disjuncts, otherwise just return a list containing X."
  (typecase x
    (disjunction (cdr x))
    (t (list x))))

(defun prop-symbol (p)
  (first p))

(defun prop-args (p)
  (rest p))

(defun literal-prop (l)
  "literal-prop L.  Get the proposition used in L.  Answer undefined if not a literal."
  (if (eq (car l) 'not) (second l) l))

(defun standardize-prop (prop)
  "standardize-prop PROP.  Replace symbolic propositions by a list, e.g. FOO -> (FOO)."
  (typecase prop
    ((or boolean cons) prop)
    (t (list prop))))

(defun standardize-literal (l)
  "standardize-literal L.  If the proposition in L is just a symbol, enclose it in a list."
  (typecase l
    (negation (negate (standardize-prop (literal-prop l))))
    (t (standardize-prop l))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; general formulae
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun is-formula (x)
  "is-formula X

Returns true iff X is one of
1) t
2) nil
3) A symbol, or list whose first element is not 'not, 'and, or 'or and whose remaining elements are symbols or numbers
4) A list '(not F) where F is a formula
5) A list '(S . F) where F is a list of formulae and S is either 'and or 'or"
  (typecase x
    (boolean t)
    (proposition (and (symbolp (prop-symbol x))
		      (each (prop-args x) #'(lambda (y) (typep y '(or symbol number))))))
    (negation (is-formula (negatee x)))
    (conjunction (each (conjuncts x) #'is-formula))
    (disjunction (each (disjuncts x) #'is-formula))))

(deftype [formula] ()
  "Type for well formed propositional formulae.  Well-formed formulae are determined using the predicate is-formula.  See its documentation for more."
  `(satisfies is-formula))


(defun propositions (formula &optional (result-type 'list))
  "propositions FORMULA &optional (RESULT-TYPE 'list).  Return the set of elementary propositions used in this FORMULA.  RESULT-TYPE can only be 'list for now."
  (assert (eql result-type 'list))
  (let ((h (make-hash-table :test #'equal)))
    (labels 
	((helper (f)
	   (typecase f
	     (literal (setf (gethash (standardize-prop (literal-prop f)) h) t))
	     (negation (helper (negatee f)))
	     (conjunction (do-elements (c (conjuncts f)) (helper c)))
	     (disjunction (do-elements (d (disjuncts f)) (helper d))))))
      (helper formula)
      (hash-keys h))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; states
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun is-state (s)
  "A state aka model aka interpretation, is represented as the set of propositions that are true in it.  Propositions must be represented as lists, e.g. the shorthand of writing 'foo for '(foo) cannot be used."
  (each s #'(lambda (x) (typep x 'proposition))))
  


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; satisfaction
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defgeneric holds (state formula)
  ;; This is a generic function because sometimes state is not a set
  ;; see lookahead/prop/prop-domain.lisp
  (:documentation "holds STATE FORMULA.  Does STATE make FORMULA true?")
  (:method (state formula)
	   (flet ((holds-in-state (x) (holds state x)))
	     (etypecase formula
	       (boolean formula)
	       (proposition (member? (standardize-prop formula) state))
	       (conjunction (each (conjuncts formula) #'holds-in-state))
	       (disjunction (any (disjuncts formula) #'holds-in-state))
	       (negation (not (holds state (negatee formula))))))))


(defun consistent (f1 f2)
  "consistent F1 F2.  True iff F1 does not imply not(F2).  Only works for dnf clauses or formulae"
  ;; this is probably not the most efficient way to do it
  (cond
   ((typep f1 'boolean) f1)
   ((typep f2 'boolean) f2)
   ((and (typep f1 '(or literal conjunction)) (typep f2 '(or literal conjunction)))
    (let ((c1 (conjuncts f1))
	  (c2 (conjuncts f2)))
      (when (> (size c1) (size c2))
	(rotatef c1 c2))
      (each c1 #'(lambda (c) 
		   (etypecase c
		     (boolean c)
		     (literal (not (member? (negate c) c2))))))))
   (t (assert (and (is-dnf-formula f1) (is-dnf-formula f2)) ()
	"consistent currently only works for DNF formulae.")
      (not (dnf-implies f1 (dnf-not f2))))))

  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; substitution
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun bind (formula bindings)
  "bind FORMULA BINDINGS.  FORMULA is a formula, and BINDINGS is an association list mapping symbols to other symbols or numbers.  Returns a new formula which is identical to FORMULA, except that for every proposition of the form (NAME ARG1 ... ARGn) occurring in FORMULA, for which some of the ARGi are keys in BINDINGS, those arguments are replaced by their corresponding value in BINDINGS.  The substitutions are done in parallel.  The newly returned formula is fresh; in particular, it shares no structure with FORMULA.  "
  (typecase formula
    (boolean formula)
    ((or conjunction disjunction)
     (cons (car formula)
	   (mapset 'same #'(lambda (f) (bind f bindings)) (cdr formula))))
    (negation
     (negate (bind (negatee formula) bindings)))
    (proposition
     (if (symbolp formula)
	 formula
       (cons (first formula)
	     (mapcar #'(lambda (arg)
			 (mvbind (val defined?)
			     (mapping:evaluate-mv bindings arg)
			   (if defined? val arg)))
		     (rest formula)))))
    (t (assert nil nil "Unrecognized formula type for ~a" formula))))
			    
      
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sets of propositions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <prop-set> (<prod-set>)
  ((prop :initarg :prop-symbol :accessor prop)
   (domains :initarg :domains :accessor domains)))

(defun make-prop-set (prop-symbol &rest domains)
  "make-prop-set PROP-SYMBOL DOMAIN_1 ... DOMAIN_M

Return a compact representation of the set of propositions of the form (PROP-SYMBOL X_1 ... X_M) where each X_i belongs to the corresponding DOMAIN_i."
  
  (if domains
  
      (let* ((n (length domains))
	     (acc
	      (make-inst-var-accessors
	       :creator 
	       #'(lambda () 
		   (cons prop-symbol 
			 (make-list n :initial-element 'uninstantiated)))
	       :readers
	       (mapset 'vector #'(lambda (i) #'(lambda (l) (nth (1+ i) l))) n)
	       :writers
	       (mapset 'vector #'(lambda (i) #'(lambda (v l) (setf (nth (1+ i) l) v))) n)
	       :invalid-inst-detector
	       #'(lambda (inst)
		   (if (listp inst)
		       (handler-case
			   (if (eq (first inst) prop-symbol)
			       (if (every #'(lambda (x s) (member? x s)) (rest inst) domains)
				   nil
				 'invalid-component)
			     'incorrect-prop-symbol)
			 (simple-error (c)
			   (if (is-true-list inst)
			       (error c)
			     'not-true-list)))
		     'not-list))
	       :var-names n
	       :var-num #'identity)))
	(make-instance '<prop-set> :sets domains :inst-acc acc :prop-symbol prop-symbol :domains domains))
    (list (list prop-symbol))))

(defun pprint-prop-set (str s)
  (pprint-logical-block (str (domains s) :suffix "]")
    (format str "[Prop set with prop symbol ~a and components " (prop s))
    (loop
      (format str "~W" (pprint-pop))
      (pprint-exit-if-list-exhausted)
      (format str ", "))))

(set-pprint-dispatch '<prop-set> #'pprint-prop-set 2)