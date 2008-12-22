(in-package prop-logic)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; special vars
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *compound-formula-type* 'hash-table
  "For conjunctions or disjunctions, this variable determines how the set of conjuncts/disjuncts will be represented by default in certain DNF operations.  Currently allowed to be 'list or 'hash-table")

(defvar *dnf-or-method* :check-subsumption "Can be either :naive, which just takes all the non-nil clauses and disjoins them, or :check-subsumption, which additionally checks each clause being or-ed in to see if it subsumes/is subsumed by an existing one.  Should only be set by 'top-level' code.")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; creation and type checking
;; DNF formulae are disjunctions of conjuncts of literals
;; The disjunct sets are lists and the conjunct sets may
;; be represented as lists or hashtables depending
;; on the value of *compound-formula-type*
;; Operations are designed to be efficient in the hashtable
;; case
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun is-dnf-clause (x)
  "is-dnf-clause X.  Is X a (possibly empty) conjunction of literals, or a single literal?"
  (or (typep x 'literal)
      (and (typep x 'conjunction)
	   (each (conjuncts x) #'(lambda (y) (typep y 'literal))))))

(defun is-dnf-formula (x)
  "is-dnf-formula.  Is X a (possibly empty) disjunction of DNF clauses?"
  (and (typep x 'disjunction)
       (each (disjuncts x) #'is-dnf-clause)))

(defun make-state-dnf (state props)
  "make-state-dnf STATE PROPS.  Make a DNF formula that is true only for the assignment represented by STATE.  For this to be well-defined (since STATE is specified using the closed-world assumption), we also need to say what the overall set of propositions is."
  (flet ((get-prop-lit (p)
	   (if (holds state p) p (negate p))))
    (disjoin 
     (conjoin-set
      (ecase *compound-formula-type*
	(list (mapset 'list #'get-prop-lit props))
	(hash-table (let ((h (make-hash-table :test #'equal :size (size props))))
		      (do-elements (p props h)
			(setf (gethash (get-prop-lit p) h) t)))))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun dnf-or (&rest f)
  "dnf-or &rest DNF-FORMULAE.  Affected by *dnf-or-method*"
  
  (ecase *dnf-or-method*
    (:naive (disjoin-set (mapcan #'(lambda (x) (remove nil (disjuncts x))) f)))
    (:check-subsumption
     (let ((clauses nil))
       (dolist (formula f (disjoin-set clauses))
	 (do-elements (clause (disjuncts formula))
	   (when clause
	     (dolist (cl clauses (push clause clauses))
	       (cond
		 ((clause-implies clause cl) (return nil)) ;; skip this clause
		 ((clause-implies cl clause) (setq clauses (delete cl clauses))))))))))))
       
	 
	 
	    

(defun dnf-and (&rest f)
  "dnf-and &rest DNF-FORMULAE.  The logical and of the formulae.  Repeatedly uses the 'distributive law'.  Also checks for contradictions or nil within clauses."
  (disjoin-set 
   (delete nil 
	   (mapset
	    'list
	    #'conjoin-clauses
	    (apply #'direct-product 'list (mapcar #'disjuncts f))))))

(defun conjoin-clauses (clauses)
  (unless (member nil clauses)
    (conjoin-set
     (case *compound-formula-type*
       (hash-table 
	(let ((h (make-hash-table :test #'equal :size (* (length clauses) (size (conjuncts (item 0 clauses)))))))
	  (do-elements (c clauses h)
	    (do-elements (l (conjuncts c))
	      (if (or (null l) (hash-table-has-key h (negate l)))
		  (return-from conjoin-clauses nil)
		(unless (eq l t) (setf (gethash l h) t)))))))
       (otherwise
	(let ((l nil))
	  (do-elements (c clauses l)
	    (do-elements (lit (conjuncts c))
	      (if (or (null lit) (member (negate lit) l :test #'equal))
		  (return-from conjoin-clauses nil)
		(unless (or (eq lit t) (member lit l :test #'equal)) (push lit l)))))))))))


(defun dnf-not (f)
  "dnf-not F.  Negate F and convert to DNF.  Works for dnf formulae or clauses."
  (etypecase f
    (conjunction (disjoin-set (mapset 'list (fn (conjoin negate)) (conjuncts f))))
    (disjunction (apply #'dnf-and (mapset 'list #'dnf-not (disjuncts f))))))

(defun dnf-implies (f1 f2)
  "does dnf formula F1 imply DNF formula f2?"
  (let ((c2 (disjuncts f2)))
    (each (disjuncts f1) #'(lambda (f) (dnf-clause-implies f c2)))))

(defun dnf-clause-implies (c clauses)
  "dnf-clause-implies CLAUSE CLAUSES.  Does CLAUSE imply the disjunction of CLAUSES?"
  (when clauses
    (dsbind (c2 . crest) clauses
      (let ((lits (conjuncts c))
	    (new-lits (make-hash-table :test #'equal :size (size (conjuncts c2)))))
	(do-elements (lit (conjuncts c2))
	  (unless (member? lit lits)
	    (unless crest (return-from dnf-clause-implies nil))
	    (let ((negated (negate lit)))
	      (if (member? negated lits)
		  (return-from dnf-clause-implies (dnf-clause-implies c crest))
		(setf (gethash negated new-lits) t)))))
	(each 
	 new-lits
	 #'(lambda (l)
	     (prog2 
		 (etypecase lits (list (push l lits)) (hash-table (setf (gethash l lits) t)))
		 (dnf-clause-implies (conjoin-set lits) crest) ;; not strictly necessary to create a new conjunction here
	       (etypecase lits
		 (list (pop lits))
		 (hash-table (remhash l lits))))))))))

(defun clause-implies (cl1 cl2)
  (subset (conjuncts cl2) (conjuncts cl1)))
      

(defun dnf-consistent (f1 f2)
  "are F1 and F2 consistent?"
  (any (disjuncts f1)
       #'(lambda (c1)
	   (any (disjuncts f2)
		#'(lambda (c2)
		    (consistent-clauses c1 c2))))))

(defun consistent-clauses (c1 c2)
  (let ((lits1 (conjuncts c1))
	(lits2 (conjuncts c2)))
    (when (> (size lits1) (size lits2))
      (rotatef lits1 lits2))
    (not (any lits1 #'(lambda (lit) (member? (negate lit) lits2))))))


(defun pprint-dnf-clause (str clause)
  (pprint-logical-block (str (if (typep clause 'conjunction) (to-list (conjuncts clause)) (to-list clause)) :prefix "(" :suffix ")")
    (loop
      (pprint-exit-if-list-exhausted)
      (let ((conjunct (pprint-pop)))
	(pprint-logical-block (str nil)
	  (pprint-pop)
	  (if (typep conjunct 'proposition)
	      (write conjunct :stream str)
	    (format str "(not ~a)" (second conjunct)))
	  (pprint-newline :fill str)))
      
      (pprint-exit-if-list-exhausted)
      (princ " and " str)
      (pprint-newline :fill str))))

(defun pprint-dnf (str f)
  (let ((clauses (to-list (disjuncts f))))
    (if (= 0 (length clauses))
	(princ nil str)
      (pprint-logical-block (str (to-list (disjuncts f)) :prefix "(" :suffix ")")
	(loop
	  (pprint-exit-if-list-exhausted)
	  (pprint-dnf-clause str (pprint-pop))
	  (pprint-exit-if-list-exhausted)
	  (princ " or " str)
	  (pprint-newline :fill str))))))


(defmacro with-dnf-pprint (&rest body)
  "with-dnf-pprint &rest BODY.
BODY is evaluated, and during the dynamic extent of this evaluation, lists that can be interpreted as dnf formulae are printed using a more readable syntax."
  `(let ((table (copy-pprint-dispatch)))
     (set-pprint-dispatch '(and list (not null) (satisfies is-dnf-formula) compound-formula) 
			  #'pprint-dnf 0 table)
     (set-pprint-dispatch '(and list (not null) (satisfies is-dnf-clause) compound-formula) 
			  #'pprint-dnf-clause 1 table)
     (set-pprint-dispatch '(and list (not null) literal) 
			  (pprint-dispatch '(a) table) 2 table)
     (let ((*print-pprint-dispatch* table))
       ,@body)))




    

(defun literal< (l1 l2)
  (let ((p1 (literal-prop l1))
	(p2 (literal-prop l2)))
    (if (prop< p1 p2)
	t
      (if (prop< p2 p1)
	  nil
	(and (not (typep l1 'negation)) (typep l2 'negation))))))

(defun prop< (p1 p2)
  (if (symbolp p1)
      (if (symbolp p2)
	  (symbol< p1 p2)
	t)
    (if (symbolp p2)
	nil
      (loop
	  for s1 in p1
	  for s2 in p2
		    
	  when (symbol-or-number< s1 s2)
	  return t
		 
	  when (symbol-or-number< s2 s1)
	  return nil
		 
	  finally (return (> (length p2) (length p1)))))))

(defun symbol-or-number< (x1 x2)
  (if (symbolp x1)
      (if (numberp x2)
	  t
	(symbol< x1 x2))
    (if (symbolp x2)
	nil
      (< x1 x2))))





(defun clause< (c1 c2)
  (loop
      with l1 = (conjuncts c1)
      with l2 = (conjuncts c2)
		  
      for s1 = (first l1)
      for s2 = (first l2)
		  
      when (null l1)
      return t
	       
      when (null l2)
      return nil
	       
      when (literal< s1 s2)
      return t
	       
      when (literal< s2 s1)
      return nil
	       
      do (setf l1 (cdr l1)
	       l2 (cdr l2))
      finally (return nil)))
	       
		 
      






