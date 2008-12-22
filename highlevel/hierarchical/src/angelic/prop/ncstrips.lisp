;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; lookahead/prop/nc-strips.lisp
;; nc-strips (nondeterministic, conditional strips) :
;; a particular representation for abstract action descriptions in 
;; propositional domains
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; nondeterministic strips
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (nstrips (:constructor create-nstrips))
  "Structure type nstrips.  Represents a nondeterministic STRIPS outcome.
Create using make-nstrips with initargs
:add-list - as in strips
:delete-list - as in strips
:poss-add-list - literals that can possibly be added.
:poss-delete-list - literals that can possibly be deleted.
:reward - bound on reward

Each of these arguments must be a designator for a list of propositions, i.e., if it is not a list it is interpreted as a one-element list.  Note that this won't work in the case where there's a single proposition which happens to be a list, i.e., you can use 'foo, but instead of '(foo 3 4), use '((foo 3 4)).

None of the lists can intersect, except poss-add-list and poss-delete-list

An nstrips object, once created, should be treated as immutable - the lists should not be changed.  The clone method therefore just returns the original object."

  (add-list nil)
  (delete-list nil)
  (poss-add-list nil)
  (reward 0)
  (poss-delete-list nil))

(defmethod clone ((d nstrips))
  d)

(defun make-nstrips (&key (add-list nil) (delete-list nil)
			  (poss-add-list nil) (poss-delete-list nil)
			  (reward 0))
  (flet ((check-valid (l)
	   (assert
	       (and (listp l) (every #'(lambda (x) (typep x 'proposition)) l))
	       nil "~a is not a list of propositions" l))
	 (check-not-intersecting (l1 l2)
	   (assert
	       (notany (lambda (x) (member x l2 :test #'equal)) l1)
	       nil "~a and ~a intersect" l1 l2)))
    (let ((add-list (designated-list add-list))
	  (delete-list (designated-list delete-list))
	  (poss-add-list (designated-list poss-add-list))
	  (poss-delete-list (designated-list poss-delete-list)))
      (check-valid add-list)
      (check-valid delete-list)
      (check-valid poss-add-list)
      (check-valid poss-delete-list)
      
      (create-nstrips :add-list add-list :delete-list delete-list
		      :poss-add-list poss-add-list 
		      :poss-delete-list poss-delete-list
		      :reward reward))))

  

(defparameter *nstrips-noop*
    (make-nstrips)
  "Object of type nstrips that represents doing nothing.  So all four lists are empty.")


(defmethod successor-set ((desc nstrips) (s <dnf-set>))
  (make-dnf-set
   (nstrips-result desc (formula s))
   (props s)))
  
(defun nstrips-result (desc f)
  (disjoin-set
    (mapcar
    #'(lambda (c) (nstrips-clause-result desc c))
    (disjuncts f))))



(defun nstrips-clause-result (desc c)
  (check-type c conjunction)

  ;; extract the set of literals in this clause
  (let ((lits (conjuncts c))
	(new-literals (make-hash-table :test #'equal))
	(adds (nstrips-add-list desc))
	(dels (nstrips-delete-list desc))
	(poss-adds (nstrips-poss-add-list desc))
	(poss-dels (nstrips-poss-delete-list desc)))

    (do-elements (l lits)
      (let ((p (literal-prop l)))
	(unless 
	    (or (member p adds :test #'equal) 
		(member p dels :test #'equal)
		(and (member p poss-adds :test #'equal) (typep l 'negation))
		(and (member p poss-dels :test #'equal) (typep l 'proposition)))
	  (setf (gethash l new-literals) t))))
      
    (dolist (p adds) (setf (gethash p new-literals) t))
    (dolist (p dels) (setf (gethash (negate p) new-literals) t))
    (conjoin-set new-literals)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; nondeterministic, conditional STRIPS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic defs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defstruct (ncstrips-clause (:conc-name ncc-) (:constructor make-ncstrips-clause (precond effect)))
  precond
  effect)

(defclass ncstrips (<simple-description>)
  ((clauses :accessor ncstrips-clauses :initarg :clauses)))

(defmethod clone ((d ncstrips))
  d)

(defun make-ncstrips (&rest args)
  "make-ncstrips PRECOND1 EFFECT1 ... PRECOND-N EFFECT-N 
or
make-ncstrips ((PRECOND1 . EFFECT1) ... (PRECOND-N . EFFECTN))

Each precondition must be a conjunction, and each effect is of type nstrips.  The preconditions are evaluated in order, so the first case that matches is the one whose effect happens.  At a state satisfying none of the preconditions, the action is a noop."
  (let ((arg (if (= (length args) 1) (first args) (p2alist args))))
    (let ((preconds (mapcar #'car arg))
	  (effects (mapcar #'cdr arg)))
      (assert (every #'is-dnf-clause preconds) ()
	"Preconditions ~a must all be conjunctions of literals" preconds)
      (assert (every #'nstrips-p effects) ()
	"Effects ~a must all be of type nstrips" effects)
      (make-instance 'ncstrips :clauses (mapcar #'make-ncstrips-clause preconds effects)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; computing successor states and sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod succ-state (s (d ncstrips))
  (succ-state-ncstrips-helper s d s))

(defun succ-state-ncstrips-helper (props d s)
  (let ((next (etypecase props (cons (copy-list props)) (hash-table (copy-hash-table props)))))
    (dolist (c (ncstrips-clauses d) next) ;; when no precondition matches, just return s
      (when (holds s (ncc-precond c))
	(let ((e (ncc-effect c)))
	  (etypecase next
	    (cons (dolist (p (nstrips-add-list e))
		    (adjoinf next p :test #'equal))
		  (dolist (p (nstrips-delete-list e))
		    (deletef next p :test #'equal)))
	    (hash-table (dolist (p (nstrips-add-list e))
			  (setf (gethash p next) t))
			(dolist (p (nstrips-delete-list e))
			  (remhash p next))))
	  (return-from succ-state-ncstrips-helper next))))))

(defmethod succ-state ((s prop-domain-state) (d ncstrips))
  (make-prop-domain-state
   :domain (pds-domain s)
   :props (succ-state-ncstrips-helper (pds-props s) d s)))
	    
(defmethod successor-set ((d ncstrips) (s <prop-state-set>))
  "ncstrips-result NCSTRIPS-DESCRIPTION PROP-STATE-SET.  Returns a prop-state-set representing the set of possibilities if we start at a state in the original set then do an action with this description."
  (make-prop-state-set (pss-domain s) (sequence-dnf (formula s) (ncstrips-clauses d))))

(defun sequence-dnf (f ncstrips-clauses)
  (apply #'dnf-or (mapcar #'(lambda (c) (sequence-cnj c ncstrips-clauses)) (disjuncts f))))

(defun sequence-cnj (dnf-clause ncstrips-clauses)
  (when ncstrips-clauses
    (dsbind (c1 . crest) ncstrips-clauses
      (with-struct (ncc- precond effect) c1
	(let ((actual-precond (dnf-and dnf-clause precond)))
	  (dnf-or (awhen actual-precond (nstrips-result effect it))
		  (sequence-dnf (dnf-and dnf-clause (dnf-not precond)) crest)))))))


(defmethod hla-sound-reward ((d ncstrips) (s <prop-state-set>) s2)
  ;; Not quite correct, but lower bounds the actual reward, so everything will still work
  (ncstrips-hla-reward d s s2 #'mymin))

(defmethod hla-complete-reward ((d ncstrips) (s <prop-state-set>) s2)
  (ncstrips-hla-reward d s s2 #'mymax))

(defun ncstrips-hla-reward (d s s2 fn)
  (if (is-empty s2)
      '-infty
    (reduce-set 
     fn
     (direct-product 'list (ncstrips-clauses d) (disjuncts (formula s)))
     :key #'(lambda (x)
	      (dsbind (ncstrips-clause clause) x
		(if (dnf-consistent clause (ncc-precond ncstrips-clause))
		    (let ((r (nstrips-reward (ncc-effect ncstrips-clause))))
		      (etypecase r
			(number r)
			(function (funcall r (make-prop-state-set (pss-domain s) clause)))))
		  (funcall fn)))))))





;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; regression
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod regress ((s1 <prop-state-set>) (s2 <prop-state-set>) (desc ncstrips))
  (make-prop-state-set (pss-domain s1) (regress-dnf-ncstrips (formula s1) (formula s2) desc )))


(defun regress-dnf-ncstrips (f1 f2 desc)
  "Implements regress for the sets represented by formulas f1 and f2, and ncstrips description desc."
  (let ((source-clauses (disjuncts f1))
	(dest-clauses (disjuncts f2))
	(nclauses (ncstrips-clauses desc)))
    (apply #'dnf-or
	   (mapset 
	    'list
	    #'(lambda (l)
		(let ((pre (conjoin-clauses (list (second l) (ncc-precond (first l)))))
		      (eff (ncc-effect (first l))))
		  (unless (dnf-implies pre nil)
		    (apply #'dnf-or
			   (mapset 
			    'list 
			    #'(lambda (c2) (regress-clause-nstrips eff pre c2))
			    dest-clauses)))))
	    (direct-product 'list nclauses source-clauses)))))
		      



(defun regress-dnf-ncstrips-subset (f1 f2 desc )
  "An approximate version of regress-dnf-ncstrips that returns a clause representing a subset of the true answer."
  (let ((source-clauses (disjuncts f1))
	(dest-clauses (disjuncts f2))
	(nclauses (ncstrips-clauses desc)))
    (do-elements (nc nclauses (disjoin))
      (do-elements (c1 source-clauses)
	(let ((pre (conjoin-clauses (list (ncc-precond nc) c1))))
	  (unless (dnf-implies pre nil)
	    (do-elements (c2 dest-clauses)
	      (awhen (regress-clause-nstrips (ncc-effect nc) pre c2)
		(return-from regress-dnf-ncstrips-subset it)))))))))

(defun regress-clause-nstrips (desc clause1 clause2)
  "regress-clause-nstrips NSTRIPS-DESC CLAUSE1 CLAUSE2.

Return a new clause that implies CLAUSE1, such that for any state compatible with the new clause, progressing that state through DESC yields a clause that's compatible with CLAUSE2."

  ;; step through clause2, and l3 = progress(clause1)
  (let ((conjuncts nil))
    (do-elements (lit (conjuncts clause2))
      (if (typep lit 'negation)
	  (cond ((or (member (negatee lit) (nstrips-delete-list desc) :test #'equal)
		     (member (negatee lit) (nstrips-poss-delete-list desc) :test #'equal)))
		((member (negatee lit) (nstrips-add-list desc) :test #'equal)
		 (return-from regress-clause-nstrips nil))
		(t (push lit conjuncts)))
	(cond ((or (member lit (nstrips-add-list desc) :test #'equal)
		   (member lit (nstrips-poss-add-list desc) :test #'equal)))
	      ((member lit (nstrips-delete-list desc) :test #'equal)
	       (return-from regress-clause-nstrips nil))
	      (t (push lit conjuncts)))))
    (conjoin-clauses (list clause1 (conjoin-set conjuncts)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; schematized ncstrips descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (ncstrips-schema (:conc-name nil) (:constructor create-ncstrips-schema))
  domains
  var-names
  bindings
  desc
  dom
  arg-lists)


(defun make-ncstrips-schema (&key planning-problem var-domains effects bindings)
  "make-ncstrips-schema &key PLANNING-PROBLEM VAR-DOMAINS EFFECTS BINDINGS

VAR-DOMAINS is an association list from symbols to sets
EFFECTS is an association list mapping from preconditions, which are formulae, to nstrips descriptions."
  (create-ncstrips-schema :domains var-domains
			  :dom planning-problem
			  :desc (make-ncstrips effects)
			  :bindings bindings
			  :var-names (mapcar #'car var-domains)
			  :arg-lists (apply #'direct-product 'list (mapcar #'cdr var-domains))))


(defmacro make-ncstrips-schemas (dom &rest args)
  "make-ncstrips-schemas PLANNING-PROB &rest SPECS

Definition macro for creating a set of ncstrips schemas.
PLANNING-PROB is a planning problem of type <prop-domain>
Each SPEC_i is a list of the form (NAME :var-domains VAR-DOMAINS :effects EFFECTS) where
- NAME is a symbol
- VAR-DOMAINS looks like ((VAR1 DOM1) ... (VAR_n DOM_n)) where VAR_i is a symbol, and DOM_i is a symbol naming a set (according to the mapping in PLANNING-PROB)
- EFFECTS is a list of effects.

Each effect in EFFECTS is a key-value list with some of the following keys
- PRECOND is a literal or dnf clause with free variables among the VAR_i above.  Defaults to t.
- Each of ADD-LIST, DELETE-LIST, POSS-ADD-LIST, POSS-DELETE-LIST is a list, where each element is either 1) a positive literal 2) a list of the form (for-all ((VAR_1 DOM_1) ... (VAR_k DOM_k)) FORMULA LIT), where the VAR_k and DOM_k are interpreted as above, FORMULA is a formula, and LIT is a positive literal having free variables among the new variables and the original ones.  Each of the four lists defaults to nil.
- REWARD is a real number, 'infty, or '-infty
  
The meaning of the second form for the effect lists is that for each set of variable bindings in the given sets that satisfy the formula, we will add a corresponding occurrence of LIT to the given list.  Note that this is done independent of the state, i.e., formula must only refer to literals that are nonfluents for the given planning problem.

Another addition is that in the precondition, in any position where a term would normally be, you can put in a list, which is then treated as a form and evaluated in the lexical context where d is bound.  This allows referring to constant parameters of the domain."
  `(list
    ,@(mapcar
       #'(lambda (spec)
	   `(cons ',(car spec)
		  ,(lookup-types-in-schema-def (cdr spec) dom)))
       args)))

(defun lookup-types-in-schema-def (def dom)
  (labels ((replace-type (x)
	     `(lookup-type ,dom ',x))
	   (replace-terms (x)
	     `(list ',(prop-symbol x)
		    ,@(mapcar
		       #'(lambda (term)
			   (etypecase term
			     (symbol `',term)
			     (list term)))
		       (prop-args x))))
	       
	   (replace-effect-types (l)
	     "replace symbols in for-alls"
	     `(list
	       ,@(mapcar #'(lambda (x)
			     (if (and (listp x) (eq (first x) 'for-all))
				 (dsbind (lists formula lit) (rest x)
				   `(list 'for-all
					  (list
					   ,@(mapcar #'(lambda (l)
							 `(cons ',(first l)
								(lookup-type ,dom ',(second l))))
						     lists))
					  ',formula
					  ',lit))
			       `',x))
			 l)))
	   (replace-precond-types (precond)
	     "replace for-alls, function calls in precondition"
	     (etypecase precond
	       ((member t nil) precond)
	       (literal (replace-precond-types (conjoin precond)))
	       (conjunction
		(let ((v (gensym)))
		  `(conjoin-set
		    (append
		     ,@(mapcar #'(lambda (x)
				   (if (and (listp x) (eq (car x) 'for-all))
				       (dsbind ((var vals) prop) (cdr x)
					 `(mapset 'list #'(lambda (,v) (bind ,(replace-terms prop) (list (cons ',var ,v))))
						  (lookup-type ,dom ',vals)))
				     `(list ,(replace-terms x))))
			       (conjuncts precond))))))))
	   
	   (parse-reward-spec (spec)
	     "Parse a reward spec of the form (FNAME . ARGLIST)"
	     (typecase spec
	       ((or null number) `(constantly ,spec))
	       (otherwise
		(dsbind (fname . args) spec
		  (if (or (member '?complete-set args) (member '?sound-set args)
			  (member '?set args))
		      `#'(lambda (binds)
			   #'(lambda (s)
			       (apply #',fname
				      (mapcar #'(lambda (v)
						  (if (member v '(?complete-set ?sound-set ?set))
						      s
						    (evaluate binds v)))
					      ',args))))
		    `#'(lambda (binds)
			 (apply #',fname (mapcar #'(lambda (v) (evaluate binds v)) ',args))))))))
	   )
		 

    (dsbind (&key var-domains effects bindings) def
			     
      `(make-ncstrips-schema
	:planning-problem ,dom
	:var-domains (list
		      ,@(mapcar #'(lambda (pair)
				    `(cons ',(first pair) ,(replace-type (second pair))))
				var-domains))
	:bindings (list 
		   ,@(mapcar #'(lambda (b)
				 (dsbind (name &key depends-on function) b
				   `(list ',name ',depends-on #',function)))
			     bindings))
	:effects (list
		  ,@(mapcar #'(lambda (eff)
				(dsbind (&key (precond nil) (poss-add-list nil) (add-list nil)
					     (delete-list nil) (poss-delete-list nil) (reward nil))
				    eff
				  `(cons 
				    ,(replace-precond-types precond)
				    (make-nstrips
				     :add-list ,(replace-effect-types add-list)
				     :delete-list ,(replace-effect-types delete-list)
				     :poss-add-list ,(replace-effect-types poss-add-list)
				     :poss-delete-list ,(replace-effect-types poss-delete-list)
				     :reward ,(parse-reward-spec reward)
				     ))))
			    effects))))))
       
  
  
  




(defun instantiate (schema arg-list)
  "instantiate SCHEMA ARG-LIST
SCHEMA is an ncstrips schema or a function
ARG-LIST is the list of arguments."
  
  ;; One special case: we represent trivial sound or complete descriptions using functions
  (etypecase schema
    (function schema)
    (ncstrips-schema  

     ;; TODO this is quite inefficiently done.  Instead, should cache how to do this
     ;; when the schema is being created.
     (let ((bindings (mapcar #'cons (var-names schema) arg-list))
	   (dom (dom schema)))
       
       ;; Add the additional bindings done within the schema
       (dolist (b (bindings schema))
	 (dsbind (name deps fn) b
	   (let ((args (mapcar #'(lambda (v) (evaluate bindings v)) deps)))
	     (push (cons name (apply fn args)) bindings)
	     )))
    
       (flet ((set-vars (f)
		(if (and (listp f) (eq (first f) 'for-all))
		    (dsbind (lists condition lit) (rest f)
		      (let ((names (mapcar #'car lists))
			    (doms (mapcar #'cdr lists)))
			(mapset 'list
			  #'(lambda (vals)
			      (bind lit (append (mapcar #'cons names vals) bindings)))
			  (filter ':list (apply #'direct-product 'list doms)
				  #'(lambda (vals)
				      (holds-background 
				       dom (bind condition (append (mapcar #'cons names vals) bindings))))))))
				
		  (list (bind f bindings)))))
	 (make-ncstrips 
	  (mapcan
	   #'(lambda (clause)
	       (let ((pre (bind (ncc-precond clause) bindings)))
		 (check-type pre conjunction)
		 (setf pre
		   (let ((conjuncts
			  (mapcar #'(lambda (lit)
				      (if (eq (prop-type (dom schema) lit) 'nonfluent)
					  (holds-background dom lit)
					lit))
				  (conjuncts pre))))
		     (unless (member nil conjuncts) (conjoin-set (delete t conjuncts)))))
		 (when pre
		   (list
		    (cons pre
			  (let ((e (ncc-effect clause)))
			    (make-nstrips 
			     :add-list (mapcan #'set-vars (nstrips-add-list e))
			     :delete-list (mapcan #'set-vars (nstrips-delete-list e))
			     :poss-add-list (mapcan #'set-vars (nstrips-poss-add-list e))
			     :poss-delete-list (mapcan #'set-vars (nstrips-poss-delete-list e))
			     :reward (let ((r (nstrips-reward e))) (etypecase r (number r) (function (funcall r bindings))))
			     )))))))
	   (ncstrips-clauses (desc schema)))))))))

    



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Standard STRIPS (implemented in terms of NCSTRIPS)
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-strips (&key precond add-list delete-list)
  "make-strips &key PRECOND ADD-LIST DELETE-LIST
PRECOND - a conjunction of positive literals
ADD-LIST, DELETE-LIST - lists of positive literals."
  (make-ncstrips (p2alist precond (make-nstrips :add-list add-list :delete-list delete-list))))

(defun is-strips (desc)
  "is-strips NCSTRIPS-DESC.  Returns t if the description has a single deterministic clause whose precondition is a conjunction of positive literals."
  (check-type desc ncstrips)
  (let* ((clauses (ncstrips-clauses desc)))
    (unless (or (null clauses) (length-exceeds clauses 1))
      (let ((eff (ncc-effect (first clauses)))
	    (precond (ncc-precond (first clauses))))
	(and
	 (or (typep precond '(or conjunction literal)))
	 (every #'(lambda (x) (typep x 'proposition)) (conjuncts precond))
	 (null (nstrips-poss-add-list eff))
	 (null (nstrips-poss-delete-list eff)))))))

(deftype strips ()
  "subtype of ncstrips that satisfies is-strips"
  `(satisfies is-strips))

(defun precond (desc)
  "precond DESC.  DESC is required to be a STRIPS description."
  (check-type desc strips)
  (ncc-precond (first (ncstrips-clauses desc))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; schemas
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun make-strips-schema (&key domain domains precond add-list delete-list (reward -1))
  "make-strips-schema &key DOMAINS PRECOND ADD-LIST DELETE-LIST (REWARD -1)
DOMAIN is of type <prop-domain>
DOMAINS are as in make-ncstrips-schema
PRECOND is a conjunction of positive literals
ADD-LIST and DELETE-LIST are lists of positive literals.
REWARD is a number"
  (make-ncstrips-schema :var-domains domains :planning-problem domain
			:effects (p2alist precond (make-nstrips :add-list add-list :delete-list delete-list
								:reward reward))))

(defun strips-effect (n)
  (ncc-effect (first (ncstrips-clauses n))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Dummy ncstrips constructors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun vacuous-complete-ncstrips-descriptions (h)
  "vacuous-complete-ncstrips-descriptions PROP-HIERARCHY.  Returns a set of vacuous complete descriptions for a prop hierarchy, that map the empty dnf-set to itself, and anything else to the universal set represented by '(and)."
  (mapset 'list
    #'(lambda (a) (cons a #'(lambda (s) 
			      (if (is-empty s) 
				  s 
				(make-prop-state-set 
				 (pss-domain s) (dnf-or (dnf-and)))))))
    (domain (hla-schemas h))))

(defun vacuous-sound-ncstrips-descriptions (h)
  "vacuous-sound-ncstrips-descriptions PROP-HIERARCHY.  Returns a set of vacuous sound descriptions for a prop hierarchy, that map any set to the empty set represented by '(or)."
  (mapset 'list
    #'(lambda (a) (cons a #'(lambda (s) (make-prop-state-set (pss-domain s) (dnf-or)))))
    (domain (hla-schemas h))))

