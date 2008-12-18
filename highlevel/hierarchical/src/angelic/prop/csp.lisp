(in-package lookahead)


(defstruct (csp (:constructor create-csp))
  unique
  nonunique
  conjuncts
  var-names)

(defun make-csp (var-domains conjuncts prop-domains deps)
  "make-csp VAR-DOMAINS CONJUNCTS PROP-DOMAINS FUNCTIONAL-DEPENDENCIES

VAR-DOMAINS - association list from variable names to sets
CONJUNCTS - list of propositions, which may have free variables
PROP-DOMAINS - list of items of the form (PROP-NAME SET1 ... SET_k), one for each proposition symbol occurring among the conjuncts
FUNCTIONAL-DEPENDENCIES - an association list from proposition symbols to lists of integers.  

At a given state, finding the set of instantiations to the variables that make the propositions true is a CSP.  This function precomputes information to solve that CSP faster using the given FUNCTIONAL-DEPENDENCIES, which is a list, where each element looks like (NAME I1 ... Ik) which means that in any legal state, given values to arguments numbered I1 ... Ik, there is at most one instantiation to the remaining arguments that will make the predicate be true.

The return value is basically a recipe for solving the csp efficiently.  It is a structure consisting of
1) A set of partial bindings for the nonunique variables
2) A list of items of the form (PROP . DET) where PROP is a proposition and DET is a set of partial bindings that need to be searched over
3) The original list of conjuncts
4) The variable names

The meaning is that for every joint instantiation of the variables in 1), you can figure out the rest of the variables deterministically using 2).  The complexity will therefore be exponential in the length of 1)."
  
  ;; Each variable value must be an allowed value for that variable as well as for the position of each proposition-argument that equals that variable
  (let ((var-domains
	 (mapcar
	  #'(lambda (pair)
	      (dsbind (var . vals) pair
		(cons var
			(dolist (c conjuncts vals)
			  (do-elements (a (prop-args c) nil i)
			    (when (eq a var)
			      (setf vals (intersect vals (elt (sets (evaluate prop-domains (prop-symbol c))) i)))))))))
	  var-domains))
	(vars (mapcar #'car var-domains)))
    
    (flet ((lookup (var) (evaluate var-domains var)))

      ;; iterate over subset sizes starting from 0
      (dotimes (i (1+ (length vars)))
	
	;; iterate over sets of this size
	(do-elements (s (subsets-of-size vars i))
	  
	  ; (format t "~&Considering set ~a" s)
	  
	  (let ((l s)
		(props nil)
		(undetermined (copy-list conjuncts)))
	    
	    (labels ((var-is-known (v)
		       (or (member v l)
			   (not (member v vars))))
		     (find-applicable-dependency (prop)
		       (find-if 
			#'(lambda (dep)
			    (dsbind (name . positions) dep
			      (and (eq name (prop-symbol prop))
				   (every #'var-is-known 
					  (mapcar #'(lambda (i) (nth i (prop-args prop)))
						  positions)))))
			deps)))
	      
	    
	      ;; repeatedly try to find a proposition whose arguments are uniquely determined
	      (loop
		
		; (format t "~&l = ~a, props = ~a, undetermined = ~a" l props undetermined)
	      
		(unless
		    ;; iterate over conjuncts that are as yet undetermined
		    (dolist (c undetermined nil)
		      
		      ; (format t "~&Considering conjunct ~a" c)
		
		      ;; If some dependency applies, add it
		      (awhen (find-applicable-dependency c)
			     
			     ; (format t "~&Found applicable dependency ~a" it)
			     
			     (let ((det nil))
			       (do-elements (v (prop-args c) nil i)
				 (if (member i (cdr it))
				     (assert (var-is-known v))
				   (progn 
				     (when (member v vars)
				       (unless (member v l)
					 (push v l)
					 (push (cons v (lookup v)) det))))))
			       (setf props (nconc props 
						  `((,c . ,(make-instance '<prod-set>
							     :inst-acc (inst-vars:make-alist-accessors (mapcar #'car det))
							     :sets (mapcar #'cdr det)
							     :iterate-quickly t))))
				     undetermined (delete c undetermined :test #'equal)))
			   
			     ;; return from the dolist with value t
			     (return t)))
		
		  ;; if we're here, that means we couldn't find any more uniquely determined vars
		  (if (set-eq l vars)

		      ;; If all vars have been determined, return from the function
		      (return-from make-csp
			(create-csp
			 :nonunique (make-instance '<prod-set> :sets (mapcar #'lookup s)
						:inst-acc (inst-vars:make-alist-accessors s) :iterate-quickly t) 
			 :unique props 
			 :conjuncts conjuncts 
			 :var-names vars)) 
		    
		    ;; Otherwise, move on to the next set
		    (return))))
	      ))))
      
      (error "Should not reach this point."))))
	    
		  
(defun solve-csp (csp s)
  "solve-csp CSP S
CSP is a csp generated by make-csp and S is a propositional state.  Return the set of lists of values of the variables (in the order that they were given to make-csp) that satisfy the CSP at this state."
  
  (let ((conjuncts (csp-conjuncts csp))
	(vars (csp-var-names csp))
	(unique (csp-unique csp)))
    (let ((solutions nil))
    
      ;; Iterate over values of nonunique variables
      (do-elements (nonunique-bindings (csp-nonunique csp) solutions)
	(let ((bindings nonunique-bindings))
	  
	
	  ;; Now figure out values of the uniquely determined variables
	  ;; If the loop terminates normally, the new binding is pushed on to the solution list
	  (dolist (det-var unique (when (holds s (bind (conjoin-set conjuncts) bindings)) 
				    (push (mapcar #'(lambda (v) (evaluate bindings v)) vars) solutions)))
	    
	    (dsbind (prop . vals) det-var
	      (let* ((prop (bind prop bindings))
		     (prop-bindings (do-elements (bindings vals 'unsuccessful)
				      (when (holds s (bind prop bindings))
					(return bindings)))))
		
		(if (eq prop-bindings 'unsuccessful)
			
		    ;; if we every can't find a value for a uniquely determined variable, 
		    ;; skip to the next binding for a nonunique variable
		    (return nil)
			
		  ;; Otherwise add the binding and continue
		  (dolist (b prop-bindings) (push b bindings)))))))))))
	  


(defun solve-csp-conjunction-complete (csp clause domain)
  "SOLVE-CSP-CONJUNCTION-COMPLETE CSP DNF-CLAUSE DOMAIN

CSP is a csp generated by make-csp and S is a propositional state.  Return the set of lists of values of the variables (in the order that they were given to make-csp) that satisfy the CSP at some state consistent with DNF-CLAUSE."
  (let ((nonunique (csp-nonunique csp))
	(unique (csp-unique csp))
	(conjuncts (csp-conjuncts csp))
	(vars (csp-var-names csp)))
    (let ((solutions (make-hash-table :test #'equal))
	  (precond (conjoin-set conjuncts))
	  (literals (conjuncts clause)))
      
      (labels ((helper (bindings unique-remaining)
		 ;; bindings = bindings thus far
		 ;; unique-remaining = props that will be used to determine remaining variables
		 
		 (aif (pop unique-remaining)
		     
		      ;; If there are some variables left to be determined, get the next proposition to be used
		      (dsbind (prop . det) it
			
			(unless (is-empty det)
			;; first, apply any existing bindings
			(let ((prop (bind prop bindings))
			      (names (mapcar #'car (item 0 det)))
			      (sets (sets det)))
		       
			  (ecase (prop-type domain prop)
			 
			    ;; 1. If it's a fluent, set up functions to match/bind extensions of it in the clause
			    (fluent		
			     (mvbind (checker binder) (prop-extension (bind prop bindings) names)

			       (mvbind (elt exists?)
				   (find-element literals checker)  
				 (if exists?
			       
				     ;; if there's a proposition in the clause that is an extension of prop, get its bindings
				     (let ((new-bindings (funcall binder elt)))
				       
				       ;; when they satisfy the type constraints, apply them and move on.  Otherwise stop.
				       (when (every #'(lambda (name dom) (member? (evaluate new-bindings name) dom)) names sets)
					 (helper (append (funcall binder elt) bindings) unique-remaining)))
			     
				   ;; Otherwise, we must recurse on all possible values of this variable
				   (do-elements (v det)
				     (let ((bound-prop (bind prop v)))
				       (unless (find-element literals #'(lambda (lit) (and (typep lit 'negation) (equal (negatee lit) bound-prop))))
					 (helper (append v bindings) unique-remaining))))))))
			 
			    ;; 2. If it's a nonfluent, it must be known in the background knowledge
			    (nonfluent
			     (mvbind (binding exists?)
				 (find-element det #'(lambda (binding) (holds-background domain (bind prop binding))))
			       (when exists?
				 (helper (append binding bindings) unique-remaining))))))))
		       
		     
		   
		      ;; Otherwise we've determined all the variables.  Check if consistent, then add to solutions
		      (let ((ground-precond (bind precond bindings)))
			(when (and (dnf-consistent clause ground-precond)
				   (every #'(lambda (prop)
					      (or (eq (prop-type domain prop) 'fluent)
						  (holds-background domain prop)))
					  (conjuncts ground-precond)))
			  (setf (gethash (mapcar #'(lambda (v) (evaluate bindings v)) vars) solutions) t))))))
    
	(do-elements (nonunique-bindings nonunique (hash-keys solutions))
	  (helper nonunique-bindings unique))))))
      
  


(defun solve-csp-complete (csp s)
  "solve-csp-complete CSP PROP-STATE-SET  Return the set of assignments that satisfy the CSP in at least one of the states in PROP-STATE-SET."
  (let ((clauses (disjuncts (formula s)))
	(d (pss-domain s))
	(l nil))
    ;; It's just a union over the clauses
    (dolist (clause clauses l)
      (dolist (assignment (solve-csp-conjunction-complete csp clause d))
	(adjoinf l assignment :test #'equal)))))
	
      
    
    
  

(defun prop-extension (prop names)
  "Given a proposition and the set of unspecified variables, return 1) a function that checks if another proposition agrees with this one on all but the unspecified variables 2) a function that takes a ground proposition satisfying 1) and returns bindings for the unspecified variables"
  (let ((l nil)
	(l2 nil)
	(args (prop-args prop))
	(symb (prop-symbol prop)))
    (do-elements (a args nil i)
      (if (member a names)
	  (push (cons a i) l2)
	(push i l)))
    (values
     
     ;; fn that checks if a ground prop is an extension of prop
     #'(lambda (ground-prop)
	  (and  (typep ground-prop '(not negation))
	   (eq (prop-symbol ground-prop) symb)
	      (let ((ground-args (prop-args ground-prop)))
		(every #'(lambda (i) (eq (nth i ground-args) (nth i args))) l))))
     
     ;; fn that extracts bindings from such an extension
     #'(lambda (ground-prop)
	 (let ((args (prop-args ground-prop)))
	   (mapcar #'(lambda (pair)
		       (cons (first pair) (elt args (cdr pair))))
		   l2))))))

