;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; lookahead/prop/prop-domain.lisp
;; Define a class of deterministic, fully observable planning domain whose state
;; consists of the truth values of a fixed set of propositions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <propositional-domain> (<planning-problem>)
  ((goal
   :reader goal
   :type [set]
   :writer set-goal)
   (init-state :accessor init-state)
   
  (types
   :reader types
   :initarg :types
   :type [mapping])
  (object-types :accessor object-types :type hash-table :initarg :object-types :initform (make-hash-table))
  (propositions :accessor propositions :type [set])
  (prop-types :accessor prop-types :type list)
  (prop-domains :accessor prop-domains)
  (background-table :accessor background-table :type list)
  (action-descs :accessor action-descs :type list)
  (functional-dependencies :accessor functional-deps :initarg :functional-deps :initform nil)
  (csps :accessor csps :type list)
  (all-actions :accessor all-actions)
  (init-state-set :accessor init-state-set)
  (pprint-state-fn :accessor pprint-state-fn :initarg :pprint-state-fn)
  )
  (:documentation "Class <propositional-domain> (<planning-problem>).  Represents a propositional planning problem. 

Required initargs 
:types - an association list mapping type names, which are symbols, to sets.  The fluents, nonfluents, and action-descs arguments will use these names to refer to the types in the domain.
:fluents, :nonfluents - lists of items of the form (PRED-NAME TYPE_1 ... TYPE_k), where PRED_NAME and each TYPE_i are symbols.  The meaning is that the set of ground propositions for this domain are of the form PRED ARG1 ... ARG_k where there is a corresponding element for PRED in the fluents or nonfluents list, and each ARG_i belongs to the set named by TYPE_i (using the types list specified above).
:goal - the goal formula
:init-state - the set of ground propositions (both fluent and nonfluent) that are true in the initial state
:action-descs - association list mapping action names, which are symbols, to lists of the form (VAR-DOMAINS PRECOND ADD-LIST DELETE-LIST).  VAR-DOMAINS is an association list from action argument names to type names.  PRECOND is a conjunction of literals.  The add and delete lists are lists of propositions.  The precond, add and delete lists may contain references to the variables in VAR-DOMAINS.

Optional
:functional-deps - Heuristic information that will be used to speed up the search for available actions in a state.  This should be a list, where each element looks like (PRED-NAME &rest ARG-NUMS).  The meaning is that, in any legal state, for any instantiation to the variables referred to by ARG-NUMS, there is at most one instantiation to the remaining variables that makes the given proposition hold.  For example, an element (C 2 0) implies that there is at most one value of ?x that makes (C 'foo ?x 'bar) hold in a given state.  Defaults to nil.
:pprint-state-fn - function that takes in STREAM and PROP-DOMAIN-STATE and pretty-prints the state to the stream.

Note that this class is a descendant of <env>, so all operations that can be done on general environments (e.g., reinforcement learning) can be done on its instances."))

(defstruct (prop-domain-state (:conc-name pds-))
  domain
  props)

(defmethod same-state ((s1 prop-domain-state) s2)
  (declare (ignore s2))
  nil)

(defmethod same-state (s1 (s2 prop-domain-state))
  (declare (ignore s1))
  nil)

(defmethod same-state ((s1 prop-domain-state) (s2 prop-domain-state))
  (and (eq (pds-domain s1) (pds-domain s2))
       (set-eq (pds-props s1) (pds-props s2))))


(defclass <prop-state-set> (<image-set>)
  ((domain :initarg :domain :accessor pss-domain)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; constructors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod initialize-instance :after ((d <propositional-domain>) &rest args 
				       &key init-state goal fluents nonfluents
					    action-descs (functional-deps nil))
  (declare (ignore args))
  (check-type goal [formula])
  (check-type init-state [set])
  
  (let ((prop-domains (mapcar #'(lambda (tuple) 
				  (cons (first tuple)
					(apply #'make-prop-set
					       (first tuple)
					       (mapcar #'(lambda (x) (lookup-type d x)) (cdr tuple)))))
			      (append fluents nonfluents))))
    (setf (prop-domains d) prop-domains)
    
    ;; Add prop type info
    (setf (prop-types d) 
      (append (mapcar #'(lambda (p) (cons (first p) 'fluent)) fluents)
	      (mapcar #'(lambda (p) (cons (first p) 'nonfluent)) nonfluents)))

    ;; Use only the fluents in the state description
    (setf (init-state d) (make-prop-domain-state 
			  :domain d 
			  :props (filter 
				  (case *compound-formula-type*
				    (list ':list)
				    (hash-table ':equal-hash-table))
				  init-state #'(lambda (x) (eq (prop-type d x) 'fluent)))))
    
    (let ((prop-set 
	   ;; the set of propositions is the disjoint union of the sets of
	   ;; propositions corresponding to ground instances of each proposition symbol
	   (disjoint-union-list (mapcar #'cdr (filter ':list prop-domains #'(lambda (x) (eq (prop-type d x) 'fluent)))))))
      (setf (propositions d) prop-set)
      (set-goal (make-prop-state-set d goal) d))
    
    
    

    
    ;; background prop tables
    (let ((table
	   (setf (background-table d)
	     (mapcar #'(lambda (entry)
			 (let ((s (car entry)))
			   (let ((sets (sets (evaluate prop-domains s))))
			     (list s sets (make-array (map 'list #'size sets) :element-type 'boolean :initial-element nil)))))
		     nonfluents))))
      (do-elements (prop init-state)
	(when (eq (prop-type d prop) 'nonfluent)
	  (dsbind (sets a)  (evaluate table (prop-symbol prop))
	    (setf (apply #'aref a (map 'list #'(lambda (x s) (item-number x s)) (prop-args prop) sets)) t)))))
      
			
    
    ;; go through action-descs and substitute the type names with the sets,
    ;; then create a strips schema for each one
    (setf (action-descs d)
      (mapcar #'(lambda (entry)
		  (dsbind (name domains precond adds dels &optional (reward -1)) entry
		    (cons name
			  (make-strips-schema 
			   :domain d
			   :domains (mapcar #'(lambda (entry)
						(cons (car entry)
						      (lookup-type d (cdr entry))))
					    domains)
			   :precond precond
			   :add-list adds
			   :reward reward
			   :delete-list dels))))
	      action-descs))

    (setf (all-actions d)
    (apply 
   #'disjoint-union 
   (mapcar 
    #'(lambda (desc)
	(dsbind (name . schema) desc
	  (let ((domains (mapcar #'cdr (domains schema))))
	    (make-image-set
	     (apply #'direct-product 'list domains)
	     #'(lambda (x) (cons name x))
	     #'(lambda (x) (if (consp x) (rest x) 'not-cons))))))
    (action-descs d))))

    (setf (init-state-set d) (make-state-set d (init-state d)))
    
    (dolist (dep functional-deps)
      (setf (cdr dep) (sort (cdr dep) #'<)))
      
    ;; use the functional dependency information, if any, to figure out an order
    ;; to search for applicable instances of a given action
    (setf (csps d)
      ;; loop over action schemas
      (mapcar #'(lambda (entry)
		  (dsbind (name . schema) entry
		    (cons name
			  (make-csp 
			   (domains schema)
			   (conjuncts (precond (desc schema)))
			   prop-domains
			   functional-deps))))
	      (action-descs d)
	      ))))
      

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; methods for prop-domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod reward ((d <propositional-domain>) s a)
  (declare (ignore s))
  (nstrips-reward (strips-effect (primitive-action-description d a))))

(defmethod avail-actions ((d <propositional-domain>) (s prop-domain-state))
  (let ((l nil))
    
    ;; loop over action schemas
    (do-entries (name schema (action-descs d) l)
      (_f nconc l
	  (mapcar #'(lambda (args) (cons name args))
		  (solve-csp (evaluate (csps d) name) s)))
      )))

(defmethod primitive-action-description ((d <propositional-domain>) a)
  "primitive-action-description DOMAIN ACTION-DESIGNATOR"
  (condlet
   (((symbolp a) (name a) (args nil))
    ((listp a) (name (first a)) (args (rest a))))
   (instantiate (evaluate (action-descs d) name) args)))


(defmethod make-state-set ((d <propositional-domain>) s)
  (make-prop-state-set d (make-state-dnf (pds-props s) (propositions d))))

(defun holds-background (d formula)
  (typecase formula
    (boolean formula)
    (conjunction (every #'(lambda (c) (holds-background d c)) (conjuncts formula)))
    (disjunction (some #'(lambda (c) (holds-background d c)) (disjuncts formula)))
    (negation (not (holds-background d (negatee formula))))
    (t (let ((name (prop-symbol formula))
		 (args (prop-args formula)))
	     (dsbind (sets table) (evaluate (background-table d) name)
	       (apply #'aref table (map 'list #'(lambda (x s) (item-number x s)) args sets)))))))

(defun pd-object-type (d obj)
  (gethash obj (object-types d)))

(defun action-name (a)
  (etypecase a
    (symbol a)
    (list (car a))))

(defun action-args (a)
  (etypecase a
    (symbol nil)
    (list (cdr a))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; methods for prop-domain-states
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod holds ((s prop-domain-state) formula)
  (let ((d (pds-domain s)))
    (typecase formula
      (boolean formula)
      (conjunction (every #'(lambda (c) (holds s c)) (conjuncts formula)))
      (disjunction (some #'(lambda (c) (holds s c)) (disjuncts formula)))
      (negation (not (holds s (negatee formula))))
      (t (ecase (prop-type d formula)
	   (fluent (holds (pds-props s) formula))
	   (nonfluent (holds-background d formula)))))))


(defun pprint-prop-domain-state (str s)
  (aif (pprint-state-fn (pds-domain s))
       (funcall it str s)
       (pprint (pds-props s) str)))


(set-pprint-dispatch 'prop-domain-state #'pprint-prop-domain-state)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; csp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-and-solve-csp (dom conjuncts var-domains s)
  "DOM is of type <prop-domain>.  CONJUNCTS is a list of propositions with free variables.  VAR-DOMAINS is an association from those free variables to sets (or to symbols, which are taken to name types in DOM.  In addition, each proposition has a set of allowed values in each slot, determined by DOM.  S is an optimistic (complete) set.  This determines a csp to find the assignments to the variables that make the conjuncts true.  This function returns the list of solutions, each of which is a list of variable values."
  (let ((actual-domains (mapcar #'(lambda (pair) (cons (car pair) (let ((vals (cdr pair))) (if (symbolp vals) (lookup-type dom vals) vals)))) var-domains)))
    (solve-csp-complete (make-csp actual-domains conjuncts (prop-domains dom) (functional-deps dom)) s)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun lookup-type (d name)
  (mapping:evaluate (types d) name))

(defun prop-type (d prop)
  (mapping:evaluate (prop-types d) (prop-symbol prop)))


(defun make-prop-state-set (d formula)
  (make-instance '<prop-state-set>
    :s (make-instance '<dnf-set> :formula formula :props (propositions d))
    :f #'(lambda (x) (make-prop-domain-state :domain d :props x))
    :equality-test #'same-state
    :f-inv #'pds-props
    :domain d))

(defmethod formula ((s <prop-state-set>))
  (formula (base-set s)))

(defmethod intersect ((s <prop-state-set>) (s2 <prop-state-set>))
  (let ((d (pss-domain s))
	(d2 (pss-domain s2)))
    (assert (eq d d2) nil
      "Can't intersect prop-state-sets with different domains ~a and ~a"
      d d2)
    (make-instance '<prop-state-set>
      :s (intersect (base-set s) (base-set s2))
      :f #'(lambda (x) (make-prop-domain-state :domain d :props x))
      :f-inv #'pds-props
      :domain d)))

(defmethod is-empty ((s <prop-state-set>))
  (is-empty (base-set s)))

(defmethod member? :around (x (s <prop-state-set>))
  (when (prop-domain-state-p x)
    (call-next-method)))
    

(defmethod print-object ((s <prop-state-set>) str)
  (print-unreadable-object (s str :type t :identity nil)
    (format str "with formula ~a" (formula s)))) 

(defun size-exceeds (s k)
  (size-exceeds (base-set s) k))

(defmethod set-eq ((s null) (s2 <prop-state-set>))
  (is-empty s2))

(defmethod set-eq ((s <prop-state-set>) (s2 null))
  (is-empty s))

(defmethod set-eq ((s <prop-state-set>) (s2 <prop-state-set>))
  (set-eq (base-set s) (base-set s2)))

(defun pprint-pss-true-and-unknown (&rest args)
  (bind-pprint-args (str pss) args
    (let ((props (props (base-set pss))))
      (pprint-logical-block (str nil :suffix "]")
	(format str "[Prop state set with clauses:~2I")
	(do-elements (c (disjuncts (formula pss)))
	  (pprint-pop)
	  (pprint-newline :mandatory str)
	  (apply #'pprint-pss-clause (car args) (cons c props)  (cddr args)))))))

(defun pprint-pss-clause (&rest args)
  (bind-pprint-args (str x) args
    (pprint-logical-block (str nil :prefix "[" :suffix "]")
      (dsbind (c . props) x
	(let ((true nil)
	      (unknown nil)
	      (con (conjuncts c)))
	  (if (member? nil con)
	      (format str "Empty set")
	    (progn 
	      (do-elements (p props)
		(cond
		 ((member? p con) (push p true))
		 ((not (member? (negate p) con)) (push p unknown))))
	      (format str "True: ~a~:@_Unknown: ~a" true unknown))))))))

(set-pprint-dispatch '<prop-state-set> #'pprint-pss-true-and-unknown)