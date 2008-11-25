(in-package lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <prop-hierarchy> (<hierarchy>)
  ((hla-schemas :type [mapping] :initarg :hla-schemas :accessor hla-schemas :initform nil)
   (refinement-levels :accessor ref-levels :initform nil)
   (complete-refinable :initarg :complete-refinable :accessor complete-refinable)
   (top-level-action-names :initarg :top-level-actions :accessor top-level-action-names))
  (:documentation "Class <prop-hierarchy> (<hierarchy>)

Initargs (in addition to those of <hierarchy>)
:hla-schemas - [mapping] from high-level action name to hla-schema
"))


(defstruct (hla-schema (:conc-name hs-))
  var-domains
  (top-level-precond nil)
  implementations
  csp)
  
  
(defstruct (implementation (:conc-name imp-) (:constructor create-imp))
  csp
  var-names
  arg-bindings
  (precond '(and))
  (state-precond '(and))
  actions)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Constructors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod initialize-instance :after ((h <prop-hierarchy>) &rest args &key hla-schemas (refinement-order nil))
  ;; create the set of all high-level actions
  (declare (ignore args))
  (setf (high-level-actions h)
    (disjoint-union-list
     (let ((l nil))
       (mapping:do-entries (name schema hla-schemas l)
	 (push
	  (make-image-set 
	   (apply #'direct-product 'list (mapcar #'cdr (hs-var-domains schema)))
	   #'(lambda (x) (cons name x))
	   #'rest)
	  l)))))
  
  ;; set refinement levels
  (dolist (pair hla-schemas)
    (let ((name (car pair)))
      (push (cons name (or (position-if #'(lambda (l) (member? name l)) refinement-order) (length refinement-order)))
	    (ref-levels h))))
  (dolist (pair (action-descs (planning-problem h)))
    (push (cons (car pair) 'infty) (ref-levels h)))
		
  
  ;; precompute information for determining set of available top-level actions at a state set
  ;; and for determining applicable refinements
  (let ((d (planning-problem h)))
    (do-entries (name schema (hla-schemas h))
      (setf (hs-csp schema)
	(make-csp (hs-var-domains schema) (conjuncts (hs-top-level-precond schema))
		  (prop-domains d) (functional-deps d)))
      (do-elements (imp (hs-implementations schema))
	(setf (imp-csp imp)
	  (make-csp (map 'list #'cons (imp-var-names imp) (sets (imp-arg-bindings imp)))
		    (append (conjuncts (imp-precond imp))
			    (conjuncts (imp-state-precond imp)))
		    (prop-domains d)
		    (functional-deps d)))))))

(defun make-flat-prop-hierarchy (d)
  "make-flat-prop-hierarchy PROP-DOMAIN"
  (make-instance '<prop-hierarchy> 
    :planning-problem d
    :top-level-actions (domain (action-descs d))))

(defun make-implementation (&key var-domains (precond '(and)) actions (state-precond '(and)))
  "make-implementation VAR-DOMAINS (PRECOND '(and)) ACTIONS (STATE-PRECOND '(and))"
  (let ((names (mapcar #'car var-domains)))
    (create-imp :var-names names
		:arg-bindings (apply #'direct-product names (mapcar #'cdr var-domains))
		:precond precond
		:state-precond state-precond
		:actions actions)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Methods from <hierarchy>
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod refinements ((h <prop-hierarchy>) a)
  (let* ((schema (mapping:evaluate (hla-schemas h) (car a)))
	 (bindings (mapcar #'cons (mapcar #'car (hs-var-domains schema)) (cdr a)))
	 (prob (planning-problem h)))
    (assert (eq (action-type a h) 'high-level) ()
      "~a is not a high-level action in ~a.  Its type is ~a" a h (action-type a h))
    (disjoint-union-list
     (mapcar 
      #'(lambda (imp)
	  (let ((actions (imp-actions imp))
		(arg-bindings (imp-arg-bindings imp)))

	    (let ((s
	    (set:make-image-set 
	     (filter ':list arg-bindings
		     #'(lambda (args)
			 (holds-background prob (bind (imp-precond imp) (append args bindings)))))
	     #'(lambda (args)
		 (let ((b (append args bindings)))
		   (map 'vector #'(lambda (a) (prop-logic:bind a b)) actions)))
	     )))
	      (set-equality-test #'equalp s)
	      s)))
      (hs-implementations schema)))))



(defmethod applicable-refinements ((h <prop-hierarchy>) a hset)
  
  (let* ((schema (mapping:evaluate (hla-schemas h) (car a)))
	 (bindings (mapcar #'cons (mapcar #'car (hs-var-domains schema)) (cdr a))))
    (assert (eq (action-type a h) 'high-level) ()
      "~a is not a high-level action in ~a.  Its type is ~a" a h (action-type a h))
    
    ;; Union over implementations
    (disjoint-union-list
     (mapcar 
      #'(lambda (imp)
	  (let ((actions (imp-actions imp))
		(csp (imp-csp imp)))
		  
	    ;; The CSP refers to the variables of a.  These need to be substituted with
	    ;; their (now known) values.
	    (let ((ground-csp
		   (create-csp
		    :nonunique (csp-nonunique csp)
		    :var-names (csp-var-names csp)
		    :conjuncts (mapcar #'(lambda (c) (bind c bindings)) (csp-conjuncts csp))
		    :unique (mapcar #'(lambda (pair)
					(dsbind (prop . det) pair
					  (cons (bind prop bindings) det)))
				    (csp-unique csp)))))
		    
		    
	      (mapcar #'(lambda (sol)
			  (let ((full-bindings (append (mapcar #'cons (csp-var-names csp) sol) bindings)))
			    (map 'vector #'(lambda (a) (bind a full-bindings)) actions)))
		      (solve-csp-complete ground-csp hset)))))
	    
      (hs-implementations schema)))))
		      
		      

		    
(defmethod ref-level ((h <prop-hierarchy>) a)
  (evaluate (ref-levels h) (car a)))
  


(defmethod applicable-top-level-actions ((h <prop-hierarchy>) hset)
  
  (disjoint-union-list
   (mapcar 
    #'(lambda (name)
	(mapcar
	 #'(lambda (args) (cons name args))
	 (solve-csp-complete (get-csp h name) hset)))
    (top-level-action-names h))))
		    
			   
			 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Other
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ground-hla-pred (name)
  "ground-hla-pred NAME.  NAME is a symbol.  Return a function that returns true if its argument is a list beginning with NAME."
  #'(lambda (a) (and (consp a) (eq (car a) name))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-csp (h name)
  "get-csp H NAME.  If NAME names a high-level action, use the csp from the corresponding schema.  If not, assume it's primitive and get the csp from the planning domain."
  (mvbind (schema defined?)
      (evaluate-mv (hla-schemas h) name)
    (if defined?
	(hs-csp schema)
      (evaluate (csps (planning-problem h)) name))))