(in-package nav-switch)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Exact descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ns-descs (d)
  (make-ncstrips-schemas d
    (act :var-domains () :effects ())
    (nav-goal 
     :var-domains ((?xs columns) (?ys rows) (?xg columns) (?yg rows))
     :effects ((:precond 
		(and (int= ?xs ?xg) (int= ?ys ?yg))
		:reward 0)
	       
	       (:precond
		(and (horiz))
		:reward (horiz-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))
	       
	       (:precond
		(and (vert))
		:reward (vert-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))))
    
    (nav-goal2 
     :var-domains ((?xs columns) (?ys rows) (?xg columns) (?yg rows))
     :effects ((:precond 
		(and (int= ?xs ?xg) (int= ?ys ?yg))
		:reward 0)
	       
	       (:precond
		(and (horiz))
		:reward (horiz-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))
	       
	       (:precond
		(and (vert))
		:reward (vert-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))))
    
    (nav-switch
     :var-domains ((?xs columns) (?ys rows) (?xg columns) (?yg rows))
     :effects ((:precond
		(and (int= ?xs ?xg) (int= ?ys ?yg) (vert))
		:add-list ((horiz))
		:delete-list ((vert))
		:reward -1)
	       
	       (:precond
		(and (int= ?xs ?xg) (int= ?ys ?yg) (horiz))
		:add-list ((vert))
		:delete-list ((horiz))
		:reward -1)
	       
	       (:precond
		(and (horiz))
		:reward (horiz-reward-flip ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg) (vert))
		:delete-list ((at ?xs ?ys) (horiz)))
	       
	       (:precond
		(and (vert))
		:reward (vert-reward-flip ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg) (horiz))
		:delete-list ((at ?xs ?ys) (vert)))))
	       
	       
			 
    (nav 
     :var-domains ((?xs columns) (?ys rows) (?xg columns) (?yg rows))
     :effects ((:precond 
		(and (at ?xs ?ys) (int= ?xs ?xg) (int= ?ys ?yg))
		:reward 0)
	       
	       (:precond 
		(and (at ?xs ?ys) (horiz))
		:reward (horiz-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))
	       
	       (:precond
		(and (at ?xs ?ys) (vert))
		:reward (vert-reward ?xs ?ys ?xg ?yg ?set)
		:add-list ((at ?xg ?yg))
		:delete-list ((at ?xs ?ys)))))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Subsumption check key
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun subsumption-key (s)
  (if (listp s)
      s
    (let (loc dir)
      (do-elements (l (conjuncts (item 0 (disjuncts (formula s)))))
	(unless (typep l 'negation)
	  (ecase (prop-symbol l)
	    (at (setf loc (prop-args l)))
	    (vert (setf dir 'v))
	    (horiz (setf dir 'h)))))
      (list loc dir))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Lower and upper heuristics for Act
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun ns-act-complete (cset &aux (d (pss-domain cset)))
  (let ((goal (prop-args (first (conjuncts (formula (goal d)))))))
    (* (good-move-cost d) (neg-manhattan-dist goal (get-loc cset)))))

(defun ns-act-sound (sset &aux (d (pss-domain sset)))
  (let ((goal (prop-args (first (conjuncts (formula (goal d)))))))
    (* (bad-move-cost d) (neg-manhattan-dist goal (get-loc sset)))))


  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *dummy-reward-for-navigating-in-place* -1e-6)
;; A hack, put in so that nav actions that have the same start and goal will be subsumed away

(defun horiz-reward (xs ys xg yg s)
  (min
   (let ((d (pss-domain s)))
     (+ (* (good-move-cost d) (- (abs-diff xs xg)))
	(* (bad-move-cost d) (- (abs-diff ys yg)))))
   *dummy-reward-for-navigating-in-place*))

(defun vert-reward (xs ys xg yg s)
  (min
   (let ((d (pss-domain s)))
     (+ (* (bad-move-cost d) (- (abs-diff xs xg)))
	(* (good-move-cost d) (- (abs-diff ys yg)))))
   *dummy-reward-for-navigating-in-place*))

(defun horiz-reward-flip (xs ys xg yg s)
  (1- (horiz-reward xs ys xg yg s)))

(defun vert-reward-flip (xs ys xg yg s)
  (1- (vert-reward xs ys xg yg s)))

(defun get-loc (s)
  (prop-args
   (find-element (conjuncts (item 0 (disjuncts (formula s))))
		 #'(lambda (p) (eq (prop-symbol p) 'at)))))

(defun neg-manhattan-dist (l1 l2)
  (- (+ (abs-diff (first l1) (first l2))
	(abs-diff (second l1) (second l2)))))
	
      
  
  