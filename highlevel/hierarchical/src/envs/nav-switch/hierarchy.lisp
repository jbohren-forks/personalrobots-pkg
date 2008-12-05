(in-package nav-switch)

(defun plan-is-valid (plan)
  (notany #'(lambda (a) (and (listp a)
			     (eq (first a) 'nav-switch)
			     (eql (second a) (fourth a))
			     (eql (third a) (fifth a))))
	  plan))

(defun nav-switch-hierarchy (d)
  (let ((nr (lookup-type d 'rows))
	(nc (lookup-type d 'columns))
	(gc (list (second (goal-prop d))))
	(gr (list (third (goal-prop d))))
	(sc (mapcar #'first (switches d)))
	(sr (mapcar #'second (switches d))))
    (make-instance '<prop-hierarchy>
      :planning-problem d
      :top-level-actions '(nav-goal nav-goal2 nav-switch)
      :refinement-order '((nav-goal nav-goal2 nav-switch) (nav))
      :valid-plan-fn #'plan-is-valid ;; Disallow nav-switch with same start and goal
      :hla-schemas (p2alist 
		    
		    'nav-goal 
		    (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xg . ,gc) (?yg . ,gr))
		     :top-level-precond '(and (at ?xs ?ys) (goal ?xg ?yg) (int-diff ?xs ?xg))
		     :implementations
		     (list (make-implementation :actions '((nav ?xs ?ys ?xg ?yg)))))
		    
		    'nav-goal2
		    (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xg . ,gc) (?yg . ,gr))
		     :top-level-precond '(and (at ?xs ?ys) (goal ?xg ?yg) (int-diff ?ys ?yg))
		     :implementations
		     (list (make-implementation :actions '((nav ?xs ?ys ?xg ?yg)))))
		    
		    'nav-switch
		    (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xg . ,sc) (?yg . ,sr))
		     :top-level-precond '(and (at ?xs ?ys) (switch-at ?xg ?yg))
		     :implementations
		     (list
		      (make-implementation
		       :state-precond '(horiz)
		       :actions '((nav ?xs ?ys ?xg ?yg) (flipv ?xg ?yg)))
		      (make-implementation
		       :state-precond '(vert)
		       :actions '((nav ?xs ?ys ?xg ?yg) (fliph ?xg ?yg)))))
		    
		    'nav
		    (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xg . ,nc) (?yg . ,nr))
		     :implementations 
		     (list
		      (make-implementation 
		       :precond '(and (int= ?xs ?xg) (int= ?ys ?yg))
		       :actions ())
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (r ?x ?xs) (horiz))
		       :actions '((r-good ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (r ?x ?xs) (horiz))
		       :actions '((r-good ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (r ?xs ?x) (horiz))
		       :actions '((l-good ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (r ?xs ?x) (horiz))
		       :actions '((l-good ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (u ?y ?ys) (vert))
		       :actions '((u-good ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (u ?y ?ys) (vert))
		       :actions '((u-good ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (u ?ys ?y) (vert))
		       :actions '((d-good ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (u ?ys ?y) (vert))
		       :actions '((d-good ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (r ?x ?xs) (vert))
		       :actions '((r-bad ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (r ?x ?xs) (vert))
		       :actions '((r-bad ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (r ?xs ?x) (vert))
		       :actions '((l-bad ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?x . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (r ?xs ?x) (vert))
		       :actions '((l-bad ?xs ?ys ?x) (nav ?x ?ys ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (u ?y ?ys) (horiz))
		       :actions '((u-bad ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (u ?y ?ys) (horiz))
		       :actions '((u-bad ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?xs ?xg) (u ?ys ?y) (horiz))
		       :actions '((d-bad ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))
		      (make-implementation 
		       :var-domains `((?y . ,nc))
		       :precond '(and (int-diff ?ys ?yg) (u ?ys ?y) (horiz))
		       :actions '((d-bad ?xs ?ys ?y) (nav ?xs ?y ?xg ?yg)))))))))