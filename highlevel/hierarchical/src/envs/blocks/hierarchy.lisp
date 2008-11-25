(in-package blocks)



(defun plan-valid (plan)
  (not
   (any
    (1- (length plan))
    #'(lambda (i)
	(let ((a1 (elt plan i))
	      (a2 (elt plan (1+ i))))
	  (and (member (action-name a1) '(move-block move-to))
	       (eql (action-name a2) 'move-block)
	       (eql (first (action-args a1)) (first (action-args a2)))))))))



(defun make-unguided-blocks-hierarchy (d)
  (let ((nr (num-rows d))
	 (nc (num-cols d))
	 (blocks (blocks d))
	 (all-blocks (all-blocks d))
	 (top-row (1- (num-rows d))))
    (let ((nav (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		     :implementations
		     (list
		      (make-implementation
		       :precond '(and (int= ?xs ?xt) (int= ?ys ?yt))
		       :actions '())
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and (pos-diff ?xs ?ys ?xt ?yt) (free ?x ?ys) (r ?x ?xs))
		       :actions '((right ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt)(free ?x ?ys) (r ?xs ?x))
		       :actions '((left ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?y ?ys))
		       :actions '((up ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?ys ?y))
		       :actions '((down ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt))))))
	  
	  (navigate (make-hla-schema
		:var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		:implementations
		(list
		 (make-implementation
		  :var-domains ()
		  :precond '(and)
		  :actions '((nav ?xs ?ys ?xt ?yt)))
		 (make-implementation
		  :var-domains `((?x . ,nc))
		  :precond '(and (facel))
		  :actions `((nav ?xs ?ys ?x ,top-row) (turnr ?x) (nav ?x ,top-row ?xt ?yt)))
		 (make-implementation
		  :var-domains `((?x . ,nc))
		  :precond '(and (facer))
		  :actions `((nav ?xs ?ys ?x ,top-row) (turnl ?x) (nav ?x ,top-row ?xt ?yt))))))
	  
	  (move-to (make-hla-schema
		       :var-domains `((?b . ,blocks) (?c . ,all-blocks) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?c ?xc ?yc) (clear ?c) 
					    (gripper-holding ?b) (u ?yt ?yc) (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg))
			 :precond '(and (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg))
			 :precond '(and (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			)))
	  
	  (move-block (make-hla-schema
		       :var-domains `((?b . ,blocks) (?a . ,all-blocks) (?c . ,all-blocks) (?xb . ,nc)
						     (?yb . ,nr) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?b ?xb ?yb) (block-pos ?c ?xc ?yc) (on ?b ?a)
					    (clear ?b) (clear ?c) (gripper-holding nothing) (u ?yt ?yc)
					    (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and (r ?x1 ?xb) (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?x1 ?xb) (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?xb ?x1) (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?xb ?x1) (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))))))
      
      (make-instance '<prop-hierarchy> 
	:planning-problem d
	:valid-plan-fn #'plan-valid
	:top-level-actions '(move-block move-to)
	:refinement-order '((move-block move-to) (navigate) (nav))
	:hla-schemas (p2alist 'navigate navigate 'nav nav 'move-block move-block 'move-to move-to)))))


(defun make-unguided-blocks-hierarchy2 (d)
  (let ((nr (num-rows d))
	 (nc (num-cols d))
	 (blocks (blocks d))
	 (all-blocks (all-blocks d))
	 (top-row (1- (num-rows d))))
    (let ((nav (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		     :implementations
		     (list
		      (make-implementation
		       :precond '(and (int= ?xs ?xt) (int= ?ys ?yt))
		       :actions '())
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and (pos-diff ?xs ?ys ?xt ?yt) (free ?x ?ys) (r ?x ?xs))
		       :actions '((right ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt)(free ?x ?ys) (r ?xs ?x))
		       :actions '((left ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?y ?ys))
		       :actions '((up ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?ys ?y))
		       :actions '((down ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt))))))
	  
	  (navigate (make-hla-schema
		:var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		:implementations
		(list
		 (make-implementation
		  :var-domains ()
		  :precond '(and)
		  :actions '((nav ?xs ?ys ?xt ?yt)))
		 (make-implementation
		  :var-domains ()
		  :precond '(and (facel))
		  :actions `((nav ?xs ?ys ?xs ,top-row) (turnr ?xs) (nav ?xs ,top-row ?xt ?yt)))
		 (make-implementation
		  :var-domains ()
		  :precond '(and (facer))
		  :actions `((nav ?xs ?ys ?xs ,top-row) (turnl ?xs) (nav ?xs ,top-row ?xt ?yt))))))
	  
	  (move-to (make-hla-schema
		       :var-domains `((?b . ,blocks) (?c . ,all-blocks) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?c ?xc ?yc) (clear ?c) 
					    (gripper-holding ?b) (u ?yt ?yc) (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg))
			 :precond '(and (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg))
			 :precond '(and (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			)))
	  
	  (move-block (make-hla-schema
		       :var-domains `((?b . ,blocks) (?a . ,all-blocks) (?c . ,all-blocks) (?xb . ,nc)
						     (?yb . ,nr) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?b ?xb ?yb) (block-pos ?c ?xc ?yc) (on ?b ?a)
					    (clear ?b) (clear ?c) (gripper-holding nothing) (u ?yt ?yc)
					    (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and (r ?x1 ?xb) (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?x1 ?xb) (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?xb ?x1) (r ?x2 ?xc))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
			 :precond '(and  (r ?xb ?x1) (r ?xc ?x2))
			 :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
				    (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))))))
      
      (make-instance '<prop-hierarchy> 
	:planning-problem d
	:valid-plan-fn #'plan-valid
	:top-level-actions '(move-block move-to)
	:refinement-order '((move-block move-to) (navigate) (nav))
	:hla-schemas (p2alist 'navigate navigate 'nav nav 'move-block move-block 'move-to move-to)))))

(defun make-navigateless-hierarchy (d)
  (let ((nr (num-rows d))
	 (nc (num-cols d))
	 (blocks (blocks d))
	 (all-blocks (all-blocks d))
	 (top-row (1- (num-rows d))))
    (let ((nav (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		     :implementations
		     (list
		      (make-implementation
		       :precond '(and (int= ?xs ?xt) (int= ?ys ?yt))
		       :actions '())
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and (pos-diff ?xs ?ys ?xt ?yt) (free ?x ?ys) (r ?x ?xs))
		       :actions '((right ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt)(free ?x ?ys) (r ?xs ?x))
		       :actions '((left ?xs ?ys ?x) (nav ?x ?ys ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?y ?ys))
		       :actions '((up ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and  (pos-diff ?xs ?ys ?xt ?yt) (free ?xs ?y) (u ?ys ?y))
		       :actions '((down ?xs ?ys ?y) (nav ?xs ?y ?xt ?yt))))))
	  
	  
	  
	  (move-to (make-hla-schema
		       :var-domains `((?b . ,blocks) (?c . ,all-blocks) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?c ?xc ?yc) (clear ?c) 
					    (gripper-holding ?b) (u ?yt ?yc) (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg) (facel))
			 :precond '(and (r ?x2 ?xc))
			 :actions '((nav ?xg ?yg ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg) (facer))
			 :precond '(and (r ?x2 ?xc))
			 :actions `((nav ?xg ?yg ?x2 ,top-row) (turnl ?x2) (nav ?x2 ,top-row ?x2 ?yt)
				     (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg) (facer))
			 :precond '(and (r ?xc ?x2))
			 :actions '((nav ?xg ?yg ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			(make-implementation
			 :var-domains `((?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (gripper-pos ?xg ?yg) (facel))
			 :precond '(and (r ?xc ?x2))
			 :actions `((nav ?xg ?yg ?x2 ,top-row) (turnr ?x2) (nav ?x2 ,top-row ?x2 ?yt)
				     (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
			)))
	  
	  
	  
	  (move-block (make-hla-schema
		       :var-domains `((?b . ,blocks) (?a . ,all-blocks) (?c . ,all-blocks) (?xb . ,nc)
						     (?yb . ,nr) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
		       :top-level-precond '(and (block-pos ?b ?xb ?yb) (block-pos ?c ?xc ?yc) (on ?b ?a)
					    (clear ?b) (clear ?c) (gripper-holding nothing) (u ?yt ?yc)
					    (block-diff ?b ?c))
		       :implementations
		       (list
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg) (facel))
			 :precond '(and (r ?x1 ?xb))
			 :actions '((nav ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1) (move-to ?b ?c ?xc ?yc ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg) (facel))
			 :precond '(and (r ?xb ?x1))
			 :actions `((nav ?xg ?yg ?x1 ,top-row) (turnR ?x1) (nav ?x1 ,top-row ?x1 ?yb) 
				    (pickupR ?b ?a ?xb ?yb ?x1) (move-to ?b ?c ?xc ?yc ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg) (facer))
			 :precond '(and (r ?xb ?x1))
			 :actions '((nav ?xg ?yg ?x1 ?yb) (pickupr ?b ?a ?xb ?yb ?x1) (move-to ?b ?c ?xc ?yc ?yt)))
			(make-implementation
			 :var-domains `((?x1 . ,nc) (?xg . ,nc) (?yg . ,nr))
			 :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg) (facer))
			 :precond '(and (r ?x1 ?xb))
			 :actions `((nav ?xg ?yg ?x1 ,top-row) (turnl ?x1) (nav ?x1 ,top-row ?x1 ?yb) 
				    (pickupl ?b ?a ?xb ?yb ?x1) (move-to ?b ?c ?xc ?yc ?yt)))
			))))
      
      (make-instance '<prop-hierarchy> 
	:planning-problem d
	:valid-plan-fn #'plan-valid
	:top-level-actions '(move-block move-to)
	:refinement-order '((move-block move-to) (nav))
	:hla-schemas (p2alist 'nav nav 'move-block move-block 'move-to move-to)))))




(defun make-blocks-hierarchy (d)
  (let ((nr (num-rows d))
	 (nc (num-cols d))
	 (top-row (1- (num-rows d)))
	(blocks (blocks d))
	(all-blocks (all-blocks d)))
    (let ((nav-right (make-hla-schema
		      :var-domains `((?xs . ,nc) (?xt . ,nc) (?y . ,nr))
		      :implementations
		      (list
		       (make-implementation
			:var-domains `((?x . ,nc))
			:precond `(and (r ?x ?xs) (right-of ?xt ?x))
			:actions `((right ?xs ?y ?x) (nav-right ?x ?xt ?y)))
		       (make-implementation
			:var-domains nil
			:precond `(and (int= ?xs ?xt))
			:actions '()))))
	  
	  (nav-left (make-hla-schema
		     :var-domains `((?xs . ,nc) (?xt . ,nc) (?y . ,nr))
		     :implementations
		     (list
		      (make-implementation
		       :var-domains `((?x . ,nc))
		       :precond `(and (r ?xs ?x) (right-of ?x ?xt))
		       :actions `((left ?xs ?y ?x) (nav-left ?x ?xt ?y)))
		      (make-implementation
		       :var-domains nil
		       :precond `(and (int= ?xs ?xt))
		       :actions '()))))
    
	  (nav-up  (make-hla-schema
		    :var-domains `((?x . ,nc) (?ys . ,nr) (?yt . ,nr))
		    :implementations
		    (list
		     (make-implementation
		      :var-domains `((?y . ,nr))
		      :precond `(and (u ?y ?ys) (above ?yt ?y))
		      :actions `((up ?x ?ys ?y) (nav-up ?x ?y ?yt)))
		     (make-implementation
		      :var-domains nil
		      :precond `(and (int= ?ys ?yt))
		      :actions '()))))
	  
	  (nav-down  (make-hla-schema
		      :var-domains `((?x . ,nc) (?ys . ,nr) (?yt . ,nr))
		      :implementations
		      (list
		       (make-implementation
			:var-domains nil
			:precond `(and (int= ?ys ?yt))
			:actions '())
		       (make-implementation
			:var-domains `((?y . ,nr))
			:precond `(and (u ?ys ?y) (above ?y ?yt))
			:actions `((down ?x ?ys ?y) (nav-down ?x ?y ?yt)))
		       )))
	  
	  (navigate (make-hla-schema
		     :var-domains `((?xs . ,nc) (?ys . ,nr) (?xt . ,nc) (?yt . ,nr))
		     :implementations
		     (list
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and (right-of ?xt ?xs) (above ?y ?ys)
				  (above ?y ?yt))
		       :actions '((nav-up ?xs ?ys ?y) (nav-right ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond `(and (right-of ?xt ?xs) (above ?y ?ys) (int= ?y ,top-row) (above ?y ?yt))
		       :state-precond '(faceL)
		       :actions '((nav-up ?xs ?ys ?y) (turnR ?xs) (nav-right ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond `(and (right-of ?xt ?xs) (above ?y ?ys) (int= ?y ,top-row) (above ?y ?yt))
		       :state-precond '(faceR)
		       :actions '((nav-up ?xs ?ys ?y) (turnl ?xs) (nav-right ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond '(and (right-of ?xs ?xt) (above ?y ?ys) (above ?y ?yt))
		       :actions '((nav-up ?xs ?ys ?y)  (nav-left ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond `(and (right-of ?xs ?xt) (above ?y ?ys) (int= ?y ,top-row) (above ?y ?yt))
		       :state-precond '(faceL)
		       :actions '((nav-up ?xs ?ys ?y) (turnR ?xs) (nav-left ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      (make-implementation
		       :var-domains `((?y . ,nr))
		       :precond `(and (right-of ?xs ?xt) (above ?y ?ys) (int= ?y ,top-row) (above ?y ?yt))
		       :state-precond '(faceR)
		       :actions '((nav-up ?xs ?ys ?y) (turnl ?xs) (nav-left ?xs ?xt ?y) (nav-down ?xt ?y ?yt)))
		      )))
	  
	  (move-block (make-hla-schema
				:var-domains `((?b . ,blocks) (?a . ,all-blocks) (?c . ,all-blocks) (?xb . ,nc)
							      (?yb . ,nr) (?xc . ,nc) (?yc . ,nr) (?yt . ,nr))
				:top-level-precond '(and (block-pos ?b ?xb ?yb) (block-pos ?c ?xc ?yc) (on ?b ?a)
						     (clear ?b) (clear ?c) (gripper-holding nothing) (u ?yt ?yc)
						     (block-diff ?b ?c))
				:implementations
				(list
				 (make-implementation
				  :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
				  :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
				  :precond '(and (r ?x1 ?xb) (r ?x2 ?xc))
				  :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
					     (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
				 (make-implementation
				  :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
				  :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
				  :precond '(and  (r ?x1 ?xb) (r ?xc ?x2))
				  :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupL ?b ?a ?xb ?yb ?x1)
					     (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))
				 (make-implementation
				  :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
				  :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
				  :precond '(and  (r ?xb ?x1) (r ?x2 ?xc))
				  :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
					     (navigate ?x1 ?yb ?x2 ?yt) (stackL ?b ?c ?xc ?yc ?x2 ?yt)))
				 (make-implementation
				  :var-domains `((?x1 . ,nc) (?x2 . ,nc) (?xg . ,nc) (?yg . ,nr))
				  :state-precond '(and (free ?x1 ?yb)  (gripper-pos ?xg ?yg))
				  :precond '(and  (r ?xb ?x1) (r ?xc ?x2))
				  :actions '((navigate ?xg ?yg ?x1 ?yb) (pickupR ?b ?a ?xb ?yb ?x1)
					     (navigate ?x1 ?yb ?x2 ?yt) (stackR ?b ?c ?xc ?yc ?x2 ?yt)))))))
						   
						   
							     
							     
				  
		      
				  
      
      (make-instance '<prop-hierarchy> 
	:planning-problem d
	:top-level-actions '(move-block)
	:refinement-order '((move-block navigate))
	:hla-schemas
	(p2alist 'nav-up nav-up 'nav-down nav-down 'nav-left nav-left
		 'nav-right nav-right 'navigate navigate
		 'move-block move-block)))))

