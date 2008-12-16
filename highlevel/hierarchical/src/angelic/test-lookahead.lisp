(defpackage test-lookahead
  (:use lookahead
	utils
	tree
	blocks
	cl
	mapping
	env-user
	set
	prop-logic)
  (:import-from lookahead
		precond
		succ-state
		high-level-actions
		action-type
		is-primitive-sequence
		instantiate
		active
		inactive
		complete-desc
		sound-desc
		complete-result
		sound-result
		succeeds-complete
		get-immediate-refinements
		succeeds-sound
		regress-clause-nstrips
		top-level-plans
		))

(in-package test-lookahead)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; STRIPS descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars d1 d2 d3 d4)

(setf d1 (make-strips 
	  :precond '(and (foo a) (bar b))
	  :add-list '((bar c) (baz))
	  :delete-list '((foo a) (oof)))
      d2 (make-ncstrips `((,(precond d1) . ,(make-nstrips :add-list '((foo) (bar)) :delete-list '((baz))))))
      d3 (make-ncstrips `((,(precond d1) . ,(make-nstrips :add-list '((foo)) :delete-list '((bar))))
			  ((foo c) . ,(make-nstrips :add-list nil :delete-list '((bar) (c))))))
      d4 (make-ncstrips `((,(precond d1) . ,(make-nstrips :add-list '((foo) (bar)) :poss-add-list '((qux))
							  :delete-list '((baz)))))))

(tests
 "STRIPS"
 ((typep d1 'strips) t)
 ((typep d2 'strips) t)
 ((typep d3 'strips) nil)
 ((typep d4 'strips) nil)
 ((precond d1) '(and (foo a) (bar b)))
 ((succ-state '((qux) (bar b) (foo a)) d1) '((baz) (qux) (bar b) (bar c)) #'set-eq)
 ((succ-state '((qux) (bar b) (foo b)) d1) '((qux) (bar b) (foo b)) #'set-eq)
 ((succ-state '((oof) (bar b) (foo a)) d1) '((baz) (bar b) (bar c)) #'set-eq))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; NCSTRIPS
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars c1 c2)

(setf d1 (make-nstrips :add-list '((foo a b) (bar))
		       :delete-list '((baz))
		       :poss-add-list '((foo c d))
		       :poss-delete-list '((qux  x)))
      d2 (make-nstrips :add-list '((foo a b) (bar))
		       :delete-list '((qux x))
		       :poss-add-list '((foo c d))
		       :poss-delete-list '((baz)))
      c1 '(and (bar) (foo c b) (not (foo c d)))
      c2 '(and (not (baz)) (not (qux x)) (oof)))

(tests
 "NCSTRIPS"
 ((mapset 'list #'identity (conjuncts (item 0 (disjuncts (regress-clause-nstrips d1 c1 c2)))))
  '((bar) (foo c b) (not (foo c d)) (oof)) #'set-eq)
 ((mapset 'list #'identity (conjuncts (item 0 (disjuncts (regress-clause-nstrips d2 c1 c2)))))
  '((bar) (foo c b) (not (foo c d)) (oof)) #'set-eq))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Blocks world
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars nr nc d s s2 acts h hlas refs)

(setf nr 4
      nc 4
      d (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
					'(2 3) '(and (block-pos a 1 2) (block-pos c 1 1))))

(do-action d '(right 2 3 3))
(do-action d '(down 3 3 2))
(setf s (env:get-state d)
      s2 (action-seq-result d (init-state d) '((left 2 3 1) (down 1 3 2))))


(setf acts
  '#((LEFT 2 3 1) (DOWN 1 3 2) (PICKUPR C BAZ 2 2 1) (up 1 2 3) (turnl 1) (down 1 3 2) (STACKL C A 0 1 1 2) (up 1 2 3) (turnr 1) (down 1 3 2) (DOWN 1 2 1) (PICKUPR BAZ :T2 2 1 1) (RIGHT 1 1 2) (STACKR BAZ :T3 3 0 2 1) (LEFT 2 1 1) (UP 1 1 2) (up 1 2 3) (turnl 1) (down 1 3 2) (PICKUPL C A 0 2 1) (up 1 2 3) (turnr 1) (down 1 3 2) (DOWN 1 2 1) (RIGHT 1 1 2)
     (LEFT 2 1 1) (STACKR C :T2 2 0 1 1) (up 1 1 2) (up 1 2 3) (turnl 1) (down 1 3 2) (down 1 2 1) (PICKUPL A :T0 0 1 1) (UP 1 1 2) (RIGHT 1 2 2) (up 2 2 3) (turnr 2) (down 2 3 2) (STACKR A BAZ 3 1 2 2) (LEFT 2 2 1) (DOWN 1 2 1) (PICKUPR C :T2 2 1 1) (up 1 1 2) (up 1 2 3) (turnl 1) (down 1 3 2) (down 1 2 1) (RIGHT 1 1 2) (STACKL C :T1 1 0 2 1) (UP 2 1 2)
     (up 2 2 3) (turnr 2) (down 2 3 2)
  (PICKUPR A BAZ 3 2 2) (up 2 2 3) (turnl 2) (down 2 3 2) (STACKL A C 1 1 2 2)))

      

(defun literal-finder (prop-name)
  #'(lambda (x)
      (if (symbolp x)
	  (eq x prop-name)
	(eq prop-name
	    (prop-symbol
	     (typecase x (negation (negatee x)) (t x)))))))
      
(do-tests
    "Blocks-ceiling domain"
  (to-list (avail-actions d (init-state d))) '((left 2 3 1) (right 2 3 3) (turnl 2) (turnr 2))
  (to-list (avail-actions d s)) '( (up 3 2 3) (down 3 2 1))
  (to-list (avail-actions d s2)) '((left 1 2 0)(up 1 2 3) (down 1 2 1) (pickupR c baz 2 2 1))
  
  (succeeds? d acts) t
  (succeeds? d (subseq acts 0 (1- (length acts)))) nil
  )
  
  
  
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hierarchies
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(setf h (make-blocks-hierarchy d)
      s (refinements h '(move-block baz a c 0 1 3 2 3))
      s2 (refinements h '(move-block c baz a 0 1 2 2 3))
      hlas (high-level-actions h)
      refs (get-immediate-refinements h #((navigate 3 1 1 1) (pickupL baz a 0 1 1) (navigate 1 1 2 2) (stackR baz c 3 2 2 3))))


(do-tests
    "Blocks-ceiling hierarchy"
  (to-list (refinements h '(nav-right 0 3 1))) '(#((right 0 1 1) (nav-right 1 3 1)))
  (to-list (refinements h '(nav-left 2 2 1))) '(#())
  (to-list (refinements h '(nav-left 2 3 1))) nil
  (to-list (refinements h '(nav-up 1 2 3))) '(#((up 1 2 3) (nav-up 1 3 3)))
  (to-list (refinements h '(nav-down 3 2 0))) '(#((down 3 2 1) (nav-down 3 1 0)))
  (to-list (refinements h '(navigate 1 1 3 0)))
  '(#((nav-up 1 1 3) (nav-right 1 3 3) (nav-down 3 3 0))
    #((nav-up 1 1 2) (nav-right 1 3 2) (nav-down 3 2 0))
    #((nav-up 1 1 1) (nav-right 1 3 1) (nav-down 3 1 0))
    #((nav-up 1 1 3) (turnr 1) (nav-right 1 3 3) (nav-down 3 3 0))
    #((nav-up 1 1 3) (turnl 1) (nav-right 1 3 3) (nav-down 3 3 0))
    )
  (size s) 16
  (size s2) 32
  (item 0 s) #((navigate 3 3 1 1) (pickupL baz a 0 1 1) (navigate 1 1 2 3) (stackR baz c 3 2 2 3))
  (to-boolean (member? #((navigate 2 3 1 1) (pickupL c baz 0 1 1) (navigate 1 1 3 3) (stackL c a 2 2 3 3)) s2)) t
  (member? #((navigate 1 3 1 1) (pickupL c baz 0 1 1) (navigate 1 1 3 3) (stackR c a 2 2 3 3)) s2) nil
  (member? #((navigate 1 3 1 1) (pickupL baz a 0 1 1) (navigate 1 1 3 3) (stackL baz c 2 2 3 3)) s) nil
  
  (size hlas) 151040
  (action-type '(nav-right 1 2 3) h) 'high-level
  (action-type '(right 2 3 2) h) 'primitive
  (action-type '(move-block a :t2 baz 0 1 3 2 3) h) 'high-level
  (action-type '(move-block :t1 :t2 baz 0 1 3 2 3) h) 'unknown-action
  (to-boolean (is-primitive-sequence '((right 2 3 2) (pickupl baz a 1 2 3)) h)) t
  (to-boolean (is-primitive-sequence '((nav-right 1 3 3) (pickupl baz a 1 2 3)) h)) nil
  (size refs) 9
  (member? '((NAV-UP 3 1 2) (NAV-LEFT 3 1 2) (NAV-DOWN 1 2 1) (PICKUPL BAZ A 0 1 1) (NAVIGATE 1 1 2 2) (STACKR BAZ C 3 2 2 3)) refs) t
  (member? '((NAV-UP 3 1 2) (NAV-LEFT 3 1 2) (NAV-DOWN 1 2 1) (PICKUPL BAZ A 0 1 1) (NAV-UP 1 1 2) (NAV-RIGHT 1 2 2) (NAV-DOWN 2 2 2) (STACKR BAZ C 3 2 2 3)) refs) nil
  )


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; sound descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars u2 q2 q2c q2s q2h flat2 p2 p2s p2f p2c p2h p3h p3c p3f p3s p3 q3 q3s q3h q3c
	 u4 q4 q4c q4s q4h flat4 p4 p4s p4f p4c props h2 h3 flat3 u3 h4 d5 h5 flat5
	 u5 p5 q5 q5c p5c d6 u6 p6 q6c flat6 f f2 a a2 p f3 s3 q2l q3l plan tree preds
	 preds0 preds1 preds2 f4 f5 f6 a3 a4 a5 a6 plans cons3 cons2
	 move2 s4 s5 move1 nd-desc nr-desc p6c q6 h6)




(setf 
    d2 (make-blocks-world-with-ceiling nr 3 '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c a)))
    u2 (make-unguided-blocks-hierarchy d2)
    q2 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy u2
	  :complete-desc-schemas (make-complete-descriptions d2)
	  :sound-desc-schemas (make-sound-descriptions d2))
    q2c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy u2
	  :complete-desc-schemas (make-complete-descriptions d2)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u2))
    q2s (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy u2
	  :hset-desc-schemas (make-complete-descriptions d2)
	  :complete-desc-schemas (make-dummy-complete-descriptions d2)
	  :sound-desc-schemas (make-sound-descriptions d2))
    q2h (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy u2
	  :hset-desc-schemas (make-complete-descriptions d2)
	  :complete-desc-schemas (make-dummy-complete-descriptions d2)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u2))
    h2 (make-blocks-hierarchy d2)
    flat2 (make-flat-prop-hierarchy d2)
    p2 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy h2
	  :complete-desc-schemas (make-complete-descriptions d2)
	  :sound-desc-schemas (make-sound-descriptions d2))
    p2s (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy h2
	  :hset-desc-schemas (make-complete-descriptions d2)
	  :complete-desc-schemas (make-dummy-complete-descriptions d2)
	  :sound-desc-schemas (make-sound-descriptions d2))
    p2f (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d2 :hierarchy flat2)
    p2c (make-instance '<prop-abstract-planning-problem>
	  :hierarchy h2 :planning-problem d2
	  :complete-desc-schemas (make-complete-descriptions d2)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h2))
    p2h (make-instance '<prop-abstract-planning-problem>
	  :hierarchy h2 :planning-problem d2
	  :hset-desc-schemas (make-complete-descriptions d2)
	  :complete-desc-schemas (make-dummy-complete-descriptions d2)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h2))
    d3 (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c a) (on baz c)))
    h3 (make-blocks-hierarchy d3)
    flat3 (make-flat-prop-hierarchy d3)
    u3 (make-unguided-blocks-hierarchy d3)
    p3 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy h3
	  :complete-desc-schemas (make-complete-descriptions d3)
	  :sound-desc-schemas (make-sound-descriptions d3))
    q3 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy u3
	  :complete-desc-schemas (make-complete-descriptions d3)
	  :sound-desc-schemas (make-sound-descriptions d3))
    q3s (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy u3
	  :complete-desc-schemas (make-dummy-complete-descriptions d3)
	  :sound-desc-schemas (make-sound-descriptions d3))
    q3h (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy u3
	  :complete-desc-schemas (make-dummy-complete-descriptions d3)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u3))
    q3c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy u3
	  :complete-desc-schemas (make-complete-descriptions d3)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u3))
    p3s (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy h3
	  :hset-desc-schemas (make-complete-descriptions d3)
	  :complete-desc-schemas (make-dummy-complete-descriptions d3)
	  :sound-desc-schemas (make-sound-descriptions d3))
    p3c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy h3
	  :complete-desc-schemas (make-complete-descriptions d3)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h3))
    p3f (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d3 :hierarchy flat3)
    d4 (make-blocks-world-with-ceiling 6 nc '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 4) '(and (on baz :t3) (on c baz)))
    h4 (make-blocks-hierarchy d4)
    flat4 (make-flat-prop-hierarchy d4)
    u4 (make-unguided-blocks-hierarchy d4)
    p4 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d4 :hierarchy h4
	  :complete-desc-schemas (make-complete-descriptions d4)
	  :sound-desc-schemas (make-sound-descriptions d4))
    q4 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d4 :hierarchy u4
	  :complete-desc-schemas (make-complete-descriptions d4)
	  :sound-desc-schemas (make-sound-descriptions d4))
    q4c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d4 :hierarchy u4
	  :complete-desc-schemas (make-complete-descriptions d4)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u4))
    p4c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d4 :hierarchy h4
	  :complete-desc-schemas (make-complete-descriptions d4)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h4))
    p4f (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d4 :hierarchy flat4)
    d5 (make-blocks-world-with-ceiling 8 5 '((a 0 1) (baz 2 1) (c 2 2) (qux 4 1) )
				       '(3 3) '(and (on baz :t3) (on c baz) (on a :t1)))
    h5 (make-blocks-hierarchy d5)
    flat5 (make-flat-prop-hierarchy d5)
    u5 (make-unguided-blocks-hierarchy d5)
    p5 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d5 :hierarchy h5
	  :complete-desc-schemas (make-complete-descriptions d5)
	  :sound-desc-schemas (make-sound-descriptions d5))
    q5 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d5 :hierarchy u5
	  :complete-desc-schemas (make-complete-descriptions d5)
	  :sound-desc-schemas (make-sound-descriptions d5))
    q5c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d5 :hierarchy u5
	  :complete-desc-schemas (make-complete-descriptions d5)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u5))
    p5c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d5 :hierarchy h5
	  :complete-desc-schemas (make-complete-descriptions d5)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h5))
    d6 (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c :t3) (on baz c) (on a baz)))
    h6 (make-blocks-hierarchy d6)
    u6 (make-unguided-blocks-hierarchy d6)
    p6 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d6 :hierarchy h6
	  :complete-desc-schemas (make-complete-descriptions d6)
	  :sound-desc-schemas (make-sound-descriptions d6))
    q6 (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d6 :hierarchy u6
	  :complete-desc-schemas (make-complete-descriptions d6)
	  :sound-desc-schemas (make-sound-descriptions d6))
    q6c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d6 :hierarchy u6
	  :complete-desc-schemas (make-complete-descriptions d6)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions u6))
    p6c (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d6 :hierarchy h
	  :complete-desc-schemas (make-complete-descriptions d6)
	  :sound-desc-schemas (vacuous-sound-ncstrips-descriptions h))
      props (propositions d2)
      s (init-state-set d2)
      s2 (sound-result p2 s '((nav-left 2 0 3)))
      f (formula s)
      f2 (formula s2)
      a '((navigate 2 3 1 2) (pickupR c baz 2 2 1) (navigate 1 2 1 2) (stackL c a 0 1 1 2))
      a2 '((move-block c baz a 2 2 0 1 2))
      )

(do-tests "Sound descriptions"
  (length f) (length f2) 
  (and (dnf-implies f2 '(and (gripper-pos 0 3) (not (gripper-pos 2 3))))
       (dnf-implies f '(and (gripper-pos 2 3) (not (gripper-pos 0 3))))) 
  t
  (succeeds-sound p2 a) t
  (succeeds-sound p2 a2) nil
  
       
       
       
)  


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; complete descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf 
    p (make-instance '<prop-abstract-planning-problem>
	  :planning-problem d :hierarchy h
	  :complete-desc-schemas (make-complete-descriptions d)
	  :sound-desc-schemas (make-sound-descriptions d))
      props (propositions d)
      s (init-state-set d)

      nr-desc (complete-desc p '(nav-left 2 0 3))
      nd-desc (complete-desc p '(nav-down 0 3 2))
      s2 (successor-set nr-desc s)
      s3 (successor-set nd-desc s2)
      f3 (formula s3)
      move1 (complete-desc p '(move-block c baz a 2 2 0 1 1))
      s4 (complete-result p s '((nav-left 2 0 3) (nav-down 0 3 2) (move-block c baz a 2 2 0 1 2)))
      f4 (formula s4)
      move2 (complete-desc p '(right 1 2 2))
      s5 (successor-set move2 s4)
      f5 (formula s5)
      a '((move-block c baz :t1 2 2 1 0 1) (move-block a :t0 c 0 1 1 1 2))
      a2 '((move-block c baz :t1 2 2 1 0 2) (move-block a :t0 c 0 1 1 1 2))
      a3 '((move-block c baz :t1 2 2 1 0 1) (navigate 2 1 3 2) (move-block a :t0 c 0 1 1 1 2))
      a4 '((navigate 2 3 1 2) (pickupr c baz 2 2 1) (stackl c a 0 1 1 2) (move-block baz :t2 :t3 2 1 3 0 1) (move-block c a :t1 0 2 1 0 1) (move-block a :t0 c 0 1 1 1 2))
      a5 '((navigate 2 3 1 2) (pickupr c baz 2 2 1) (stackr c baz 2 1 1 2) (move-block baz :t2 :t3 2 1 3 0 1) (move-block c a :t1 0 2 1 0 1) (move-block a :t0 c 0 1 1 1 2))
      a6 '((navigate 2 3 1 2) (pickupr c baz 2 2 1) (navigate 1 2 1 2) (stackl c a 0 1 1 2) (move-block baz :t2 :t3 2 1 3 0 1) (move-block c a :t1 0 2 1 0 1) (move-block a :t0 c 0 1 1 1 2))
)      

(setf cons2 (conjuncts (item 0 (disjuncts (formula s2))))
      cons3 (conjuncts (item 0 (disjuncts (formula s3)))))
(do-tests
    "Complete descriptions"
  (size cons2) 198
  (member? '(not (gripper-pos 0 3)) cons2) nil
  (to-boolean (member? '(not (gripper-pos 0 2)) cons2)) t
  
  (filter ':list (direct-product 'list 4 4) 
	  #'(lambda (x) 
	      (and (> (second x) 0) (not (dnf-implies f3 `(or (and (not (gripper-pos ,@x)))))))))
  '((2 3) (1 3) (0 3) (0 2))
  
  (to-boolean (and (member? '(on c baz) cons3) (member? '(clear a) cons3)
		   p(member? '(not (on c a)) cons3) (member? '(not (clear baz)) cons3))) 
  t
  
  (dnf-consistent f5 '(gripper-pos 0 3)) nil
  (dnf-consistent f5 '(gripper-pos 1 2)) nil
  (to-boolean (dnf-consistent f5 '(gripper-pos 2 2))) t
  (to-boolean (dnf-consistent f4 '(gripper-pos 0 3))) nil
  (to-boolean (dnf-consistent f4 '(gripper-pos 1 2))) t
  
  (to-boolean (succeeds-complete p a)) t
  (succeeds-complete p a2) nil
  (to-boolean (succeeds-complete p a3)) t
  (to-boolean (succeeds-complete p a4)) nil
  (to-boolean (succeeds-complete p a5)) nil
  (to-boolean (succeeds-complete p a6)) t
  )
  
      
      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; abstract planning problems
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf plans (top-level-plans p)
      *refinement-set-type* ':queue)

(do-tests "abstract planning problems"
  (item 0 plans) nil
  (member? '((move-block a :t0 c 0 1 2 2 3)) plans) t
  (member? '((move-block c baz a 2 2 0 1 2)) plans) t)

(setf q3l
  (make-instance '<prop-abstract-planning-problem>
    :preprocess t :admissible-heuristic (dist-heuristic d3)
    :planning-problem d3 :hierarchy u3
    :subsumption-checker (make-blocks-subsumption-checker d3)
    :complete-desc-schemas (make-complete-descriptions d3)
    :sound-desc-schemas (make-sound-descriptions d3))
  q2l
  (make-instance '<prop-abstract-planning-problem>
    :preprocess t :admissible-heuristic (pg-heuristic d2)
    :planning-problem d2 :hierarchy u2
    :complete-desc-schemas (make-complete-descriptions d2)
    :sound-desc-schemas (make-sound-descriptions d2))
  
  preds0 '((act :reward infty) ((move-block move-to) :reward -1) (navigate :count 1) (nav :count 1) (:primitive :count 1))
  preds1 '((act :reward infty) ((move-block move-to) :reward -3) (navigate :count 0) (nav :count 3) (:primitive :count 3)))

(defun priority-fn (a cr sr)
  (if (my> sr '-infty)
      (avg cr sr .5)
    (my* cr
	 (if (eq a 'act)
	     2
	   1.5 ;; covers move-block, move-to, finish, navigate and nav (when they don't have sound descs)
	   ))))

(defun plan-valid (p)
  (dotimes (i (1- (length p)) t)
    (let ((a1 (aref p i))
	  (a2 (aref p (1+ i))))
      (when (and (consp a1)
		 (consp a2)
		 (eq (first a1) 'move-block)
		 (eq (first a2) 'move-block)
		 (eq (second a1) (second a2)))
	(return nil)))))

(tests "offline lookahead"
       ((mvsetq (plan tree) (aha* q2l :termination ':begins-with-primitive))
	#((LEFT 2 3 1) (NAV 1 3 1 2) (PICKUPR C BAZ 2 2 1) (NAV 1 2 1 3) (TURNL 1) (NAV 1 3 1 2) (STACKL C A 0 1 1 2) FINISH))
       ((is-well-formed tree q2l) t)
       
       ((aha* q2l :termination ':primitive :valid-plan-pred #'plan-valid)
	#((LEFT 2 3 1) (DOWN 1 3 2) (PICKUPR C BAZ 2 2 1) (UP 1 2 3) (TURNL 1) (DOWN 1 3 2) (STACKL C A 0 1 1 2) FINISH))
       
       ((progn
	  (mvsetq (plan tree) (ahss q3l '-infty :termination ':primitive :priority-fn #'priority-fn ))
	  (succeeds? d3 (subseq plan 0 (1- (length plan)))))
	t)
       
       ((is-well-formed tree q3l) t))
	




(tests "hfs"
  ((succeeds? d2 (sound-complete-forward-search q2)) t)

  ((let ((*refinement-set-type* ':stack))
     (succeeds? d2 (sound-complete-forward-search q2)))
  t)
  
  ((sound-complete-forward-search q2 :first-action-only t) '((left 2 3 1)))
  

  )


  
      
      
  

(defun run-first (p d &optional (n 1) (verbose t))
  (time (dotimes (i n) (print (sound-complete-forward-search p :verbose verbose :first-action-only t)))))

(defun run (p d &optional (n 1) (verbose t))
  (time (dotimes (i n) (assert (succeeds? d (sound-complete-forward-search p :verbose verbose ))))))