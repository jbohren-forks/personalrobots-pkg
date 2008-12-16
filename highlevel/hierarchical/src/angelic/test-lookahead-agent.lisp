(defpackage test-lookahead-agent
  (:use lookahead
	utils
	tree
	blocks
	cl
	mapping
	env-user
	set
	prop-logic)
  (:import-from 
   lookahead

   mean
   var
   hierarchical-forward-search
   pprint-plans
   action
   complete-reward
   sound-reward
   complete-reward-upto
   sound-reward-upto
   complete-q
   status

   clear-subsumption-checker

   add-entry
   is-subsumed
   active
   inactive
   is-well-formed)
  (:import-from
   blocks
   make-blocks-subsumption-checker))


(in-package test-lookahead-agent)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; examples
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun heuristic (d)
  #'(lambda (s)
      (if (intersects s (goal d))
	  0
	-1)))

(setf 
    nr 4
    nc 4
    d2 (make-blocks-world-with-ceiling nr 3 '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c a)))
    u2  (make-unguided-blocks-hierarchy d2)
    
    f2 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d2)
	 :planning-problem d2 :hierarchy (make-flat-prop-hierarchy d2))
    
    q2 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d2)
	 :planning-problem d2 :hierarchy u2
	 :complete-desc-schemas (make-complete-descriptions d2)
	 :sound-desc-schemas (make-sound-descriptions d2))
    h2 (make-navigateless-hierarchy d2)
    g2 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d2)
	 :planning-problem d2 :hierarchy h2
	 :complete-desc-schemas (make-complete-descriptions d2)
	 :sound-desc-schemas (make-sound-descriptions d2))
    d3 (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c a) (on baz c)))
    u3  (make-unguided-blocks-hierarchy d3)
    q3 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d3)
	 :planning-problem d3 :hierarchy u3
	 :complete-desc-schemas (make-complete-descriptions d3)
	 :sound-desc-schemas (make-sound-descriptions d3))
    h3 (make-navigateless-hierarchy d3)
    g3 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d3)
	 :planning-problem d3 :hierarchy h3
	 :complete-desc-schemas (make-complete-descriptions d3)
	 :sound-desc-schemas (make-sound-descriptions d3))
    d4 (make-blocks-world-with-ceiling 6 4 '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 4) '(and (on baz :t3) (on c baz)))
    u4 (make-unguided-blocks-hierarchy d4)
    q4 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d4)
	 :planning-problem d4 :hierarchy u4
	 :complete-desc-schemas (make-complete-descriptions d4)
	 :sound-desc-schemas (make-sound-descriptions d4))
    h4 (make-navigateless-hierarchy d4)
    g4 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d4)
	 :planning-problem d4 :hierarchy h4
	 :complete-desc-schemas (make-complete-descriptions d4)
	 :sound-desc-schemas (make-sound-descriptions d4))

    d5 (make-blocks-world-with-ceiling 6 10 
				       '((a 0 1) (b 0 2) (c 0 3) (d 0 4)) '(1 2)
				       '(and (on d :t8) (on c d) (on b c) (on a b)))
    u5 (make-unguided-blocks-hierarchy d5)
    q5 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d5)
	 :planning-problem d5 :hierarchy u5
	 :complete-desc-schemas (make-complete-descriptions d5)
	 :sound-desc-schemas (make-sound-descriptions d5))
    f5 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d5)
	 :planning-problem d5 :hierarchy (make-flat-prop-hierarchy d5))
    h5 (make-navigateless-hierarchy d5)
    g5 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d5)
	 :planning-problem d5 :hierarchy h5
	 :complete-desc-schemas (make-complete-descriptions d5)
	 :sound-desc-schemas (make-sound-descriptions d5))


    d6 (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
				       '(2 3) '(and (on c :t3) (on baz c) (on a baz)))
    u6 (make-unguided-blocks-hierarchy d6)
    q6 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d6)
	 :planning-problem d6 :hierarchy u6
	 :complete-desc-schemas (make-complete-descriptions d6)
	 :sound-desc-schemas (make-sound-descriptions d6))
    h6 (make-navigateless-hierarchy d6)
    g6 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d6)
	 :planning-problem d6 :hierarchy h6
	 :complete-desc-schemas (make-complete-descriptions d6)
	 :sound-desc-schemas (make-sound-descriptions d6))

    d7 (make-blocks-world-with-ceiling 8 5 '((a 0 1) (baz 2 1) (c 2 2) (qux 4 1) )
				       '(3 3) '(and (on baz :t3) (on c baz) (on a :t1)))
    u7 (make-unguided-blocks-hierarchy d7)
    q7 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d7)
	 :planning-problem d7 :hierarchy u7
	 :complete-desc-schemas (make-complete-descriptions d7)
	 :sound-desc-schemas (make-sound-descriptions d7))
    f7 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d7)
	 :planning-problem d7 :hierarchy (make-flat-prop-hierarchy d7))
    h7 (make-navigateless-hierarchy d7)
    g7 (make-instance '<prop-abstract-planning-problem>
	 :preprocess t :admissible-heuristic (dist-heuristic d7)
	 :planning-problem d7 :hierarchy h7
	 :complete-desc-schemas (make-complete-descriptions d7)
	 :sound-desc-schemas (make-sound-descriptions d7))

    

    props (propositions d2)
    )


(setf c2 (make-blocks-subsumption-checker d2)
      c3 (make-blocks-subsumption-checker d3)
      c4 (make-blocks-subsumption-checker d4)
      c5 (make-blocks-subsumption-checker d5)
      c6 (make-blocks-subsumption-checker d6)
      c7 (make-blocks-subsumption-checker d7))
      
      


(setf preds0 '((act :reward infty) ((move-block move-to) :reward -1) (navigate :count 1) (nav :count 1) (:primitive :count 1))
      preds1 '((act :reward infty) ((move-block move-to) :reward -3) (navigate :count 0) (nav :count 3) (:primitive :count 3))
      preds2 '((act :reward infty) ((move-block move-to navigate nav :primitive) :reward -20))
      preds3 '((act :reward infty) ((move-block move-to navigate nav :primitive) :reward -10)))

(defun preds (i j)
  `((act :reward infty) ((move-block move-to navigate nav) :reward ,(- i)) (:primitive :reward ,(- j))))




(abbrev hbf hierarchical-breadth-first)
(abbrev hrt hierarchical-real-time)

(defun mean-var-fn (a cr sr)
  (let* ((mean (mymax
		(my* (my+ cr sr) .5)
		(my* cr 
		     (if (eq a 'act) 
			 3 
		       2))))
	 (d (if (eql cr mean) 0 (- cr mean))))
    (values mean (/ (* d d) 3))))


(defun priority-fn (a cr sr)
  (mymax
   (my* (my+ cr sr) .5)
   (if (eq a 'act)
       (mymin (my* cr 3) -1)
     (my* cr 2)))) ;; covers move-block, move-to, finish, navigate and nav (when they don't have sound descs)


(defun print-node (x)
  (if (eq (status x) 'inactive) (format nil "~a (inactive)" (action x))
    (format nil "~a.  Bounds: [~a, ~a].  Mean: ~0,1f. Std: ~0,1f. Q: ~0,1f.  Q-std: ~0,1f" 
	    (action x)
	    (sound-reward x) (complete-reward x)
	    (mean x) (awhen (var x) (sqrt it)) (mean-q x) (awhen (hla::var-q x) (sqrt it)))))

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