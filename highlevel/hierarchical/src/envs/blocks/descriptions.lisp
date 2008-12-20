(in-package :blocks)

(defun make-complete-descriptions (d)
  (let ((max-row (1- (num-rows d))))
    (flet ((move-upper-bound (xb yb xc yt cset)
	     (move-reward-upper-bound xb yb xc yt max-row cset))
	   (move-to-bound (xc yt cset)
	     (move-to-upper-bound xc yt max-row cset)))
  (make-ncstrips-schemas d
    (act
     :var-domains ()
     :effects ())
    (nav-right 
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ((:precond 
		(and (gripper-pos ?xs ?y) (right-of ?xt ?xs) (free ?xt ?y))
		:poss-add-list ((gripper-pos ?xt ?y))
		:poss-delete-list ((gripper-pos ?xs ?y)))))
    
    (nav-left 
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ((:precond 
		(and (gripper-pos ?xs ?y) (right-of ?xs ?xt) (free ?xt ?y))
		:poss-add-list ((gripper-pos ?xt ?y) (for-all ((?x columns)) (xbetween ?xt ?x ?xs)
									    (gripper-pos ?x ?y)))
		:poss-delete-list ((gripper-pos ?xs ?y) (for-all ((?x columns)) (xbetween ?xt ?x ?xs)
									       (free ?x ?y))))))
    
    (nav-up
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ((:precond 
		(and (gripper-pos ?x ?ys) (above ?yt ?ys) (free ?x ?yt))
		:poss-add-list ((gripper-pos ?x ?yt) )
		:poss-delete-list ((gripper-pos ?x ?ys) ))))
    
    (nav-down
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ((:precond 
		(and (gripper-pos ?x ?ys) (above ?ys ?yt) (free ?x ?yt))
		:poss-add-list ((gripper-pos ?x ?yt) )
		:poss-delete-list ((gripper-pos ?x ?ys) ))))

    (navigate
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ((:precond
		(and (gripper-pos ?xs ?ys) (int= ?xs ?xt) (int= ?ys ?yt))
		:reward (negated-manhattan-dist ?xs ?ys ?xt ?yt)
		:poss-add-list ((faceR) (faceL))
		:poss-delete-list ((faceR) (faceL)))
	       (:precond
		(and (gripper-pos ?xs ?ys) (free ?xt ?yt))
		:reward (negated-manhattan-dist ?xs ?ys ?xt ?yt)
		:poss-add-list ((faceR) (faceL)) 
		:add-list ((gripper-pos ?xt ?yt))
		:poss-delete-list ((faceR) (faceL)) 
		:delete-list ((gripper-pos ?xs ?ys)))))
    
    (nav 
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ((:precond
		(and (gripper-pos ?xs ?ys) (int= ?xs ?xt) (int= ?ys ?yt))
		:reward (negated-manhattan-dist ?xs ?ys ?xt ?yt))
	       (:precond
		(and (gripper-pos ?xs ?ys) (free ?xt ?yt))
		:reward (negated-manhattan-dist ?xs ?ys ?xt ?yt)
		:add-list ((gripper-pos ?xt ?yt))
		:delete-list ((gripper-pos ?xs ?ys)))))
     
    (move-block
     :var-domains ((?b actual-blocks) (?a blocks) (?c blocks) (?xb columns) (?yb rows)
				      (?xc columns) (?yc all-rows) (?yt rows))
     :effects ((:precond 
		(and (clear ?b) (clear ?c) (on ?b ?a))
		:reward (move-upper-bound ?xb ?yb ?xc ?yt ?complete-set)
		:delete-list ((on ?b ?a) (clear ?c) (block-pos ?b ?xb ?yb) (free ?xc ?yt)
					 (for-all ((?x columns) (?y rows))
						  (or (not (int= ?y ?yt))
						      (not (or (r ?x ?xc) (r ?xc ?x))))
						  (gripper-pos ?x ?y)))
		:poss-add-list ((for-all ((?x columns)) (or (r ?x ?xc) (r ?xc ?x)) (gripper-pos ?x ?yt)) )
		:add-list ((on ?b ?c) (clear ?a) (block-pos ?b ?xc ?yt) (free ?xb ?yb)))))
    
    (move-to
     :var-domains ((?b actual-blocks) (?c blocks) (?xc columns) (?yc all-rows) (?yt rows))
     :effects ((:precond 
		(and (gripper-holding ?b) (clear ?c))
		:reward (move-to-bound ?xc ?yt ?complete-set)
		:delete-list ((gripper-holding ?b) (clear ?c) (free ?xc ?yt)
						   (for-all ((?x columns) (?y rows))
							    (or (not (int= ?y ?yt))
								(not (or (r ?x ?xc) (r ?xc ?x))))
							    (gripper-pos ?x ?y)))
		:poss-add-list ((for-all ((?x columns)) (or (r ?x ?xc) (r ?xc ?x)) (gripper-pos ?x ?yt)) )
		:add-list ((on ?b ?c) (clear ?b) (block-pos ?b ?xc ?yt) (gripper-holding nothing)))))
    ))))


(defun make-sound-descriptions (d)
  (let ((max-row (1- (num-rows d))))
    (labels ((nav-lower-bound (xs ys xt yt)
	       (- 0
		(- max-row ys) ;; nav to top
		(abs-diff xs xt) ;; nav across
		(- max-row yt) ;; nav back down
		))
	   (navigate-lower-bound (xs ys xt yt)
	     (1- (nav-lower-bound xs ys xt yt)))) ;; 1 extra step to turn around
    (make-ncstrips-schemas d
      (act
       :var-domains ()
       :effects ())
      (nav-right 
       :var-domains ((?xs columns) (?xt columns) (?y rows))
       :effects ((:precond 
		  (and (gripper-pos ?xs ?y) (int= ?xt ?xs) ))
		 (:precond 
		  (and (gripper-pos ?xs ?y) (right-of ?xt ?xs) (for-all (?x columns) (free ?x ?y)))
		  :add-list ((gripper-pos ?xt ?y))
		  :delete-list ((gripper-pos ?xs ?y)))))
      (nav-left
       :var-domains ((?xs columns) (?xt columns) (?y rows))
       :effects ((:precond 
		  (and (gripper-pos ?xs ?y) (int= ?xs ?xt) ))
		 (:precond 
		  (and (gripper-pos ?xs ?y) (right-of ?xs ?xt) (for-all (?x columns) (free ?x ?y)))
		  :add-list ((gripper-pos ?xt ?y))
		  :delete-list ((gripper-pos ?xs ?y)))))
      (nav-up 
       :var-domains ((?x columns) (?ys rows) (?yt rows))
       :effects ((:precond 
		  (and (gripper-pos ?x ?ys) (int= ?yt ?ys) ))
		 (:precond 
		  (and (gripper-pos ?x ?ys) (above ?yt ?ys) (for-all (?y rows) (free ?x ?y)))
		  :add-list ((gripper-pos ?x ?yt))
		  :delete-list ((gripper-pos ?x ?ys)))))
    
      (nav-down 
       :var-domains ((?x columns) (?ys rows) (?yt rows))
       :effects ((:precond 
		  (and (gripper-pos ?x ?ys) (int= ?ys ?yt) ))
		 (:precond 
		  (and (gripper-pos ?x ?ys) (above ?ys ?yt) (for-all (?y rows) (free ?x ?y)))
		  :add-list ((gripper-pos ?x ?yt))
		  :delete-list ((gripper-pos ?x ?ys)))))
    
      (navigate
       :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
       :effects ((:precond
		  (and (gripper-pos ?xs ?ys) (int= ?xs ?xt) (int= ?ys ?yt))
		  :reward (navigate-lower-bound ?xs ?ys ?xt ?yt)
		  :poss-add-list ((facer) (facel))
		  :poss-delete-list ((facer) (facel))
		  :add-list nil :delete-list nil)
		 (:precond
		  (and (gripper-pos ?xs ?ys) (free ?xt ?yt) (for-all (?x columns) (free ?x (1- (num-rows d)))))
		  :reward (navigate-lower-bound ?xs ?ys ?xt ?yt)
		  :poss-add-list ((facer) (facel))
		  :poss-delete-list ((facer) (facel))
		  :add-list ((gripper-pos ?xt ?yt))
		  :delete-list ((gripper-pos ?xs ?ys)))))
    
      (nav
       :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
       :effects ((:precond
		  (and (gripper-pos ?xs ?ys) (int= ?xs ?xt) (int= ?ys ?yt))
		  :reward 0
		  :add-list nil :delete-list nil)
		 (:precond
		  (and (gripper-pos ?xs ?ys) (free ?xt ?yt) (for-all (?x columns) (free ?x (1- (num-rows d)))))
		  :reward (nav-lower-bound ?xs ?ys ?xt ?yt)
		  :add-list ((gripper-pos ?xt ?yt))
		  :delete-list ((gripper-pos ?xs ?ys)))))
      (move-block
       :var-domains ((?b actual-blocks) (?a blocks) (?c blocks) (?xb columns) (?yb rows)
					(?xc columns) (?yc all-rows) (?yt rows))
       :effects nil
       )
      (move-to
       :var-domains ((?b actual-blocks) (?c blocks) (?xc columns) (?yc all-rows) (?yt rows))
       :effects nil
       )
      ))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; helper
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun negated-manhattan-dist (xs ys xt yt)
  (- (+ (abs-diff xs xt) (abs-diff ys yt))))

(defun illegal-gripper-pos (s)
  (let ((pos (possible-gripper-positions s)))
    (values
     (each
      (disjuncts (formula s))
      #'(lambda (clause)
	  (let ((occupied nil))
	    (do-elements (lit (conjuncts clause))
	      (when (eq (car lit) 'block-pos)
		(push (cddr lit) occupied)))
	    (each pos #'(lambda (p) (member p occupied :test #'equal))))))
     pos)))

(defun move-reward-upper-bound (xb yb xc yt max-row complete-set)
  "First the gripper has to move beside the block.  After this, each action decreases the distance by at most 1."
  (let ((gpos (possible-gripper-positions complete-set)))
    (if (is-empty gpos)
	'-infty 
      (my+
   
       ;; Distance from source dest to goal
       (min (negated-manhattan-dist xb yb xc yt) -3)
   
       ;; Distance to move to source
       (reduce-set 
	#'mymax gpos
	:key #'(lambda (pos)
		 
		 (my+ 
		  
		  ;; Additional term for turning around

		  (reduce-set 
		   #'mymin
		   (disjuncts (formula complete-set))
		   :key #'(lambda (clause)
			    (let ((yg (second pos)))
			      (let ((free-clauses (get-clause-instances 'free clause)))
				(dolist (dir '(facel facer) 0)
				  (when (and (get-clause-instances dir clause)
					     (or (not (member-equal (putdown-location xb yb dir) free-clauses))
						 (not (member-equal (putdown-location xc yt dir) free-clauses))))
				    (return (1- (* 2 (- (max yb yg yt) max-row))))))))))

		 
		  (let ((d (apply #'negated-manhattan-dist xb yb pos)))
		    ;; if d is zero, this is in fact an inconsistent possibility
		    (if (zerop d) 
			'-infty
		      (+ d (if (eql xb (first pos)) -1 1) ;; if we're directly above block, further inefficiency
			 ))))))))))


(defun move-to-upper-bound (xc yt max-row complete-set)
  ;; As above
  (mymin -1
	 (reduce-set 
	  #'mymin
	  (disjuncts (formula complete-set))
	  :key #'(lambda (clause)
		   (let ((free-clauses (get-clause-instances 'free clause)))
		     (dsbind (xg yg) (check-not-null (first (get-clause-instances 'gripper-pos clause)))
		       (+
			;; Each action can decrease distance of block to goal by at most 1
			(negated-manhattan-dist xc yt xg yg)
		      
			;; penalty for having to turn around
			(dolist (dir '(facel facer) 0)
			  (when (and (get-clause-instances dir clause)
				     (not (member-equal (putdown-location xc yt dir) free-clauses)))
			    (return (1- (* 2 (- (max yg yt) max-row)))))))))))))


(defun putdown-location (x y dir)
  "return list (x' y') s.t. doing a putdown from (x' y') while facing direction DIR puts the block at (x y)"
  (list
   (ecase dir
     (facel (1+ x))
     (facer (1- x)))
   y))
	      
	  

    
(defun dist-heuristic (d)
  "dist-heuristic D.  

Given the on(x,y) predicates in the goal, there will be some blocks which must end up at a certain location.  For these blocks, sum the distance to their destination.  There must also be other groups of blocks (disjoint from the first type) which must be in a contiguous column together.  For such a group, the total horizontal displacement is bounded below by the summed distance to median of the current columns.  Also, for every pair A on B in such a group, there must be vertical displacement sufficient to make A one above B.  

There is one case handled differently, when all the destinations are absolute.  In this case, we can take into account the travel time of the gripper between blocks more completely by using a max-weight matching."

  (let ((on-chains (compute-on-chains (formula (goal d))))
	(known-positions nil)
	(groups nil)
	(base-blocks (mapset 'list #'base-block (num-cols d)))
	)
    
    (dolist (chain on-chains)
      (aif (position (first chain) base-blocks)
	  (do-elements (b chain nil i)
	    (push (cons b (list it i)) known-positions))
	(push chain groups)))

    (if groups

	;; Case when some destinations are not absolute
	#'(lambda (s)
	    (if (illegal-gripper-pos s)
		'-infty
	      (reduce-set  
	       #'mymax
	       (disjuncts (formula s))
	       :key
	       #'(lambda (clause)
		   (-
		    (sum-over 
		     known-positions
		     #'(lambda (p)
			 (dsbind (b col row) p
			   (apply #'negated-manhattan-dist col row (get-block-pos b clause)))))
		
		    (sum-over 
		     groups
		     #'(lambda (group)
			 (let* ((cols (sort (map 'vector #'(lambda (b) (first (get-block-pos b clause))) group) #'<))
				(rows (mapcar #'(lambda (b) (second (get-block-pos b clause))) group))
				(median (aref cols (floor (length cols) 2))))
			   (my+
			    (sum-over cols #'(lambda (x) (abs-diff x median))) ;; column displacement
			    (loop for r1 in rows for r2 in (rest rows) summing (abs-diff (1+ r1) r2)) ;; row displacement
			    )))))))))

      ;; Special case when all destinations are known
      (matching-based-heuristic known-positions on-chains))))


(defun matching-based-heuristic (known on-chains)
  #'(lambda (s)
      (debug-out :blocks 1 t "~&Computing matching-based heuristic for ~a~&  known: ~a~&  on-chains: ~a" s known on-chains)
      (mvbind (illegal gripper) (illegal-gripper-pos s)
	(setf gripper (mapset 'list #'identity gripper))

	(if illegal
	    '-infty
	  (my+ (* -4 (count-switches s on-chains known))
	   
	  
	  
	       (reduce-set 
		#'mymax
		(disjuncts (formula s))
		:key
		#'(lambda (cl)
		    (let ((positions nil)
			  (nodes (make-hash-table :test #'equal)))
	       
		      (addf nodes 'start)
		      (addf nodes 'end)
	      
		      ;; Compute goal-positions, current-positions
		      (dolist (k known)
			(dsbind (b . goal-pos) k
			  (let ((current-pos (get-block-pos b cl)))
			    (unless (equal current-pos goal-pos)
			      (push (list b goal-pos current-pos) positions)
			      (addf nodes b)
			      (addf nodes (cons b 'dest))))))
		 
		      ;; Construct graph
		      (let ((g (make-instance '<adjacency-matrix-graph> :nodes nodes))
			    (max-dist 0))
		 
			;; First figure out max-dist in order to make weights all positive (is this necessary?)
			(dolist (e positions)
			  (dsbind (b gp cp) e
			    (declare (ignore b))
			    (dolist (e2 positions)
			      (let ((cp2 (third e2)))
				(maxf max-dist (+ -1 (dist gp cp) (dist gp cp2)))))))
		 
			;; Now create the adjacency matrix
			(dolist (entry positions)
			  (dsbind (b gpos cpos) entry
			    (let ((bd (cons b 'dest)))
			      (add-edge g b 'start :label (- max-dist (reduce-set #'mymin gripper :key #'(lambda (pos) (max 1 (dist pos cpos))))))
			      (add-edge g 'end bd :label (- max-dist (dist gpos cpos)))
			      (dolist (entry2 positions)
				(dsbind (b2 gpos2 cpos2) entry2
				  (declare (ignore gpos2))
				  (add-edge g b2 bd :label (- max-dist (max 1 (+ (dist gpos cpos) (dist gpos cpos2))))))))))

			;; Solve the matching problem
			(let ((m (max-weight-bipartite-matching g)))
			  (sum-over m #'(lambda (e) (- (edge-label e) max-dist)))))))))))))


(defun count-switches (s on-chains known)
  (let ((c (item 0 (disjuncts (formula s)))))
    (sum-over
     on-chains
     #'(lambda (chain)
	 (sum-over
	  (length chain)
	  #'(lambda (i)
	      (let* ((b (nth i chain))
		     (p (get-block-pos b c))
		     (g (mapping:evaluate known b)))
		(indicator
		 (any i
		      #'(lambda (j)
			  (let* ((b2 (nth j chain))
				 (p2 (get-block-pos b2 c))
				 (g2 (mapping:evaluate known b2)))
			    (and
			     (not (equal g2 p2))
			     (above p p2)
			     (above g g2)))))))))))))

(defun above (p p2)
  (and (= (first p) (first p2))
       (> (second p) (second p2))))
		       
	       

	
	     


(defun compute-on-chains (f)
  "Helper for col-heuristic.  Returns a list of items of the form (B1 ... Bn), which mean that in the goal, B1 must be below B2 which is below B3 and so on.  B1 may or may not be a base block."
  (let ((goal-conjuncts (conjuncts f))
	(on-chains nil))
    (do-elements (c goal-conjuncts on-chains)
      (when (eq (prop-symbol c) 'on)
	(dsbind (b1 b2) (cdr c)
	  (let ((entry (find b2 on-chains :key #'slast))
		(entry2 (assoc b1 on-chains)))
	    (cond
	     ((and entry entry2) (deletef on-chains entry2) (nconc entry entry2))
	     (entry (let ((last-item (last entry))) (setf (cdr last-item) (list b1))))
	     (entry2 (setf (cdr entry2) (cons (car entry2) (cdr entry2)) (car entry2) b2))
	     (t (push (list b2 b1) on-chains)))))))))



(defun pg-heuristic (d)
  "heuristic: define the energy of a state to be, for each goal proposition (on a b), the distance a is from the position above b.  In a goal state, the energy is 0.  Each primitive action decreases the total energy by at most 2 (since each block can be involved in at most 2 goal propositions).  So we just divide the total distance by 2 and negate to get the heuristic."
  (let ((goal-conjuncts (conjuncts (formula (goal d)))))
    #'(lambda (s)
	
	;; First, if the gripper is in an illegal position, don't allow
	(if (illegal-gripper-pos s)
	    '-infty
	
	  ;; Otherwise, sum distances
	  (sum-over
	   goal-conjuncts
	   #'(lambda (c)
	       (iwhen (eq (car c) 'on)
		 (dsbind (b1 b2) (cdr c)
		   (my- 0 (my/ (distance b1 b2 s) 2))))))))))



(defun distance (b1 b2 s)
  (reduce-set 
   #'mymin
   (disjuncts (formula s))
   :key #'(lambda (clause)
	    (let ((p1 (get-block-pos b1 clause))
		  (p2 (get-block-pos b2 clause)))
	      (if (and p1 p2)
		  (+ (abs-diff (first p1) (first p2))
		     (abs-diff (second p1) (1+ (second p2))))
		'infty)))))


(defun get-block-pos (b clause)
  (let ((held (get-clause-instances 'gripper-holding clause)))
    (if (member (list b) held :test #'equal)
	(check-not-null (car (get-clause-instances 'gripper-pos clause)))
      (cddr (find-element (conjuncts clause)
			  #'(lambda (x)
			      (and (length-exceeds x 1)
				   (eq (first x) 'block-pos)
				   (eq (second x) b))))))))
			     
	 



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; dummy descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-dummy-complete-descriptions (d)
  (make-ncstrips-schemas d
    (nav-right 
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
    
    (nav-left 
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
    
    (nav-up
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
    
    (nav-down
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))

    (navigate
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
    
    (nav
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
     
    (move-block
     :var-domains ((?b actual-blocks) (?a blocks) (?c blocks) (?xb columns) (?yb rows)
				      (?xc columns) (?yc all-rows) (?yt rows))
     :effects ((:precond 
		(and)
		:poss-add-list ((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
				(for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
				(for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
				(for-all ((?x columns) (?y rows)) t (free ?x ?y))
				(for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
				(for-all ((?c gripper-contents)) t (gripper-holding ?c)))
					 
		:poss-delete-list 
		((for-all ((?x columns) (?y rows)) t (gripper-pos ?x ?y))
		 (for-all ((?b blocks) (?x columns) (?y rows)) t (block-pos ?b ?x ?y))
		 (for-all ((?b blocks) (?c blocks)) t (on ?b ?c))
		 (for-all ((?x columns) (?y rows)) t (free ?x ?y))
		 (for-all ((?b blocks)) t (clear ?b)) (facer) (facel)
		 (for-all ((?c gripper-contents)) t (gripper-holding ?c))))))
    ))
    
(defun make-dummy-sound-descriptions (d)
  (make-ncstrips-schemas d
    (nav-right 
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ())
    (nav-left
     :var-domains ((?xs columns) (?xt columns) (?y rows))
     :effects ())
    (nav-up 
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ()
     )
    
    (nav-down 
     :var-domains ((?x columns) (?ys rows) (?yt rows))
     :effects ())
    
    (navigate
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ()
     )
    
    (nav
     :var-domains ((?xs columns) (?ys rows) (?xt columns) (?yt rows))
     :effects ()
     )
    (move-block
     :var-domains ((?b actual-blocks) (?a blocks) (?c blocks) (?xb columns) (?yb rows)
				      (?xc columns) (?yc all-rows) (?yt rows))
     :effects nil
     )
    ))     
     

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; misc
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun get-clause-instances (prop clause)
  
  (let ((pos nil)
	(neg nil))
    (do-elements (c (conjuncts clause) (values pos neg))
      (when (and (not (typep c 'boolean)) (eq prop (prop-symbol (literal-prop c))))
	(let ((args (prop-args (literal-prop c))))
	  (typecase c
	    (negation (push args neg))
	    (otherwise (push args pos))))))))

(defun dist (p p2)
  (+ (abs-diff (first p) (first p2))
     (abs-diff (second p) (second p2))))