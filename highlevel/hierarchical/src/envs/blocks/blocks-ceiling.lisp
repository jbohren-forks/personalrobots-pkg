(defpackage blocks
  (:use cl
	lookahead
	utils
	graph
	prop-logic
	set
	prod-set)
  (:export 
   
   ;; blocks-ceiling env
   make-blocks-world-with-ceiling
   gripper-pos
   gripper-holding
   faceR
   faceL
   nothing
   block-pos
   on
   turnL
   turnR
   left
   right
   up
   down
   int=
   pos-diff
   u
   r
   above
   right-of
   free
   xbetween
   clear
   pickupL
   pickupR
   stackL
   stackR
   ?x
   ?x1
   ?x2
   ?a
   ?b
   ?c
   ?xb
   ?xc
   ?xs
   ?xg
   ?xt
   ?y
   ?ys
   ?yt
   ?yg
   ?ybg
   ?picked-up-from
   ?picked-up
   ?stacked-on
   ?stacked
   columns
   rows
   blocks
   actual-blocks
   
   ;; hierarchy
   make-blocks-hierarchy
   make-unguided-blocks-hierarchy
   make-unguided-blocks-hierarchy2
   make-navigateless-hierarchy
   nav-left
   nav-right
   nav-up
   nav-down
   navigate
   nav
   move-block
   move-to

   ;; descriptions
   make-complete-descriptions
   make-sound-descriptions
   make-dummy-sound-descriptions
   make-dummy-complete-descriptions
   pg-heuristic
   dist-heuristic
   
   ;; subsumption
   make-blocks-subsumption-checker
   )
  )

(in-package blocks)


(defun make-blocks-world-with-ceiling (num-rows num-cols block-positions gripper-position goal)
  "
NUM-ROWS, NUM-COLS - positive integers
BLOCK-POSITIONS - association list from block names (symbols) to positions, which are lists of the form (X Y).  No block may have the name 'gripper, or a name of the form 'tI where I is an integer, since these names are reserved for the gripper and table blocks.
GRIPPER-POSITION - a position.
GOAL - DNF formula describing goal-set.  May use (ground instances of) the propositions (block-pos ?b ?x ?y), (gripper-pos ?x ?y), (on ?b ?c), (clear ?b), (right-of ?x1 ?x2), (above ?y1 ?y2).

Creates a blocks-world-with-ceiling domain of the given dimensions (where row 0 is the table), in which the blocks are the ones in BLOCKS-POSITIONS starting at the given positions, and the gripper starts at GRIPPER-POSITION. "
  
  (let ((actual-blocks (mapcar #'car block-positions))
	(table-blocks (mapset 'list #'base-block num-cols)))
    (let ((a (make-array (list num-cols num-rows) :initial-element nil))
	  (actual-positions (direct-product 'list num-cols (consec 1 (1- num-rows))))
	  (background (append
		       (mapset 'list #'(lambda (i) `(int= ,i ,i)) (max num-cols num-rows))
		       (mapset 'list #'(lambda (p) `(pos-diff ,@p))
			       (filter ':list
				       (direct-product 'list num-cols num-rows num-cols num-rows)
				       #'(lambda (p) (not (and (= (first p) (third p)) (= (second p) (fourth p)))))))
		       (mapset 'list #'(lambda (p) `(block-diff ,(first p) ,(second p)))
			       (filter ':list (direct-product 'list (append actual-blocks table-blocks) (append actual-blocks table-blocks))
				       #'(lambda (x) (not (eq (first x) (second x))))))
		       (mapset 'list #'(lambda (x) `(r ,(1+ x) ,x)) (1- num-cols))
		       (mapset 'list #'(lambda (y) `(u ,(1+ y) ,y)) (1- num-rows))
		       (mapset 'list #'(lambda (l) `(right-of ,(first l) ,(second l)))
			       (filter ':list
				       (direct-product 'list num-cols num-cols)
				       #'(lambda (x) (>= (first x) (second x)))))
		       (mapset 'list #'(lambda (l) `(above ,(first l) ,(second l)))
			       (filter ':list
				       (direct-product 'list num-rows num-rows)
				       #'(lambda (x) (>= (first x) (second x)))))
		       (mapset 'list #'(lambda (l) `(xbetween ,(first l) ,(second l) ,(third l)))
			       (filter ':list
				       (direct-product 'list num-cols num-cols num-cols)
				       #'(lambda (x)
					   (dsbind (i j k) x
					     (< i j k))))))))

				     
			  
    
      ;; make a 2d map to simplify life
      ;; first, add the table blocks
      (loop
	  for x below num-cols
	  for b in table-blocks
	  do (setf (aref a x 0) b))
    
      ;; next, add the blocks and gripper
      (dolist (tuple (append block-positions `((gripper ,@gripper-position))))
	(dsbind (name x y) tuple
	  (assert (null (aref a x y)) nil 
	    "~a and ~a have the same position" (aref a x y) name)
	  (setf (aref a x y) name)))
    
      (flet ((has-block (p) (not (member (aref a (first p) (second p)) '(nil gripper)))))

	;; Actually create the <propositional-domain> object
	(make-instance '<propositional-domain>
	  :types (p2alist 'columns num-cols 'all-rows num-rows 'rows (consec 1 (1- num-rows))
			  'blocks (disjoint-union actual-blocks table-blocks)
			  'actual-blocks actual-blocks
			  'ints (max num-cols num-rows)
			  'gripper-contents (disjoint-union actual-blocks '(nothing)))
		      
	  :fluents '((block-pos blocks columns all-rows)
		     (gripper-pos columns all-rows)
		     (gripper-holding gripper-contents)
		     (free columns rows)
		     (faceR)
		     (faceL)
		     (clear blocks)
		     (on blocks blocks))
	
	  :nonfluents '((r columns columns)
			(u all-rows all-rows)
			(block-diff blocks blocks)
			(xbetween columns columns columns)
			(above all-rows all-rows)
			(right-of columns columns)
			(pos-diff ints ints ints ints)
			(int= ints ints))
	
	  :pprint-state-fn #'pprint-bcs-state
      
	  :goal goal
	  :init-state 
	
	  (append background
			    
		  ;; Block positions
		  (mapset 'list #'(lambda (tuple) `(block-pos ,@tuple)) block-positions)
		  (mapcar #'(lambda (c s) `(block-pos ,s ,c 0)) (below num-cols) table-blocks)
       
		  ;; Gripper
		  `((gripper-pos ,@gripper-position) (gripper-holding nothing) (faceR))
		  
       
		  ;; Free
		  (mapset 'list #'(lambda (pos) `(free ,@pos))
			  (filter ':list actual-positions (fn (not has-block))))
       
		  ;; Clear
		  (mapset 'list #'(lambda (p) `(clear ,(aref a (first p) (1- (second p)))))
			  (filter ':list actual-positions
				  #'(lambda (p)
				      (and (not (has-block p))
					   (has-block (list (first p) (1- (second p))))))))
       
		  ;; On
		  (mapset 'list #'(lambda (p) (dsbind (x y) p `(on ,(aref a x y) ,(aref a x (1- y)))))
			  (filter ':list actual-positions
				  #'(lambda (p)
				      (and (has-block p) (has-block (list (first p) (1- (second p)))))))))
	
	  :action-descs
	  `((left ((?xs . columns) (?ys . rows) (?xg . columns))
		  (and (r ?xs ?xg) (gripper-pos ?xs ?ys) (free ?xg ?ys))
		  ((gripper-pos ?xg ?ys) )
		  ((gripper-pos ?xs ?ys) ))
	    (right ((?xs . columns) (?ys . rows) (?xg . columns))
	     (and (r ?xg ?xs) (gripper-pos ?xs ?ys) (free ?xg ?ys))
	     ((gripper-pos ?xg ?ys) )
	     ((gripper-pos ?xs ?ys) ))
	    (up ((?xs . columns) (?ys . rows) (?yg . rows))
	     (and (u ?yg ?ys) (gripper-pos ?xs ?ys) (free ?xs ?yg))
	     ((gripper-pos ?xs ?yg) )
	     ((gripper-pos ?xs ?ys) ))
	    (down ((?xs . columns) (?ys . rows) (?yg . rows))
	     (and (u ?ys ?yg) (gripper-pos ?xs ?ys) (free ?xs ?yg))
	     ((gripper-pos ?xs ?yg) )
	     ((gripper-pos ?xs ?ys) ))
	    (turnL 
	     ((?x . columns))
	     (and (gripper-pos ?x ,(1- num-rows)))
	     ((faceL))
	     ((faceR)))
	    (turnR 
	     ((?x . columns))
	     (and (gripper-pos ?x ,(1- num-rows)))
	     ((faceR))
	     ((faceL)))
	    (pickupL ((?picked-up . blocks) (?picked-up-from . blocks) (?xb . columns) (?yg . rows) (?xg . columns))
	     (and (faceL) (gripper-holding nothing) (r ?xg ?xb) (block-pos ?picked-up ?xb ?yg) 
	      (gripper-pos ?xg ?yg) (clear ?picked-up) (on ?picked-up ?picked-up-from))
	     ((gripper-holding ?picked-up) (faceL) (clear ?picked-up-from) (free ?xb ?yg))
	     ((on ?picked-up ?picked-up-from) (faceR) (clear ?picked-up) (block-pos ?picked-up ?xb ?yg) (gripper-holding nothing)))
	    (pickupR ((?picked-up . blocks) (?picked-up-from . blocks) (?xb . columns) (?yg . rows) (?xg . columns))
	     (and (faceR) (gripper-holding nothing) (r ?xb ?xg) (block-pos ?picked-up ?xb ?yg) 
	      (gripper-pos ?xg ?yg) (clear ?picked-up) (on ?picked-up ?picked-up-from) )
	     ((gripper-holding ?picked-up) (faceR) (clear ?picked-up-from) (free ?xb ?yg) )
	     ((on ?picked-up ?picked-up-from) (faceL) (clear ?picked-up) (block-pos ?picked-up ?xb ?yg) (gripper-holding nothing)))
	    (stackL ((?stacked . blocks) (?stacked-on . blocks) (?xb . columns) (?ybg . all-rows) (?xg . columns) (?yg . rows))
	     (and (faceL) (gripper-holding ?stacked) (r ?xg ?xb) (u ?yg ?ybg) (gripper-pos ?xg ?yg) 
	      (block-pos ?stacked-on ?xb ?ybg) (clear ?stacked-on))
	     ((block-pos ?stacked ?xb ?yg) (faceL) (clear ?stacked) (on ?stacked ?stacked-on) (gripper-holding nothing))
	     ((clear ?stacked-on) (free ?xb ?yg) (faceR) (gripper-holding ?stacked)))
	    (stackR ((?stacked . blocks) (?stacked-on . blocks) (?xb . columns) (?ybg . all-rows) (?xg . columns) (?yg . rows))
	     (and (faceR) (gripper-holding ?stacked) (r ?xb ?xg) (u ?yg ?ybg) (gripper-pos ?xg ?yg) 
	      (block-pos ?stacked-on ?xb ?ybg) (clear ?stacked-on) )
	     ((block-pos ?stacked ?xb ?yg) (faceR) (clear ?stacked) (on ?stacked ?stacked-on) (gripper-holding nothing))
	     ((clear ?stacked-on) (free ?xb ?yg) (faceL) (gripper-holding ?stacked)))

	    )
	
	  :functional-deps
	  '((gripper-pos)
	    (r 0)
	    (r 1)
	    (u 0)
	    (u 1)
	    (on 0)
	    (on 1)
	    (faceR)
	    (faceL)
	    (block-pos 0)
	    (block-pos 1 2)
	    (gripper-holding))
	  ))))	)
	

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; accessing parameters of blocks world from state
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun num-rows (s)
  (lookup-type
   (etypecase s (<propositional-domain> s) (prop-domain-state (pds-domain s)))
   'all-rows))

(defun num-cols (s)
  (lookup-type
   (etypecase s (<propositional-domain> s) (prop-domain-state (pds-domain s)))
   'columns))

(defun gripper-holding (s)
  (check-not-null
      (do-elements (x (pds-props s))
	(when (eq (first x) 'gripper-holding) (return (second x))))))

(defun facing (s)
  (let ((props (pds-props s)))
    (if (member? '(faceR) props)
	(if (member? '(faceL) props)
	    'right-left
	  'right)
      (if (member? '(faceL) props)
	  'left
	'neither-way))))

(defun blocks (s)
  (lookup-type
   (etypecase s (<propositional-domain> s) (prop-domain-state (pds-domain s)))
   'actual-blocks))

(defun all-blocks (s)
  (lookup-type
   (etypecase s (<propositional-domain> s) (prop-domain-state (pds-domain s)))
   'blocks))

(defun state->list (s)
  (let* ((blocks (blocks s))
	 (l (mapcar #'(lambda (x) (cons x 'unknown)) blocks))
	 (gp 'unknown)
	 (gf 'unknown))
    (do-elements (p (pds-props s) (list* gf gp l))
      (dsbind (name . args) p
	(case name
	  (block-pos (let ((entry (assoc (car args) l))) (awhen entry (setf (cdr it) (cdr args)))))
	  (gripper-pos (setf gp args))
	  (facer (setf gf 'right))
	  (facel (setf gf 'left))
	  (gripper-holding (let ((entry (assoc (car args) l))) (awhen entry (setf (cdr it) 'gripper)))))))))

(defun sound-set-state (s d)
  (unless (typep s 'list)
    (let ((dis (disjuncts (formula s))))
      (when (= 1 (size dis))
	(let ((con (conjuncts (item 0 dis)))
	      (num-props (size (conjuncts (item 0 (disjuncts (formula (init-state-set d))))))))
	  (when (= (size con) num-props)
	    (make-prop-domain-state :domain d :props con)))))))
    
(defun base-block (i)
  "Return the symbol for the base block in column I"
  (intern (format nil "T~a" i) 'keyword))

    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debugging
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun pprint-bcs-state (str s)
  (let* ((p (pds-props s))
	 (nr (num-rows s))
	 (nc (num-cols s))
	 (a (make-array (list nc nr) :initial-element #\.))
	 (holding (gripper-holding s)))

    (do-elements (prop p)
      (case (first prop)
	(block-pos (dsbind (name x y) (rest prop)
		     (assert (eql #\. (aref a x y)) ()
		       "Location (~a, ~a) contained ~a and ~a" x y name (aref a x y))
		     (setf (aref a x y) (char-upcase (aref (symbol-name name) 0)))))
	(gripper-pos (dsbind (x y) (rest prop)
		       (assert (eql #\. (aref a x y)) ()
			 "Location (~a, ~a) contained gripper and ~a" x y (aref a x y))
		       (setf (aref a x y)
			 (if (eq holding 'nothing)
			     #\G
			   (char-downcase (aref (symbol-name holding) 0))))))))
    (pprint-logical-block (str nil)
      (for-loop (r (1- nr) 0 -1 #'<)
		(dotimes (c nc)
		  (write-char (aref a c r) str))
	(format str "~@:_"))
      (format str "Facing ~a~@:_" (facing s)))
    (values)))

