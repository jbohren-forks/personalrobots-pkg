(defpackage nav-switch
  (:documentation "Package nav-switch

<nav-switch>
make-nav-switch-world
ns-act-complete
ns-act-sound
nav-switch-hierarchy
ns-descs
subsumption-key

{u,l,r,d}-{good,bad}
flipH
flipV

nav")
  
  (:export
   
   <nav-switch>
   make-nav-switch-world
   random-ns-world
   ns-act-complete
   ns-act-sound
   nav-switch-hierarchy
   ns-descs
   subsumption-key
   
   r u d l f
   u-good
   u-bad
   l-good
   l-bad
   r-good
   r-bad
   d-good
   d-bad
   flipH
   flipV
   
   nav)
  
  (:import-from
   prop-logic
   
   prop-args
   prop-symbol
   conjuncts
   disjuncts
   formula
   negation)
  
  (:use
   cl
   lookahead
   utils
   prob
   set
   prod-set))

(in-package nav-switch)


	 
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Constructor for the prop domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <nav-switch-world> (<propositional-domain>)
  ((good-move-cost :initarg :good-move-cost :reader good-move-cost)
   (bad-move-cost :initarg :bad-move-cost :reader bad-move-cost)
   (switches :initarg :switches :reader switches)
   (goal-prop :reader goal-prop :initarg :goal)
   (num-rows :initarg :num-rows)
   (num-cols :initarg :num-cols)))
  

(defun random-ns-world (nr nc ns &optional (good 2) (bad 5))
  "random-ns-world NR NC NS &optional (GOOD-MOVE-COST 2) (BAD-MOVE-COST 5)
Return a random nav-switch world with the given dimensions, and NS switches"
  (let* ((r (floor nr 2))
	 (c (floor nc 2))
	 (init (list (floor c 2) (floor r 2)))
	 (goal (list (+ c (floor c 2)) (+ r (floor r 2))))
	 (switches (list init goal)))
    (until (= (length switches) (+ 2 ns))
	   (pushnew (list (random nc) (random nr)) switches :test #'equal))
    (print init)
    (print goal)
    (print (setf switches (cddr (nreverse switches))))
    (make-nav-switch-world nr nc switches goal init :good-move-cost good :bad-move-cost bad)))

(defun make-nav-switch-world (nr nc switches goal init-loc 
			      &key (good-move-cost 2) (bad-move-cost 5)
			      &aux (good-rew (- good-move-cost)) (bad-rew (- bad-move-cost)))
								
  "make-nav-switch-world NR NC SWITCHES GOAL INIT-LOC &key (GOOD-MOVE-COST 2) (BAD-MOVE-COST 5)
NR, NC: fixnums
SWITCHES: list of switch locations of form (x y)
GOAL: goal location
INIT-LOC: Init location

GOOD-MOVE-COST, BAD-MOVE-COST: cost of moving in and against the direction of the switches respectively (flipping always has cost 1)."
  
  (make-instance '<nav-switch-world>
    :types (p2alist 'columns nc 'rows nr 'ints (max nr nc))

    :fluents '((at columns rows)
	       (horiz)
	       (vert))
    
    :nonfluents '((switch-at columns rows)
		  (r columns columns)
		  (int= ints ints)
		  (int-diff ints ints)
		  ;;;(pos-diff ints ints ints ints)
		  (u rows rows)
		  (goal columns rows))
    
    :functional-deps '((at)
		       (r 0)
		       (r 1)
		       (int= 0)
		       (int= 1)
		       (u 0)
		       (u 1)
		       (goal))
		       
    :pprint-state-fn #'pprint-nav-switch-state
    
    :goal (cons 'at goal)
    
    :good-move-cost good-move-cost
    :bad-move-cost bad-move-cost
    :num-rows nr
    :num-cols nc
    :switches switches
    
    :init-state (list* (cons 'at init-loc) '(horiz)
		       (background-preds nr nc goal switches))
    
    :action-descs 
    `((l-good
       ((?xs . columns) (?ys . rows) (?xg . columns))
       (and (r ?xs ?xg) (at ?xs ?ys) (horiz))
       ((at ?xg ?ys))
       ((at ?xs ?ys))
       ,good-rew)
      
      (l-bad
       ((?xs . columns) (?ys . rows) (?xg . columns))
       (and (r ?xs ?xg) (at ?xs ?ys) (vert))
       ((at ?xg ?ys))
       ((at ?xs ?ys))
       ,bad-rew)
      
      (r-good 
       ((?xs . columns) (?ys . rows) (?xg . columns))
       (and (r ?xg ?xs) (at ?xs ?ys) (horiz))
       ((at ?xg ?ys))
       ((at ?xs ?ys))
       ,good-rew)
      
      (r-bad 
       ((?xs . columns) (?ys . rows) (?xg . columns))
       (and (r ?xg ?xs) (at ?xs ?ys) (vert))
       ((at ?xg ?ys))
       ((at ?xs ?ys))
       ,bad-rew)
      
      (u-good
       ((?xs . columns) (?ys . rows) (?yg . rows))
       (and (u ?yg ?ys) (at ?xs ?ys) (vert))
       ((at ?xs ?yg))
       ((at ?xs ?ys))
       ,good-rew)
      
      (u-bad
       ((?xs . columns) (?ys . rows) (?yg . rows))
       (and (u ?yg ?ys) (at ?xs ?ys) (horiz))
       ((at ?xs ?yg))
       ((at ?xs ?ys))
       ,bad-rew)
      
      (d-good
       ((?xs . columns) (?ys . rows) (?yg . rows))
       (and (u ?ys ?yg) (at ?xs ?ys) (vert))
       ((at ?xs ?yg))
       ((at ?xs ?ys))
       ,good-rew)
      
      
      (d-bad
       ((?xs . columns) (?ys . rows) (?yg . rows))
       (and (u ?ys ?yg) (at ?xs ?ys) (horiz))
       ((at ?xs ?yg))
       ((at ?xs ?ys))
       ,bad-rew)
      
      (flipH
       ((?xs . columns) (?ys . rows))
       (and (at ?xs ?ys) (switch-at ?xs ?ys) (vert))
       ((horiz))
       ((vert)))
      
      (flipV
       ((?xs . columns) (?ys . rows))
       (and (at ?xs ?ys) (switch-at ?xs ?ys) (horiz))
       ((vert))
       ((horiz))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; state accessors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun switch-dir (s) 
  (if (holds s '(horiz)) 
      'H 
    'V))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun background-preds (nr nc goal switch-locs)
  "generate background nonfluent predicates"
  (let ((m (max nc nr)))
    (append
     (mapset 'list #'(lambda (x) `(r ,(1+ x) ,x)) (1- nc))
     (list (cons 'goal goal))
     (mapset 'list #'(lambda (x) `(u ,(1+ x) ,x)) (1- nr))
     (mapset 'list #'(lambda (i) `(int= ,i ,i)) m)
     (mapset 'list #'(lambda (l) (cons 'int-diff l))
	     (filter ':implicit (direct-product 'list m m)
		     #'(lambda (x) (not (= (first x) (second x))))))
     #| (mapset 'list #'(lambda (p) (cons 'pos-diff p))
	   (filter ':implicit
		   (direct-product 'list nc nr nc nr)
		   #'(lambda (p) (not (and (= (first p) (third p)) (= (second p) (fourth p))))))) |#
     (mapcar #'(lambda (loc) (cons 'switch-at loc)) switch-locs))))
		 

(defun pos (s)
  (with-slots (num-rows num-cols) (pds-domain s)
    (check-not-null
     (find-element (fast-product (list num-cols num-rows))
		   #'(lambda (l) (holds s (cons 'at l)))))))

(defun orientation (s)
  (if (holds s '(vert))
      'vert
    'horiz))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun pprint-nav-switch-state (str s)
  (let* ((d (pds-domain s))
	 (switch-char (if (eq (switch-dir s) 'H) #\H #\V))
	 (nr (lookup-type d 'rows))
	 (nc (lookup-type d 'columns))
	 (m (make-array (list nc nr) :initial-element #\.)))
    
    (dotimes (x nc)
      (dotimes (y nr)
	(when (holds-background d (list 'switch-at x y))
	  (setf (aref m x y) switch-char))
	(when (holds s (list 'at x y))
	  (setf (aref m x y) #\A))))
    
    (pprint-logical-block (str nil)
      (for-loop (r (1- nr) 0 -1 #'<)
	(dotimes (c nc)
	  (write-char (aref m c r) str))
	(pprint-newline :mandatory str)))))

(defmethod env-user:translate-action ((e <nav-switch-world>) a s choices)
  (declare (ignore s))
  (setf a 
    (cond
     ((eq a 'r) '(r-bad r-good))
     ((eq a 'u) '(u-bad u-good))
     ((eq a 'd) '(d-bad d-good))
     ((eq a 'l) '(l-bad l-good))
     ((eq a 'f) '(flipv fliph))))
  
  (or (find-element choices #'(lambda (c) (member (car c) a)))
      (error 'env-user:untranslatable-action)))
  

