(defpackage :test-dependency-graph 
    (:use :dependency-graph :cl :utils :set :mapping))
  
(in-package :test-dependency-graph)

(defparameter g (make-dependency-graph))
(defparameter foo 0)
(defparameter bar 2)
(defparameter h (make-dependency-graph))

;; Graph g
;; a -> b -> d
;;   /     /
;; c --> e
;; b: +
;; d: *
;; e: +
;;
;; Graph h
;; x -> z
;;   / 
;; y
;; z: x-y
(tests "dependency graph"
  ((variables g) nil)
  ((out-of-date-variables g) nil)
  ((progn (add-variable g 'a :external)
	  (variables g))
   '(a))
  ((out-of-date-variables g) nil)
  ((progn (add-variable g 'b :internal
			:update-fn #'(lambda (parent-vals parent-diffs old-val s)
				       (declare (ignore parent-diffs old-val))
				       (if s
					 (values s (new-val-diff s) nil t)
					 (let ((saved (reduce #'+ parent-vals :key #'cdr))) (values (/ saved 2) (new-val-diff (/ saved 2)) saved nil))))
			:dependees '(a) 
			:update-hooks (list #'(lambda (v d) (declare (ignore v)) (multf bar (new-val d)))))
	  (variables g))
   '(a b) #'set-eq)
  ((out-of-date-variables g) '(b))
  ((results-in-error t (up-to-date-value g 'a)) t) ;; Can't get up-to-date value of uninitialized external var
  ((results-in-error t (do-next-update g)) t) ;; Can't update B because a is uninitialized
  ((progn (update-external-variable g 'a (new-val-diff 3))
	  (out-of-date-variables g)) 
   '(b))
  ((results-in-error t (current-value g 'b)) t) ;; Can't get value of uninitialized internal var
  ((do-next-update g) 'b)
  ;; a = 3
  ((out-of-date-variables g) '(b))
  ((current-value g 'b) 1.5)
  (bar 3)
  ;; update: b <- 3, bar <- 9

  ((do-next-update g) 'b)
  ((out-of-date-variables g) nil)
  ((current-value g 'a) 3)
  ((current-value g 'b) 3)
  (bar 9)
  ((results-in-error t (update-external-variable g 'b (new-val-diff 4))) t) ;; Can't set value of internal variable
  ((progn (update-external-variable g 'a (new-val-diff 5))
	  (add-variable h 'x :external)
	  (add-variable h 'y :external)
	  (add-variable h 'z :internal :dependees '(x y) 
			:simple-update-fn (make-alist-function (x y) (- x y)))
	  (tie-variables g 'b h 'x)
	  (add-variable g 'c :external :dependants '(b))
	  (add-variable g 'd :internal :update-fn (make-simple-aggregator #'*)
			:dependees '(b) :update-hooks (list #'(lambda (v d) (declare (ignore d)) (incf foo v))))
	  (add-variable g 'e :internal :update-fn (make-simple-aggregator #'+)
			:dependees '(c) :dependants '(d))
	  (variables g))
   '(a b c d e) #'set-eq)
  ;; set a to 5, Added h, tied b to x so x <- 3
  ((out-of-date-variables h) '(z))
  ((results-in-error t (tie-variables g 'd h 'x)) t) ;; Can't tie x to two different things
  ((current-value h 'x) 3)
  ((out-of-date-variables g) '(b d e) #'set-eq)
  ((up-to-date? 'b g) nil)
  ((up-to-date? 'a g) t)
  ((progn (update-external-variable g 'a (new-val-diff 7))
	  (current-value g 'a))
   7)
  ;; a <- 7
  ((current-value g 'b) 3)
  ((results-in-error t (up-to-date-value g 'b)) t) ;; Can't get up-to-date value as c is uninitialized

  ((out-of-date-variables g) '(d b e) #'set-eq)
  ((progn (update-external-variable g 'c (new-val-diff 8))
	  (out-of-date-variables g)) 
   '(b d e) #'set-eq)
  ((progn (do-next-update g)
	  (do-next-update g)
	  (do-next-update g)
	  (out-of-date-variables g))
   '(d))
  ;; B should have been incrementally updated twice, e once.  Bar therefore got multiplied twice, once by 7.5 and once by 15
  (bar 1012.5)

  ((out-of-date-variables h) '(z))
  ((current-value h 'x) 15)
  ((results-in-error t (tie-variables g 'd h 'z)) t) ;; Can't tie to internal var
  ((progn (tie-variables g 'd h 'y)
	  (out-of-date-variables h))
   '(z))
  ((results-in-error t (up-to-date-value h 'y)) t) ;; Uninitialized - tied to d but d isn't initialized either
  ((progn (update-external-variable h 'y (new-val-diff 3))
	  (do-next-update h)
	  (out-of-date-variables h))
   nil)
  ((current-value h 'z) 12)

  ((current-value g 'e) 8)
  ((progn (update-external-variable g 'a (new-val-diff 2))
	  (up-to-date-value g 'd))
   80)
  ;; b should have been updated (twice) to 10, d to 80 (10*8), bar multiplied once by 10, foo incremented once by value of d (80)
  (foo 80)
  (bar 10125)
  ((out-of-date-variables g) nil)
  ((out-of-date-variables h) '(z))
  ((current-value h 'z) 12)
  ((up-to-date-value h 'z) -70)
  ((out-of-date-variables h) nil))