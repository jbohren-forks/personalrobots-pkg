(defpackage test-prop-logic
  (:use 
   cl
   utils
   prod-set
   set
   prop-logic))

(in-package test-prop-logic)


(defvars f1 f2 f3 f4 f5 f6 b s2 s3 s5 s6 s7 s)
(setf f1 '(foo))
(setf f2 (disjoin '(foo) '(bar)))
(setf f3 (conjoin '(foo) (negate '(bar 4))))
(setf f4 (disjoin t (conjoin '(foo) '(bar)) (conjoin (negate '(bar)) (negate '(baz)))))
(setf f5 (negate '(qux)))
(setf f6 (conjoin '(foo) (disjoin '(bar) '(baz))))


(do-boolean-tests 
    "Propositional formula types"
  (is-formula t) t
  (is-formula nil) t
  (is-formula '(foo)) t
  (is-formula 3) nil
  (is-formula '(a b 6)) t 
  (is-formula '(a 4 (b 4))) nil
  (is-formula '(or)) t
  (is-formula '(and nil (or (and (foo) t) (and (not (bar 3)))) (not (and (bar) (or (baz) nil))))) t
  (is-formula '(and nil (or (and (foo) t) (and (not (bar 3)))) (not (and (bar) (or (baz) nil))))) t
  (typep f1 'literal) t
  (typep t 'literal) nil
  (typep nil 'literal) nil
  (typep f5 'literal) t
  (typep f2 'literal) nil
  (typep f3 'literal) nil
  (is-dnf-clause f1) t
  (is-dnf-clause f2) nil
  (is-dnf-clause f3) t
  (is-dnf-clause t) nil
  (is-dnf-clause nil) nil
  (is-dnf-clause f4) nil
  (is-dnf-formula t) nil
  (is-dnf-formula nil) nil
  (is-dnf-formula f1) nil
  (is-dnf-formula f2) t
  (is-dnf-formula f3) nil
  (is-dnf-formula f4) nil)
  

(setf f1 '(and (foo x (a 3)) (not (bar y)) (not (or (x (b 2) y))))
      b '((x . y) (y . 24)))


(do-tests
    "Substitution"
  (setf f2 (bind f1 b)) '(and (foo y (a 3)) (not (bar 24)) (not (or (x (b 2) 24))))
  f1 '(and (foo x (a 3)) (not (bar y)) (not (or (x (b 2) y))))
  (bind '(x) b) '(x)
  (bind '(and t nil (foo y)) b) '(and t nil (foo 24)))

(let ((s1 nil)
      (s2 '((foo) (bar) (baz)))
      (s3 '((baz) (qux a 3 4) (foo))))
  (do-boolean-tests
      "Proposition formula holds function"
    (holds s1 t) t
    (holds s3 t) t
    (holds s1 nil) nil
    (holds s2 nil) nil
    (holds s1 '(bar)) nil
    (holds s2 '(bar)) t
    (holds s3 '(bar)) nil
    (holds s1 '(foo)) nil
    (holds s3 '(foo)) t
    (holds s1 '(not (qux))) t
    (holds s2 '(not (qux))) t
    (holds s3 '(not (qux a 3 4))) nil
    (holds s2 '(or (and (foo) (bar)) (not (qux)))) t
    (holds s3 '(or (and (foo) (bar)) (not (qux a 3 4)))) nil
    (holds s1 '(or (and (foo) (bar)) (not (qux)))) t))





(let ((a (make-prop-set 'foo 3 '(x y))))
  (do-boolean-tests
      "Proposition sets"
    (member? '(foo 2 x) a) t
    (member? '(foo 3 x) a) nil
    (member? '(foo 1) a) nil
    (member? '(bar 2 y) a) nil))




(setf f1 '(or (and (foo) (bar)) (and (not (baz))) (and (not (bar)) (foo))))
(setf f2 '(or (and (baz) (qux)) (and (not (qux)) (not (foo)))))

(defun same-clause (c1 c2)
  (let ((con1 (conjuncts c1))
	(con2 (conjuncts c2)))
    (set-eq con1 con2)))

(defun same-dnf-formula (f1 f2)
  (each (disjuncts f1)
	#'(lambda (c1)
	    (any (disjuncts f2)
		 #'(lambda (c2) (same-clause c1 c2))))))

    


(tests 
 "DNF operations"
 ((dnf-or '(or (and (foo) (bar)) (and (baz) (qux))) '(or (and (bar)) (and (qux) (baz))))
  '(or (and (foo) (bar)) (and (baz) (qux)) (and (bar))) #'same-dnf-formula)
 ((dnf-or '(or (and)) '(or)) '(or (and)) #'same-dnf-formula)
 ((dnf-and '(or (and (foo) (bar)) (and (not (foo)) (baz))) '(or (and (qux) (foo)) (and (foo) (not (bar)))))
  '(or (and (foo) (qux) (bar))) #'same-dnf-formula)
 ((dnf-not '(or (and (foo) (bar)) (and (not (foo)) (baz))))
  '(or (and (not (foo)) (not (baz))) (and (not (bar)) (foo)) (and (not (bar)) (not (baz))))
  #'same-dnf-formula)
 ((dnf-implies '(or (and)) '(or)) nil)
 ((dnf-implies '(or) '(or (and))) t)
 ((dnf-implies '(or (and (x))) '(or (and (y)) (and (not (y))))) t)
 ((dnf-implies '(or (and (x))) '(or (and (y)) (and (not (y)) (not (x))))) nil)
 ((dnf-implies '(or (and (x) (a)) (and (z) (b))) '(or (and (y) (a)) (and (not (y)) (b)))) nil)
 ((dnf-implies '(or (and (x) (a)) (and (z) (b))) 
	       '(or (and (y) (a)) (and (not (y)) (b)) (and (x) (a) (not (b))) (and (z) (y) (b)))) t)
 ((dnf-implies '(or (and (x) (a)) (and (z) (b))) 
	       '(or (and (y) (a)) (and (not (y)) (b)) (and (x) (a) (not (b))) )) nil)
 ((dnf-implies '(or (and (x) (a)) (and (z) (b))) 
	       '(or (and (y) (a)) (and (not (y)) (b))  (and (z) (y) (b)))) nil)
 ((dnf-implies '(or (and (x) (a)) (and (z) (b))) 
	       '(or (and (y) (a)) (and (not (y)) (b)) (and (x) (a) (not (b))) (and (z) (y) (not (b))))) nil))



      
(setf s (make-dnf-set f1 '((foo) (bar) (qux) (baz)))
      s2 (make-instance '<dnf-set> :formula f2 :props '((foo) (bar) (qux) (baz)))
      s3 (intersect s s2)
      ;;s4 (binary-union s s2)
      f3 '(or (and (not (qux)) (not (foo))) (and (baz) (qux)))
      s5 (make-instance '<dnf-set> :formula f3 :props '((foo) (bar) (qux) (baz)))
      s6 (make-instance '<dnf-set> :formula '(or (and (x) (a)) (and (z) (b))) :props '((x) (a) (z) (b)))
      s7 (make-instance '<dnf-set> :formula '(or (and (y) (a)) (and (not (y)) (b)) (and (x) (a) (not (b)))
					      (and (z) (y) (b)))
			:props '((x) (a) (z) (b))))



(do-tests
    "DNF sets"
  (member? '((foo) (bar) (qux)) s) t
  (member? '((baz)) s) nil
  (member? '((foo) (bar) (qux)) s3) nil
  (member? '((foo) (baz) (qux)) s3) t
  ;;(member? '((foo) (bar) (qux)) s4) t
  ;;(member? '((foo) (baz) (qux)) s4) t
  (size-must-exceed s2 4) nil
  (size-must-exceed s2 3) t
  (set-eq s2 s5) t
  (set-eq s2 s) nil
  (subset s6 s7) t
  (subset s7 s6) nil
  )
