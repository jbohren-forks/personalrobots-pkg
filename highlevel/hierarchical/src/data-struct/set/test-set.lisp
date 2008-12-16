(defpackage test-set
  (:use
   cl
   utils
   set
   prod-set))


(in-package test-set)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; set of k-size subsets of a set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars s1 s2 s3 s4 s5 s6 doms s sub iter)

(setf s1 '(a b c d)
      s2 (make-instance '<subsets-of-size> :base-set s1 :size 0)
      s3 (make-instance '<subsets-of-size> :base-set s1 :size 1)
      s4 (make-instance '<subsets-of-size> :base-set s1 :size 2)
      s5 (make-instance '<subsets-of-size> :base-set s1 :size 4)
      s6 (make-instance '<subsets-of-size> :base-set s1 :size 5))

(defun size-equals (s k)
  (= (sum-over s (constantly 1)) k))

(tests 
 "Subsets-of-size"
 (s2 1 #'size-equals)
 (s3 4 #'size-equals)
 (s4 6 #'size-equals)
 (s5 1 #'size-equals)
 (s6 0 #'size-equals)
 (s4 '((b a) (c a) (d a) (c b) (d b) (d c)) #'set-eq))
       
      


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; direct products
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf s1 (direct-product 'list '(foo bar) 3)
      s2 (make-instance '<prod-set> :inst-acc (inst-vars:make-list-accessors 2) :iterate-quickly t
			:sets '((foo bar) 3)))


(defun foo (s)
  (let ((iter (iterator s)))
    (list
     (funcall iter)
     (funcall iter))))

(defun bar (s)
  (let ((iter (iterator s)))
    (flet ((adder (n) (lambda (x) (+ x (second n)))))
      (let ((a (adder (funcall iter))))
	(loop repeat 4 do (funcall iter))
	(let ((b (adder (funcall iter))))
	  (list
	   (funcall a 10)
	   (funcall b 10)))))))


(do-tests
    "direct product sets"
  
  (foo s1)
  '((foo 0) (foo 1))
  
  (foo s2)
  '((foo 1) (foo 1))
  
  (bar s1)
  '(10 12)
  
  (bar s2)
  '(12 12)

  (list (size s1) (size s2))
  '(6 6))
  
  
  
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; subspaces
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
  


(setf doms 
  (list #(a b c) 4 6 '(foo bar) (direct-product 'list '(baz qux) 2)))

(setf s (apply #'direct-product 'vector doms))

(setf sub (make-subspace s '(1 3 0)))

(setf iter (iterator sub))

(do-tests
    "direct product subspaces"
  (item 0 sub) #(a 0 not-used foo not-used)
  (item 10 sub) #(b 1 not-used bar not-used)
  (size sub) 24
  (member? #(c 3 not-used bar not-used) sub) t
  (member? #(c 3 nt-used bar nil) sub) nil
  (member? #(c 3 not-used ba not-used) sub) nil
  (item-number #(c 3 not-used bar not-used) sub) 23
  (funcall iter) #(a 0 not-used foo not-used)
  (loop repeat 6 do (funcall iter) finally (return (funcall iter)))
  #(b 1 not-used foo not-used))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; recursive enumeration
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf s 
  (make-instance '<recursive-enumeration>
    :init-state-fn (constantly 1)
    :trans-fn #'(lambda (x) (let ((y (* x 2))) (if (> y 100) ':no-more-elements y)))
    :output-fn #'1+))
;; s = {1 + powers of 2 that are less than 100}

(do-tests
    "recursive enumerations"
  (item 3 s) 9
  (item-number 65 s) 6
  (handler-case (item-number 129 s) (item-not-in-set (c) (declare (ignore c)) 'success)) 'success
  (handler-case (item 10 s) (index-out-of-bounds (c) (declare (ignore c)) 'success)) 'success)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; nondeterministic sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf s (ndlet ((i 10)) (* i i))
      s2 (ndunion ((i 5)) (ndlet ((j i)) (list i (* j j))))
      s3 (ndlet-fail ((x (direct-product 'list 5 5)))
	   (dbind (a b) x
	     (if (>= a b) 
		 'fail
	       (cons a b)))))
	       

(do-tests
    "nondeterministic sets"
  (size s) 10
  (member? 36 s) t
  (member? 37 s) nil
  (member? 100 s) nil
  (member? 0 s) t
  (size s2) 10
  (member? '(2 4) s2) nil
  (member? '(3 4) s2) t
  (set-eq s3 '((0 . 1) (0 . 2) (0 . 3) (0 . 4) (1 . 2) (1 . 3) (1 . 4) (2 . 3) (2 . 4) (3 . 4))) t
  )
  
