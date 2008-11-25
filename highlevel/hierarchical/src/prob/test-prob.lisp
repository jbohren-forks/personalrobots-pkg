(defpackage test-prob
  (:use cl 
	utils
	set
	lin-alg
	prob)
  
  )

(in-package test-prob)


(defvars mu sigma p trans cov bias d p2 y m v p3 mg mlg tmg g1 g2 g3 lg g cd tab e s be bs)

(defparameter *tol* .05)
(defun close-to (x y)
  (if (numberp x)
      (< (abs-diff x y) *tol*)
    (< (lp-dist x y 'infty) *tol*)))

(setf mu #(2 3)
      sigma #2A((1 -1) (-1 5))
      p (make-instance '<gaussian> :mean mu :cov sigma)
      trans #2A((1 2) (3 4) (5 6))
      cov #2A((4 -4 6) (-4 13 -3) (6 -3 11))
      bias #(-1 -1 2)
      d (make-instance '<linear-gaussian> :trans trans :bias bias :cov cov)
      p2 (transform-dist p d)
      y #(5 1 1)
      m #(-5.489948 4.411026)
      v #2A((0.3350706 -0.29932904) (-0.29932976 0.427392))
      p3 (condition-on-dist p d y)
      mg (mixture-of-gaussians #(.3 .5 .2) #(5 -1 3) #(8 5 1))
      mlg (mixture-of-linear-gaussians #(.8 .2) #(3 4) #(1 -1) #(2 6))
      tmg (transform-dist mg mlg)
      g1 (make-instance '<gaussian> :mean 3 :cov 2)
      g2 (make-instance '<gaussian> :mean 5 :cov 1)
      g3 (make-instance '<gaussian> :mean 0 :cov 4)
      lg (mixture-of-linear-gaussians #(.2 .8) #(4 3) #(-1 -2) #(3 4))
      g (mixture-of-gaussians #(.3 .3 .4) #(3 5 0) #(2 1 4))
      cd (condition-on-dist g lg 4)
      )

(tests
 "Gaussians"
 ((mean p2) #(7 17 30))
 ((covariance p2) #2A((21 29 55) (29 78 94) (55 94 156)))
 ((mean p3) m #'close-to)
 ((covariance p3) v #'close-to)
 ((map 'vector #'mean (mixture-components tmg)) #(16 19 -2 -5 10 11) #'close-to)
 ((mixture-weights tmg) #(.24 .06 .4 .1 .16 .04) #'close-to))
 


(setf tab #2A((19 4 0 0 0 0) (11 63 64 3 1 0) (2 16 18 20 2 2) (1 4 1 9 6 2) (0 0 1 2 4 3) (0 0 0 1 1 1)))
(dotimes (i 6)
  (dotimes (j 6)
    (divf (aref tab i j) 261)))
(setf d (make-instance '<tabular-dist> :table tab)
      e #'first
      s #'second
      be #'(lambda (x) (indicator (> (first x) 2)))
      bs #'(lambda (x) (indicator (< (second x) 3))))
      

(tests 
 "Information-theory"
 ((entropy d) 3.46 #'close-to)
 ;((mutual-information d e s) .58 #'close-to)
 ;((conditional-entropy d be bs) .39 #'close-to)
 ;((conditional-entropy d bs be) .56 #'close-to))
)
