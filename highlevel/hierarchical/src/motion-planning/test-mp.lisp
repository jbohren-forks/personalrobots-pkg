(defpackage test-mp
  (:use mplan
	cl
	pick-place
	graph
	utils
	set
	geom))

(in-package test-mp)

(setf *readtable* *geometry-readtable*)

(defvars r c1 c2 cs c3 c4 c5 c6 c7 c8 c9 csf cs1 cs2 g confs path paths o1 o2 s d1 d2 bounds)

(setf 
    r (make-instance '<polygon> :vertices '((-1 -1) (-1 1) (1 1) (1 -1)))
    o1 (make-instance '<polygon> :vertices '((2 3) (4 5) (4 2)))
    o2 (make-instance '<polygon> :vertices '((3 3) (6 0) (3 -3) (0 0)))
    cs (make-instance '<simple-cspace>
	   :bounds '(#(-2 -2) #(12 10))
	   :robot r :obstacles (list o1 o2))
    c1 (make-rigid-2d :v #(0 0) :theta 0)
    c2 (make-rigid-2d :v #(8 1) :theta 0)
    c3 (make-rigid-2d :v #(8 1) :theta (- (* 2 pi) 1))
    c4 (make-rigid-2d :v #(0 2) :theta 0)
    c5 (make-rigid-2d :v #(0 2) :theta -.7)
    c6 (make-rigid-2d :v #(-1 2) :theta -.7)
    c7 (make-rigid-2d :v #(0 2) :theta .7)
    c8 (make-rigid-2d :v #(3 6.4) :theta -.7)
    c9 (make-rigid-2d :v #(7 6.4) :theta -.7))


(defun approx= (x y)
  (< (abs-diff x y) *tol*))

(tests 
 "simple c-space"
 ((distance cs c1 c2) (sqrt 65) #'approx=)
 ((distance cs c1 c3) (1+ (sqrt 65)) #'approx=)
 ((distance cs c6 c7) 2.4 #'approx=)
 ((funcall (get-path cs c6 c7) .5) 
  (make-rigid-2d :v #(-.5 2) :theta 0)
  #'equal-transformations)
 ((funcall (get-path cs c4 c3) .25)
  (make-rigid-2d :v #(2 1.75) :theta -.25)
  #'equal-transformations))
				   

(tests 
 "is-free"
 ((is-free cs c1) nil)
 ((is-free cs c2) t)
 ((is-free cs c3) t)
 ((is-free cs c4) nil)
 ((is-free cs c5) t)
 ((is-free cs c6) nil)
 ((is-free cs c7) t)
 ((is-free cs c8) t)
 ((is-free cs c9) t))

(defun eq-bool (x y)
  (not (xor x y)))

(tests
 "line-segment-collides"
 ((path-collides c5 c7 cs) t #'eq-bool)
 ((path-collides c1 c2 cs) t #'eq-bool)
 ((path-collides c2 c3 cs) nil)
 ((path-collides c8 c9 cs .1) t #'eq-bool)
 ((path-collides c8 c9 cs 4) nil))




;; Cspace family
(setf csf (make-instance '<simple-cspace-family> :bounds '(#(0 0) #(10 10))
			 :robot #P((-1 0) (0 1) (1 0))
			 :objects '(#P((-1 -1) (-1 1) (1 1) (1 -1))
				    #P((0 0) (2 1) (1 -1))))
      cs1 (get-cspace csf (make-instance '<simple-cs-mode> :held nil :confs '(#R(1 1 45) #T(rigid-2d 2 2 0))))
      cs2 (get-cspace csf (make-instance '<simple-cs-mode> :held 0 :held-object-conf #R(1 0 0) :confs '(nil #R(3 1 0)))))
      

(tests
 "Cspace families"
 ((robot cs1) #P((-1 0) (0 1) (1 0)) #'same-point-set)
 ((sfirst (obstacles cs1)) #P((-.4142 1) (1 2.4142) (2.4142 1) (1 -.4142)) #'same-point-set)
 ((ssecond (obstacles cs1)) `#P((2 2) (4 3) (3 1)) #'same-point-set)
 ((length (obstacles cs1)) 6)
 ((first (union-sets (robot cs2))) #P((-1 0) (0 1) (1 0)) #'same-point-set)
 ((second (union-sets (robot cs2))) #P((0 -1) (0 1) (2 1) (2 -1)) #'same-point-set)
 ((length (obstacles cs2)) 6)
 ((ssecond (obstacles cs2)) #P((3 1) (5 2) (4 0)) #'same-point-set)
 ((sfirst (obstacles cs2)) nil)
 ((is-free cs1 #R(8.5 8.5 0)) t #'eq-bool)
 ((is-free cs2 #R(8.5 8.5 0)) nil))
  
			 


(tests
 "Simple roadmaps"
 ((progn (setf g (construct-simple-roadmap cs 25)) (mvsetq (confs path) (connect-using-roadmap cs g c3 c5))
	 (first confs)) c3)
 ((slast confs) c5)
 ((funcall path 0) c3 #'equal-transformations)
 ((funcall path 1) c5 #'equal-transformations)
 ((equal-transformations (funcall path .5) c3) nil)
 ((each 100 #'(lambda (i) (is-free cs (funcall path (/ i 100))))) t))

(tests
 "Visibility roadmaps"
 ((setf confs (first (visibility-roadmap-paths cs c3 c5 50)))
  t #'eq-bool)
 ((progn (setf path (nth-value 1 (confs->path cs confs)))
	 (funcall path 0)) 
  c3 #'equal-transformations)
 ((funcall path 1) c5 #'equal-transformations)
 ((each 100 #'(lambda (i) (is-free cs (funcall path (/ i 100))))) t))
	 
	 


(setf o1 #P((6 10) (6 7) (8 7) (8 10))
      o2 #P((3 0) (3 5) (7 5) (7 0))
      r #P((5 8) (5.5 5.5) (4.5 5.5))
      s #R(0 0 0)
      d1 #R(-3 -5 0)
      d2 #R(4 -5 0)
      bounds '((0.05 0.05) (9.95 9.95))
      g (2d-polygonal-visibility-graph r (list o1 o2) bounds)
      path (connect-using-visibility-graph g s d1))

(tests 
    "Visibility graph"
  ((first path) #(0 0))
  ((slast path) #(-3 -5))
  ((connect-using-visibility-graph g s d2) nil))
