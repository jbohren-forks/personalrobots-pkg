(defpackage test-geometry
  (:use
   cl
   utils
   lin-alg
   set
   geom))

(in-package test-geometry)

(setf *readtable* *geometry-readtable*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Intervals
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars 
    i1 i2 i3 i4 i5 i6 i7
    b1 h1 h2 h3
    p1 p2 p3 r1 r2 r22 v3
    p4 p5 p6 p7 p8 p9 p10 b s cs cs2 cs3 cs4 r)


(setf i1 (make-instance '<interval> :a '-infty :b 10 :left-open t :right-open t)
      i2 (make-instance '<interval> :a '-infty :b 10 :left-open t :right-open nil)
      i3 (make-instance '<interval> :a '-60 :b -10 :left-open t :right-open t)
      i4 (make-instance '<interval> :a '-60 :b 10 :left-open nil :right-open nil)
      i5 (make-instance '<interval> :a 10 :b 'infty :left-open t :right-open t)
      i6 (make-instance '<interval> :a 0 :b 20 :left-open t :right-open t)
      i7 (make-instance '<interval> :a 0 :b 30 :left-open nil :right-open nil))
      

(defun pattern (i)
  (list
   (member? '-infty i)
   (member? -100 i)
   (member? -60 i)
   (member? 10 i)
   (member? 100 i)
   (member? 'infty i)))

(defun int-info (i) (list (left-bound i) (right-bound i) (left-open i) (right-open i)))

(tests
 "Intervals"
 ((pattern i1) '(nil t t nil nil nil))
 ((pattern i2) '(nil t t t nil nil))
 ((pattern i3) '(nil nil nil nil nil nil))
 ((pattern i4) '(nil nil t t nil nil))
 ((pattern i5) '(nil nil nil nil t nil))
 ((int-info (intersect i1 i6)) '(0 10 t t))
 ((int-info (intersect i2 i6)) '(0 10 t nil))
 ((int-info (intersect i6 i7)) '(0 20 t t))
 ((intersects i6 i7) t)
 ((min-distance i6 i7) 0)
 ((intersects i1 i5) nil)
 ((min-distance i3 i5) 20))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Axis-Aligned-Boxes
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf b1 (make-axis-aligned-box '((2 4) (-5000 3) (4 999) (-513 2364))))

(tests 
 "Boxes"
 ((member? #(2 1 6 253) b1) t)
 ((member? #(1 1 6 253) b1) nil)
 ((member? #(4 -2351 699 4) b1) t)
 ((member? #(6 -2351 699 4) b1) nil)
 ((member? #(2 1 0 6) b1) nil)
 ((member? #(2 2345 234 263) b1) nil)
 ((member? #(2 1 6 253 4) b1) nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; half-spaces
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf h1 (make-instance '<half-space> :a #(2 -3 4) :b 5)
      h2 (make-instance '<half-space> :a #(1 2) :b 4)
      h3 (transform #T(rigid-2d 2 3 270) h2))


(tests
 "Half-spaces"
 ((member? #(1 1 2) h1) t)
 ((member? #(1 1 1) h1) nil)
 ((member? #(2 1 1) h1) t)
 ((member? #(0 2 1 1) h1) nil)
 ((nth-value 1 (member? #(0 2 1 1) h1)) ':incorrect-length)
 ((member? '(1 1 2) h1) nil)
 ((nth-value 1 (member? '(1 1 2) h1)) ':not-vector)
 ((member? #(0 2.9) h3) nil)
 ((member? #(0 3.1) h3) t)
 ((member? #(2 6.9) h3) nil)
 ((member? #(2 7.1) h3) t))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; line segments
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars l1 l2 l3 l-1 l-2 l-3 l4 l5 l6 l7 l8)

(setf l1 #L((1 1) (2 2))
      l2 #L((1 2) (3 0))
      l3 #L((1 2) (1.25 1.75))
      l-1 #L((2 2) (1 1))
      l-2 #L((3 0) (1 2))
      l4 #L((0 0) (3 3))
      l5 #L((0 0) (1.2 1.2))
      l6 #L((3 3) (4 4))
      l7 #L((3 3) (0 0))
      l8 #L((0 1) (3 4)))


(tests
 "Line segments"
 ((intersect l1 l1) l1 #'same-point-set)
 ((intersect l1 l2) #(1.5 1.5) #'close-to)
 ((intersect l1 l3) nil)
 ((intersect l-1 l2) #(1.5 1.5) #'close-to)
 ((intersect l-1 l-2) #(1.5 1.5) #'close-to)
 ((intersect l1 l4) l1 #'same-point-set)
 ((intersect l5 l1) #L((1.2 1.2) (1 1)) #'same-point-set)
 ((intersect l1 l6) nil)
 ((intersect l1 l7) l1 #'same-point-set)
 ((intersect l1 l8) nil))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; polygons
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf p1 (make-instance '<polygon> :vertices #(#(-1 2) #(2 5) #(6 -1) #(3 -4)))
      p2 (make-instance '<polygon> :vertices '(#(1 1) #(2 2) #(4 0)))
      p3 (transform (make-rigid-2d :theta (/ pi 4) :v #(-1 -1)) p2)
      r2 (sqrt 2)
      r22 (* r2 2)
      v3 (vertices p3)
      p4 #P((2 3) (6 5) (2 5) (6 3))
      p5 (make-instance '<polygon> :vertices #(#(2 3) #(4 0) #(0 0)))
      p6 (make-instance '<polygon> :vertices #(#(1.9 3.1) #(3 0) #(0 0)))
      p7 (make-instance '<polygon> :vertices #(#(0 0) #(0 10) #(10 10) #(10 0)))
      p8 (make-instance '<polygon> :vertices #(#(2 2) #(4 0) #(0 0)))
      p9 (shrink-polygon p8 .5)
      p10 (expand-polygon p4 1)
      b (bounding-box p2)
      s (bounding-sphere p5))
      

(tests
 "Polygons"
 ((consistent-polygon? p1) t)
 ((consistent-polygon? p2) t)
 ((consistent-polygon? p3) t)
 ((member? #(-1 2) p1) t)
 ((member? #(2 5) p1) t)
 ((member? #(6 -1) p1) t)
 ((member? #(3 -4) p1) t)
 ((member? #(2.1 4.9) p1) nil)
 ((member? #(2.1 4.7) p1) t)
 ((member? #(3 .5) p1) t)
 ((member? #(-10 10) p1) nil)
 ((member? `#(,(- r2 .9) -.9) p3) nil)
 ((member? `#(,(- r2 .9) -1.1) p3) t)
 ((close-to `#(,(1- r22) ,(- (1+ r22))) (sthird v3)) t)
 ((close-to `#(,(1- r2) -1) (sfirst v3)) t)
 ((close-to `#(,(1- r22) -1) (ssecond v3)) t)
 ((intersects p4 p5) t)
 ((intersects p4 p6) nil)
 ((intersects p4 p7) t)
 ((intersects p4 p8) nil)
 ((area p8) 4)
 ((area p8) 4) ;; memoized check
 ((centroid p4) #(4 4))
 ((min-distance p4 p1) 0)
 ((min-distance p4 p5) 0)
 ((min-distance p1 p2) 0)
 ((min-distance p4 p2) 1)
 ((diameter p1) (sqrt 82))
 ((diameter p2) (sqrt 10))
 ((diameter p4) (sqrt 20))
 ((member? #(2 .6) p9) t)
 ((member? #(2 .5) p9) t)
 ((member? #(2 .4) p9) nil)
 (#P((1 0) (1 2) (4 2) (4 0)) b #'same-point-set)
 (p10 #P((7 6) (7 2) (1 2) (1 6)) #'same-point-set)
 ((same-point-set p10 p7) nil)
 (s (make-instance '<sphere> :centre #(2 1) :radius (sqrt 5)) #'same-point-set)
 ((intersect p7 #L((2 6) (8 3))) #L((2 6) (8 3)) #'same-point-set)
 ((intersect p7 #L((12 3) (11 5))) nil)
 ((intersect p7 #L((2 -1) (-1 2))) #L((1 0) (0 1)) #'same-point-set)
 ((intersect p7 #L((1 -1) (-1 1))) #(0 0))
 ((intersect p7 #L((1 -1) (3 1))) #L((2 0) (3 1)) #'same-point-set))



(defvars tr tr1 tr2 points)
(setf tr1 (make-instance 'rigid-2d :v #(8 4) :center #(2 1) :theta 2)
      tr2 (make-instance 'rigid-2d :v #(5 7) :center #(3 4) :theta 6)
      tr (compose-transformations tr1 tr2)
      points (list #(5 6) #(10 -2) #(8.1 2) #(5 7) #(9 3)))

(tests "rigid-2d motions"
  ((mapcar #'(lambda (p) (transform tr p)) points)
   (mapcar #'(lambda (p) (transform tr1 (transform tr2 p))) points)
   #'(lambda (s s2) (every #'close-to s s2))))



(setf cs (transformations-into p5 p7 'rigid-2d)
      cs2 (make-instance '<rigid-2d-motions> :offsets #P((-1 1) (1 1) (1 -1) (-1 -1)))
      cs3 (make-instance '<rigid-2d-motions> :center #(2 2) :offsets (square :center #(1.8 -2) :angle 0 :radius 1))
      cs4 (make-instance '<rigid-2d-motions> :center #(2 2) :offsets (square :center #(4 4) :angle 0 :radius 1)))

      
(defun not-member (x s)
  (not (member? x s)))

(tests
 "Configuration sets"
  ((make-instance 'rigid-2d :center #(3 1) :theta 0 :v #(4 4)) cs4 #'member?)
  ((make-instance 'rigid-2d :center #(3 1) :theta 0 :v #(4 1)) cs4 #'not-member)
  ((make-instance 'rigid-2d :center #(3 1) :theta (/ pi 2) :v #(2 4)) cs4 #'member?)
  ((make-instance 'rigid-2d :center #(3 1) :theta pi :v #(2 4)) cs4 #'not-member)
  (#R(.4 1.4 0) cs #'member?)
 (#R(.4 1.4 100) cs #'not-member)
 (#R(-.5 .5 0) cs #'not-member)
 (#R(-.5 .5 80) cs #'not-member)
 (#R(5 6 0) cs #'member?)
 (#R(5 6 100) cs #'member?)
 (#R(6.1 7 0) cs #'not-member)
 (#R(6 7 100) cs #'member?)
 ((intersects cs2 cs3) t)
 ((intersects cs2 cs4) nil))


(setf r (swept-region cs p5))

(tests 
 "Swept regions"
 (#(2 2) r #'member?)
 (#(-2 -2) r #'not-member)
 (#(8 2) r #'member?)
 (#(12 2) r #'not-member)
 
 )

