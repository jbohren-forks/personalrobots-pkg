(defpackage test-dcel
  (:use
   cl
   geometry
   utils
   set
   dcel))

(in-package test-dcel)

(defvars p1 p2 p3 p4 p5 p6 p7 p8
	 v1 v2 v3 v4 v5 v6 v7 v8 v9
	 e12 e23 e32 e31 e24 e45 e53
	 e67 e78 e86 e69 e97
	 f11 f12 f21 f22
	 d1 d2)

(setf p1 #(5 10)
      p2 #(3 5)
      p3 #(6 6)
      p4 #(0 1)
      p5 #(3 2)
      p6 #(4.5 4)
      p7 #(8 3)
      p8 #(6 0)
      v1 (make-vertex :coords p1)
      v2 (make-vertex :coords p2)
      v3 (make-vertex :coords p3)
      v4 (make-vertex :coords p4)
      v5 (make-vertex :coords p5)
      e12 (make-edge-pair v1 v2)
      e23 (make-edge-pair v2 v3)
      e31 (make-edge-pair v3 v1)
      e32 (twin e23)
      e24 (make-edge-pair v2 v4)
      e45 (make-edge-pair v4 v5)
      e53 (make-edge-pair v5 v3)
      v6 (make-vertex :coords p6)
      v7 (make-vertex :coords p7)
      v8 (make-vertex :coords p1)
      v9 (make-vertex :coords p8)
      e67 (make-edge-pair v6 v7)
      e78 (make-edge-pair v7 v8)
      e86 (make-edge-pair v8 v6)
      e69 (make-edge-pair v6 v9)
      e97 (make-edge-pair v9 v7)
      f11 (make-face-bounded-by e12 e23 e31)
      f12 (make-face-bounded-by e24 e45 e53 e32)
      f13 (make-face :inner-edges (list (twin e12)))
      f21 (make-face-bounded-by e67 e78 e86)
      f22 (make-face-bounded-by e69 e97 (twin e67))
      f23 (make-face :inner-edges (list (twin e69)))
      d1 (make-dcel-from-faces (list f11 f12 f13))
      d2 (make-dcel-from-faces (list f21 f22 f23)))

(apply #'attach (mapcar #'twin (list e12 e31 e53 e45 e24 e12)))
(apply #'attach (mapcar #'twin (list e86 e78 e97 e69 e86)))




(tests "DCELs"
  ((to-list (boundary-iterator f11)) (list e12 e23 e31) #'set-eq)
  ((to-list (half-edges-from v3)) (list e32 e31 (twin e53)) #'set-eq)
  ((is-consistent-dcel d1) t)
  ((is-consistent-dcel d2) t))