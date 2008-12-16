(defpackage geometry
  (:documentation "package geometry (geom)

Generalities
- Points in R^n are represented as n-vectors

Types, constructors
-------------------
make-axis-aligned-box
r^n
<half-space>
<polygon>
<line-segment>
<sphere>
<rigid-2d-motions>
square

Operations
----------
transform 
invert
inverse-transform
intersects
min-distance
max-distance
centroid
area
equal-transformations
compose-transformations
diameter
same-point-set
is-convex
bounding-box
bounding-sphere
vdc-sequence
transformations-into
transformations-intersecting
swept-region
compose-transformation-sets
minkowski-reverse-sum

Rigid 2d-transformations
------------------------
make-rigid-2d
rigid-2d-theta
rigid-2d-v
rigid-2d-centered-v
rigid-2d
init-conf
offsets
angles

Line segments
-------------
endpoint1
endpoint2

Polygons
--------
consistent-polygon?
vertices
half-spaces
shrink-polygon
expand-polygon
sides

Algorithms
----------
2d-convex-hull

Other
-----
close-to
lp-dist
*tol*
*2pi*
*geometry-readtable*

")
  (:export
   make-axis-aligned-box
   r^n
   <half-space>
   <polygon>
   <line-segment>
   <sphere>
   <rigid-2d-motions>
   square
   
   transform 
   invert
   inverse-transform
   intersects
   min-distance
   max-distance
   area
   centroid
   equal-transformations
   compose-transformations
   diameter
   same-point-set
   is-convex
   bounding-box
   bounding-sphere
   vdc-sequence
   transformations-into
   transformations-intersecting
   swept-region
   compose-transformation-sets
   minkowski-reverse-sum
   
   make-rigid-2d
   rigid-2d-theta
   rigid-2d-v
   rigid-2d-centered-v
   rigid-2d
   init-conf
   offsets
   angles
   
   endpoint1
   endpoint2
   
   vertices
   half-spaces
   consistent-polygon?
   shrink-polygon
   expand-polygon
   sides

   close-to
   l2-dist
   l1-dist
   *tol*
   *2pi*
   *geometry-readtable*
   
   2d-convex-hull
   )
  (:use
   cl
   utils
   lin-alg
   prob
   set)
  (:nicknames geom))

(in-package geometry)

(defun r^n (n)
  "r^n N
Return the set R^n"
  (apply #'direct-product 'vector
	 (make-list n :initial-element (make-instance '<interval> :a '-infty :b 'infty :left-open t :right-open t))))



(defparameter *r2* (r^n 2))
(defparameter *r3* (r^n 3))


(defparameter *tol* .0001)

(defun close-to (x y &optional (tol *tol*))
  "close-to X Y &optional (TOL *TOL*).  Are X and Y within L-infinity distance TOL of each other."
  (<= (lp-dist x y 'infty) tol))

(defun squared-l2-dist (x y)
  (let ((s 0))
    (dotimes (i (length x) s)
      (incf s (expt (- (aref x i) (aref y i)) 2)))))

