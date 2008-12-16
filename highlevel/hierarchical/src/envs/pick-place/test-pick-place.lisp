(defpackage test-pick-place
  (:use
   cl
   utils
   pick-place
   mplan
   hla
   set
   hplan
   geom))

(in-package test-pick-place)

(defvar *goal-dist* 2)
(setf *readtable* *geometry-readtable*)
(defvars rob o init-conf d h p)

(setf rob #P((-1 -1) (-1 1) (1 1) (1 -1))
      o '(#P((1 1) (1 4) (4 4) (4 1)) #P((2 5) (2 6) (3 6) (3 5)))
      init-conf #R(6 2 0))

(setf mplan:*default-path-collision-resolution* 1)
(defun pri (a c s)
  (declare (ignore a))
  (if (my> s '-infty)
      (my+ s c)
      (my* c 5)))


(setf d (make-instance '<pick-place-env>
		       :init-conf init-conf :object-confs #(#R(0 0 0) #R(0 0 0))
		       :goal (make-pps-set 
			      :held nil
			      :confs t
			      :objects (list (transformations-intersecting  
					      (first o) (square :center #(8 8) :radius 2.8 :angle (/ pi 2)) 'rigid-2d)
					     t))
		       :robot rob :objects o :bounds '(#(0 0) #(10 10)))
      h (simple-hierarchy d 5)
      p (make-instance '<abstract-planning-problem>
		       :planning-problem d :hierarchy h :preprocess t
		       :sound-desc-fn #'simple-hierarchy-sound-desc
		       :complete-desc-fn #'simple-hierarchy-complete-desc
		       :admissible-heuristic 
		       #'(lambda (cs)
			   (- (max 0 (1- (min-distance (swept-region (sfirst (pss-objects cs)) (sfirst o)) #(10 10))))))))
