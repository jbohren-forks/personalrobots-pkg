(in-package geometry)

(defmethod closest-points ((s1 list) s2)
  (assert s1 nil "Can't find closest point to empty set.")
  (let ((best 'infty) p1 p2)
    (dolist (q s1 (values p1 p2 best))
      (mvbind (q1 q2 dist) (closest-points q s2)
	      (when (my< dist best)
		(setf best dist 
		      p1 q1
		      p2 q2))))))

(defmethod closest-points (s1 (s2 list))
  (mvbind (q2 q1 dist) (closest-points s2 s1)
	  (values q1 q2 dist)))

(defmethod is-convex ((s list))
  (not (length-exceeds s 1)))

(def-symmetric-method max-distance ((l list) s)
  (reduce #'mymax l :key #'(lambda (p) (max-distance p s))))

(defmethod centroid ((l list))
  (assert l)
  (a/ (apply #'a+ l) (length l)))
	
