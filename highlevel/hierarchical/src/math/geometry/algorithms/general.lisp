(in-package geometry)

(defun 2d-convex-hull (points)
  "convex-hull POINTS

POINTS is a finite set of points in R^2.

Returns the convex hull, represented as a <polygon>."
  

  (let ((points (sort (mapset 'vector #'identity points) 
		      #'(lambda (p1 p2)
			  (dbind (x1 y1) p1
			    (dbind (x2 y2) p2
			      (or (< x1 x2)
				  (and (= x1 x2) (< y1 y2)))))))))
    (make-instance 
     '<polygon>
     :vertices
     (nreverse (concatenate 'list 
			    (convex-hull-helper points)
			    (butlast (cdr (convex-hull-helper (reverse points)))))))))


(defun convex-hull-helper (points)
  (let ((hull (list (aref points 1) (aref points 0)))
	(n (length points))
	(m 2))
    (flet ((add-to-hull (p)
	     (push p hull)
	     (incf m))
	   (delete-second ()
	     (setf (cdr hull) (cddr hull))
	     (decf m)))
      
      (for-loop (i 2 n)
	(add-to-hull (aref points i))
	(while (and (> m 2)
		    (or (close-to (third hull) (second hull))
			(close-to (first hull) (second hull))
			(not (right-turn (third hull) (second hull) (first hull)))))
	  (delete-second)))
      hull)))

(defun right-turn (p1 p2 p3)
  (> (angle (a- p3 p2) (a- p2 p1)) 0))


  
	      
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Minkowski sum of polygons
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
    
    
(defmethod minkowski-reverse-sum ((p1 <polygon>) (p2 <polygon>))
  (2d-convex-hull (ndlet ((v1 (vertices p1))
			  (v2 (vertices p2)))
		    (a- v1 v2))))
    