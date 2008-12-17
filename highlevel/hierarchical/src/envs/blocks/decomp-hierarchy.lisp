(in-package :blocks)

(defclass <blocks-hierarchy> ()
  ((domain :reader planning-domain :initarg :domain)))

(make-hlas (h <blocks-hierarchy>) (init-opt-set) (left up down right pickupL pickupR stackL stackR turnL turnR finish)
  
  (act () :or 
       (disjoint-union 
	'(finish)
	(ndlet-fail ((b (possibly-clear-blocks init-opt-set))
		     (s (possibly-clear-surfaces init-opt-set)))
	  (if (eq b s)
	    'fail
	    `(move ,b ,s)))))

  (move (b s) :sequence `((get ,b) (put ,b ,s)))

  (get (b) :sequence `((nav-get ,b) (pickup ,b)))
  
  (put (b s) :sequence `((nav-put ,s) (put-down ,b ,s)))

  (nav-get (b) :or 
	   (dsbind (c r) (unique-block-position b init-opt-set)
	     (ndlet-fail ((c2 (list (1+ c) (1- c))))
	       (if (between c2 0 (num-cols (planning-domain h)))
		 `(nav ,c2 ,r)
		 'fail))))

  (nav-put (b) :or
	   (dsbind (c r) (unique-block-position b init-opt-set)
	     (let ((r2 (1+ r))
		   (d (planning-domain h)))
	       
	       (when (< r2 (num-rows d))
		 (ndlet-fail ((c2 (list (1+ c) (1- c))))
		   (if (between c2 0 (num-cols d))
		     `(nav ,c2 ,r2)
		     'fail))))))

  (nav (c r) :or `((navigate ,c ,r) (right-turn-and-navigate ,c ,r) (left-turn-and-navigate ,c ,r)))

  (navigate (c r) :or
	    (mapcan 
	     #'(lambda (pos)
		 (if (equal pos (list c r))
		   (list nil)
		   `((left-and-navigate ,c ,r) (right-and-navigate ,c ,r) (up-and-navigate ,c ,r) (down-and-navigate ,c ,r))))
	     (possible-gripper-positions init-opt-set)))

  (left-and-navigate (c r) :sequence (navigate-refinements init-opt-set 'left c r))
  (right-and-navigate (c r) :sequence (navigate-refinements init-opt-set 'right c r))
  (up-and-navigate (c r) :sequence (navigate-refinements init-opt-set 'up c r))
  (down-and-navigate (c r) :sequence (navigate-refinements init-opt-set 'down c r))
  (left-turn-and-navigate (c r) :sequence (turn-and-navigate-refinements init-opt-set 'turnL c r))
  (right-turn-and-navigate (c r) :sequence (turn-and-navigate-refinements init-opt-set 'turnR c r)))

(defun turn-and-navigate-refinements (s dir c r)
  (let ((rmax (1- (num-rows (pss-domain s)))))
    (mapcan #'(lambda (pos)
		(let ((c0 (first pos)))
		  `(((navigate ,c0 ,rmax) (,dir ,c0 ,rmax) (navigate ,c ,r)))))
	    (possible-gripper-positions s))))

  

(defparameter *dirs* '((left -1 0) (right 1 0) (up 0 1) (down 0 -1)))

(defun navigate-refinements (init-opt-set dir c r)
  (dsbind (dc dr) (mapping:evaluate *dirs* dir)
    (mapcan #'(lambda (pos)
		(dsbind (c0 r0) pos
		  (when (possibly-free (+ c0 dc) (+ r0 dr) init-opt-set)
		    `(((,dir ,c0 ,r0 ,(if (zerop dc) (+ r0 dr) (+ c0 dc))) (navigate ,c ,r))))))
	    (possible-gripper-positions init-opt-set))))


#|
  (navigate (r c) :or (if (equal (get-unique-pos init-opt-set) (list r c)) (list nil))))

	    
	    


      (nav ;; either navigate, right-turn-and-navigate, or left-turn-and-navigate
	 )
      (navigate ;; if at dest, no-op.  else, left-and-navigate, or ...
	 )
      (left-and-navigate ;; left followed by navigate)
	 )
      (right-and-navigate)
      (up-and-navigate)
      (down-and-navigate)
      (left-turn-and-navigate ;; navigate to top, turnL, navigate
	 )
      (right-turn-and-navigate ;; navigate to top, turnR, navigate
	 )
      (pickup ;; pickupL or pickupR
	 )
      (putdown ;; putdownL or putdownR
	 ))))

      
|#