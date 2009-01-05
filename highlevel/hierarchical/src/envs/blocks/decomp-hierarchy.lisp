(in-package :blocks)

(defclass <blocks-hierarchy> ()
  ((domain :reader planning-domain :initarg :domain)))



(make-hlas (h <blocks-hierarchy>) (init-opt-set) (left up down right pickupL pickupR stackL stackR turnL turnR finish)

  ;; Act: either do nothing or move a block and act again
  (act () :or 
       (disjoint-union 
	'(((finish)))
	(ndlet-fail ((b (possibly-clear-blocks init-opt-set))
		     (s (possibly-clear-surfaces init-opt-set)))
	  (if (eq b s)
	      'fail
	      (dsbind (xb yb) (unique-block-position b init-opt-set)
		(dsbind (xs ys) (unique-block-position s init-opt-set)
		  (let ((a (unique-block-below b init-opt-set)))
		    `((move-then-act ,b ,a ,s ,xb ,yb ,xs ,ys)))))))))

  (move-then-act (b a s xb yb xs ys) :sequence `(((move-block ,b ,a ,s ,xb ,yb ,xs ,ys ,(1+ ys)) (act))))

  ;; Same as in the old hierarchy
  (move-block (b a s xb yb xs ys yt) :sequence `(((navigate-beside ,xb ,yb) (pickup ,b ,a ,xb ,yb)
						  (navigate-beside ,xs ,yt) (stack ,b ,s ,xs ,yt))))

  ;; Either move to the left of or right of target
  (navigate-beside (x y)
		   :or
		   (mapcan
		    #'(lambda (pos)
			(dsbind (xg yg) pos
			  (loop
			     for d in '(-1 1)
			     when (possibly-free (+ x d) y init-opt-set)
			     collect `((navigate ,xg ,yg ,(+ x d) ,y)))))
		    (possible-gripper-positions init-opt-set)))

  ;; Navigate: either go straight or turn first
  (navigate (xg yg x y)
	    :or
	    (let ((dirs (possibly-facing init-opt-set)))
	      (append 
	       `(((nav ,xg ,yg ,x ,y)))
	       (when (member 'left dirs) `((navigate-with-right-turn ,xg ,yg ,x ,y)))
	       (when (member 'right dirs) `((navigate-with-left-turn ,xg ,yg ,x ,y))))))

  ;; Turn and navigate
  (navigate-with-right-turn (xg yg x y)
		      :sequence 
		      (let ((ymax (1- (num-rows (planning-domain h)))))
			`(((nav ,xg ,yg ,xg ,ymax) (turnR ,xg) (nav ,xg ,ymax ,x ,y)))))

  (navigate-with-left-turn (xg yg x y)
		     :sequence 
		     (let ((ymax (1- (num-rows (planning-domain h)))))
		       `(((nav ,xg ,yg ,xg ,ymax) (turnL ,xg) (nav ,xg ,ymax ,x ,y)))))


  ;; Navigation
  (nav (xg yg x y)
       :or
       (if (and (= xg x) (= yg y))
	   '(())
	   (append
	    (when (> xg 0) `(((left-and-nav ,xg ,yg ,x ,y))))
	    (when (< xg (1- (num-cols (planning-domain h)))) `(((right-and-nav ,xg ,yg ,x ,y))))
	    (when (> yg 1) `(((down-and-nav ,xg ,yg ,x ,y))))
	    (when (< yg (1- (num-rows (planning-domain h)))) `(((up-and-nav ,xg ,y ,x ,y)))))))

  (left-and-nav (xg yg x y)
		:sequence
		`(((left ,xg ,yg ,(1- xg)) (nav ,(1- xg) ,yg ,x ,y))))
		  
  (right-and-nav (xg yg x y)
		 :sequence
		 `(((right ,xg ,yg ,(1+ xg)) (nav ,(1+ xg) ,yg ,x ,y))))
		  
  (up-and-nav (xg yg x y)
	      :sequence
	      `(((up ,xg ,yg ,(1+ yg)) (nav ,xg ,(1+ yg) ,x ,y))))

  (down-and-nav (xg yg x y)
		:sequence
		`(((down ,xg ,yg ,(1- yg)) (nav ,xg ,(1- yg) ,x ,y))))

  
  ;; Pickup/stack
  (pickup (b a xb yb) :or
	  (let ((positions (possible-gripper-positions init-opt-set))
		(facing (possibly-facing init-opt-set)))
	    (mapcan 
	     #'(lambda (pos)
		 (dsbind (x0 y0) pos
		   (when (= y0 yb)
		     (cond 
		       ((= x0 (1- xb)) (when (member 'right facing) `(((pickupR ,b ,a ,xb ,y0 ,x0)))))
		       ((= x0 (1+ xb)) (when (member 'left facing) `(((pickupL ,b ,a ,xb ,y0 ,x0)))))))))
	     positions)))


  (stack (b s xs yt) :or
	 (let ((positions (possible-gripper-positions init-opt-set))
	       (facing (possibly-facing init-opt-set)))
	   (mapcan
	    #'(lambda (pos)
		(dsbind (x0 y0) pos
		  (when (= y0 yt)
		    (cond
		      ((= x0 (1- xs)) (when (member 'right facing) `(((stackR ,b ,s ,xs ,(1- yt) ,x0 ,y0)))))
		      ((= x0 (1+ xs)) (when (member 'left facing) `(((stackL ,b ,s ,xs ,(1- yt) ,x0 ,y0)))))))))
	    positions))))
		   
	  

