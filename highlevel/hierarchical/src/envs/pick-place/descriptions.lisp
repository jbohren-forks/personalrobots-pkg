(in-package pick-place)

(defun simple-hierarchy-sound-desc (a h &aux (d (planning-problem h)))
  (if (eq 'primitive (action-type a h))

      (make-instance 
       '<simple-description>
       :succ-state-fn #'(lambda (ss)
			  (awhen (unique-state ss d)
			    (make-state-set d (succ-state it (primitive-action-description d a)))))
       :reward-fn #'(lambda (ss ss2)
		      (declare (ignore ss2))
		      (aif (unique-state ss d)
			   (reward d it a)
			   0)))


      (make-instance '<simple-description>
		     :succ-state-fn (constantly nil)
		     :reward-fn (constantly 0))))



  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Complete descriptions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun transfer-desc (h o r &aux (d (planning-problem h)) (g (aref (regions h) r)))
  #'(lambda (s)
      
      ;; Must be holding nothing
      (when (and s (null (pss-held s)))
	
	;; Substitute the Oth object with the appropriate set
	;; Also, don't directly specify robot resulting conf, but add a constraint that 
	;; R must be in position where O is graspable (as grasp and ungrasp are inverses)
	(let ((object-confs (copy-seq (pss-objects s))))
	  (setf (elt object-confs o) 
	    (intersect (transformations-into (object d o)  g 'rigid-2d ':outer-bound)
		       (object-free-space-outer-bound d o s)))
	  (make-pps-set :held nil :confs t :objects object-confs
			:constraints (list (cons o (all-graspable-confs h o))))))))

(defun transfer-reward (h o r &aux (d (planning-problem h)) (g (aref (regions h) r)))
  #'(lambda (s s2)
      (declare (ignore s2))
      (let ((object (swept-region (aref (pss-objects s) o) (object d o)))
	    (robot (swept-region (pss-confs s) (robot d))))
	(my- 
	 (* 2 (pickup-tol d))
	 (my+ (min-distance object g) 
	      (min-distance object robot)
	      (* 2 (grasp-cost d)))))))


(defun grasp-side-desc (h o i)
  #'(lambda (s)

      ;; Must be holding nothing
      (when (null (pss-held s))
	
	;; Add a constraint between robot and object o conf
	(make-pps-set 
	 :held o :objects (pss-objects s) :confs t
	 :constraints (list (cons o (graspable-confs h o i)))))))


(defun grasp-side-reward (h o i &aux (cs (default-cspace h)) (c (grasp-cost (planning-problem h))))
  (declare (ignore o i))
  #'(lambda (s s2)
      (- (+ c (min-conf-distance cs (pss-confs s) (pss-confs s2))))))


(defun transfer-to-region-desc (h o r &aux (d (planning-problem h)) (g (aref (regions h) r)))
  ;; TODO need to add constraint as well
  #'(lambda (s)
      (when (eql (pss-held s) o)
	(let ((object-confs (copy-seq (pss-objects s))))
	  (setf (elt object-confs o) (transformations-into (object d o) g 'rigid-2d ':outer-bound))
	  (make-pps-set
	   :held nil :confs t :objects object-confs)))))

(defun transfer-to-region-reward (h o r &aux (cs (default-cspace h)) (c (grasp-cost (planning-problem h))))
  (declare (ignore o r))
  #'(lambda (s s2)
      (- (+ c (min-conf-distance cs (pss-confs s) (pss-confs s2))))))


(defun navigate-complete-desc (h confs)
  #'(lambda (s)
      
      ;; Just the conf changes.  Also, constraints are preserved iff the object is held.
      (let ((s2 (copy-pps-set s)))
	(setf (pss-confs s2) confs
	      
	      (pss-constraints s2) 
	      (awhen (pss-held s)
		(cons it (object-constraint s it)))))))

(defun navigate-complete-reward (h confs)
  (declare (ignore confs))
  #'(lambda (s s2)
      (min-conf-distance (default-cspace h) (pss-confs s) (pss-confs s2))))  

(defun navigate-relative-to-desc (h o rel-conf)
  (declare (ignore h))
  (let ((inv-rel-conf (invert rel-conf)))
    #'(lambda (cs)
	(let ((obj-conf (unique-conf (elt (pss-objects cs) o))))
	  (make-pps-set 
	   :held (pss-held cs) :objects (pss-objects cs)
	   :confs (singleton (compose-transformations obj-conf inv-rel-conf))
	   :constraints (list (cons o (singleton rel-conf))))))))

(defun navigate-relative-to-reward (h o r &aux (cs (default-cspace h)))
  (declare (ignore o r))
  #'(lambda (s s2) (- (min-conf-distance cs (pss-confs s) (pss-confs s2)))))
  
(defun navigate-putdownable-desc (h o conf)
  (declare (ignore h))
  #'(lambda (cs)
      
      (make-pps-set
       :held (pss-held cs) :objects (pss-objects cs)
       :confs (make-instance 
	       '<rigid-2d-motions>
	       :conf (compose-transformations conf (invert (unique-conf (object-constraint cs o)))))
       :constraints (pss-constraints cs))))

(defun navigate-putdownable-reward (h o conf &aux (cs (default-cspace h)))
  (declare (ignore conf o r))
  #'(lambda (s s2)
      (- (min-conf-distance cs (pss-confs s) (pss-confs s2)))))
				       


(defmacro function-plist (suffix &rest fns)
  `(p2alist ,@(loop
		 for f in fns
		 collect `',f
		  collect `(function ,(intern-compound-symbol f suffix)))))

(defun unique-conf (s)
  (check-not-null (init-conf s)))

(defun singleton (conf)
  (make-instance '<rigid-2d-motions> :conf conf))

(defun pickup-desc (h i)
  (declare (ignore h))
  #'(lambda (cs)
      (let ((relative-conf (compose-transformations (invert (unique-conf (pss-confs cs)))
						    (unique-conf (elt (pss-objects cs) i)))))
	
	(make-pps-set :held i :objects (pss-objects cs)
		      :confs (pss-confs cs) 
		      :constraints (list (cons i (singleton relative-conf)))))))
							 

(defun putdown-desc (h)
  (declare (ignore h))
  #'(lambda (cs)
      (let ((o (copy-seq (pss-objects cs)))
	    (i (pss-held cs)))
	(setf (elt o i) (make-instance '<rigid-2d-motions>
				       :conf (compose-transformations
					      (unique-conf (pss-confs cs))
					      (unique-conf (object-constraint cs i)))
			 ))
	(make-pps-set :held nil :objects o :confs (pss-confs cs) :constraints nil))))

(defun putdown-reward (h)
  (let ((r (- (grasp-cost (planning-problem h)))))
    (constantly r)))

(defun move-desc (h dest)
  (declare (ignore h))
  #'(lambda (cs)
      (let ((held (pss-held cs)))
	(make-pps-set :held held :objects (pss-objects cs)
		      :confs (singleton dest) 
		      :constraints (filter ':list (pss-constraints cs)
					   #'(lambda (x) (eql (first x) held)))))))

(defun move-reward (h dest &aux (cspace (default-cspace h)))
  #'(lambda (cs cs2)
      (declare (ignore cs2))
      (- (min-conf-distance cspace (singleton dest) (pss-confs cs)))))

(defun pickup-reward (h i)
  (declare (ignore i))
  (let ((r (- (grasp-cost (planning-problem h)))))
    (constantly r)))
      



(defparameter *complete-descs*
    (function-plist -desc transfer grasp-side transfer-to-region navigate-relative-to 
		    putdown pickup move navigate-putdownable))
(defparameter *complete-rewards*
    (function-plist -reward transfer grasp-side transfer-to-region navigate-relative-to 
		    putdown pickup move navigate-putdownable))

(defun simple-hierarchy-complete-desc (a h)
  (when (typep a 'rigid-2d)
    (setf a (list 'move a)))
  (dsbind (name &rest args) a
    (make-instance '<simple-description>
      :succ-state-fn (apply (evaluate *complete-descs* name) h args)
      :reward-fn (apply (evaluate *complete-rewards* name) h args))))