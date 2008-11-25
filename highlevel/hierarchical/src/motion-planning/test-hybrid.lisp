(defpackage test-hybrid
  (:use
   mplan
   cl
   hplan
   geometry
   lin-alg
   set
   utils
   hla
   prob)
  )

(in-package test-hybrid)

(setf *readtable* *mplan-readtable*)
(defparameter *pickup-dist* .1)
(defparameter *goal-dist-threshold* .1)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Mode
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (mode (:include simple-cs-mode) (:constructor create-mode))
  (gripper-dists nil)
  (corner-dist nil)
  (last-action nil))

(defun make-mode (held confs &optional (last-action nil))
  (create-mode :held held :confs confs :last-action last-action))

(defun clone-mode (m)
  (let ((m2 (copy-mode m)))
    (setf (mode-gripper-dists m2) (copy-seq (mode-gripper-dists m))
	  (mode-confs m2) (copy-seq (mode-confs m)))
    (return-from clone-mode m2)))

(defun update-mode (conf m)
  (let ((m2 (clone-mode m))
	(r (transform conf robot))
	(cs (get-cspace csf m))
	(held (mode-held m)))
    (setf (mode-gripper-dists m2)
      (map 'vector #'(lambda (conf obj) (min-distance r (transform conf obj))) (mode-confs m) o))
    (typecase (mode-last-action m)
      ((eql pickup) (setf (elt (mode-confs m2) held)
		      (transform-to-robot-coords conf (elt (mode-confs m) held))))
      (cons (let ((held (cdr (mode-last-action m))))
	      (setf (elt (mode-confs m2) held)
		(transform-to-world-coords conf (elt (mode-confs m) held))))))
    (setf (mode-last-action m2) nil)
    (setf (mode-corner-dist m2)
      (aif (sfirst (obstacles cs))
	  (lp-dist #(0 0) (sfirst (vertices it)) 2)
	'infty))
    (return-from update-mode m2)))


(defun transform-to-robot-coords (robot-conf conf)
  (compose-transformations (invert robot-conf) conf))

(defun transform-to-world-coords (robot-conf conf)
  (compose-transformations robot-conf conf))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Symbolic domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;




(defun putdown-pre (m)
  (mode-held m))

(defun pickup-pre (m o)
  (and (not (mode-held m))
       (my< (aref (mode-gripper-dists m) o) *pickup-dist*)))

(defun pickup-trans (m o)
  (let ((m2 (clone-mode m)))
    (setf (mode-held m2) o
	  (mode-last-action m2) 'pickup)
    (return-from pickup-trans m2)))

(defun putdown-trans (m)
  (let ((m2 (clone-mode m)))
    (setf (mode-last-action m2) (cons 'putdown (mode-held m2))
	  (mode-held m2) nil)
    (return-from putdown-trans m2)))

(defun at-goal (m)
  (let ((d (mode-corner-dist m)))
    (and d (my< d *goal-dist-threshold*))))

(defun pickup-desc (o)
  #'(lambda (s)
      (if (pickup-pre s o)
	  (pickup-trans s o)
	s)))

(setf putdown-desc
  #'(lambda (s)
      (if (putdown-pre s)
	  (putdown-trans s)
	s)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Hybrid problem
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(setf robot #P((-1 -1) (-1 1) (1 1) (1 -1))
      o '(#P((1 1) (1 4) (4 4) (4 1)) #P((2 5) (2 6) (3 6) (3 5)))
      m (make-mode nil '(#R(0 0 0) #R(0 0 0)) nil)
      init-conf #R(6 2 0)
      csf (make-instance '<simple-cspace-family> :bounds '(#(0 0) #(10 10))
			 :robot robot :objects o)
      sd (make-instance '<planning-problem>
	  :init-state (update-mode init-conf m)
	  :goal (objects-satisfying #'at-goal)
	  :avail-actions-fn (constantly '((putdown) (pickup 0) (pickup 1)))
	  :action-descriptions `(((putdown) . ,putdown-desc) 
				 ((pickup 0) . ,(pickup-desc 0)) 
				 ((pickup 1) . ,(pickup-desc 1))))
      d (make-instance '<hybrid-planning-problem>
	  :init-conf init-conf :mode-update-fn #'update-mode
	  :symbolic-domain sd :cspace-family csf))
