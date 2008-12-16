(in-package pick-place)


(defclass <pick-place-hierarchy> (<variable-hierarchy>)
  ((regions :initarg :regions :reader regions)
   (grasp-confs :accessor grasp-confs :initarg :grasp-confs)))


(defun pick-place-hierarchy (env region-size)

  (let* ((regions (get-regions (bounds env) region-size))
	 (num-regions (length regions))
	 (grasp-confs (precompute-grasp-confs (movable-objects env) (robot env))))

    (make-variable-hierarchy
     (:env env :hierarchy-type <pick-place-hierarchy> :hierarchy-initargs (:regions regions :grasp-confs grasp-confs))

     ((transfer (o (num-movable-objects env)) (r num-regions))
      :top-level t
      :vars ((i (num-sides env o)))
      :ref ((grasp-side o i) (transfer-to-region o r)))

     ((grasp-side o i)
      :bindings ((confs (graspable-confs o i complete-set)))
      :ref ((navigate confs) (pickup o)))

     ((transfer-to-region o r)
      :ref ((putdown)))

     ((navigate confs)
      :refinable-when (and (unique-cspace complete-set) (unique-conf complete-set))
      :ref-form (motion-planner env complete-set confs)))))














;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Old
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

  
(defun simple-hierarchy (env region-size)
  "simple-hierarchy PICK-PLACE-ENV REGION-SIZE

Create a simple hierarchy with HLAs transfer, transfer-to, grasp-side, and navigate"

  (let* ((regions (get-regions (bounds env) region-size))
	 (num-regions (length regions))
	 (grasp-confs (precompute-grasp-confs (movable-objects env) (robot env))))
    
    (make-variable-hierarchy
     
     (:env env :hierarchy-type <pick-place-hierarchy> :hierarchy-initargs (:regions regions :grasp-confs grasp-confs))
   
     ((transfer (o (num-movable-objects env)) (r num-regions))
      :top-level t 
      :vars ((i (num-sides env o)))
      :ref ((grasp-side o i) (transfer-to-region o r)))
   
     ((transfer-to-region o r)
      :vars ((conf (discretize (transformations-into (object env o) (aref regions r) 'rigid-2d ':exact))))
      :ref ((navigate-putdownable o conf) (putdown)))
   
     ((grasp-side o i)
      :vars ((conf (discretize (graspable-confs grasp-confs o i))))
      :ref ((navigate-relative-to o conf) (pickup o)))

     ((navigate-putdownable o conf)
      :refinable-when (unique-configuration complete-set)
      :ref-form (putdown-paths env complete-set conf))

     ((navigate-relative-to o conf)
      :refinable-when (unique-configuration complete-set)
      :ref-form (pickup-paths env o complete-set conf)))))


(defvar *disc* 5)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; General
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *roadmap-num-confs* 20)

(defun putdown-paths (d comp-set conf)
  (let ((cspace (construct-cspace d comp-set))
	(start (check-not-null (init-conf (pss-confs comp-set))))
	(rel-conf (held-object-conf (sample-mode comp-set))))
    (let ((goal (compose-transformations conf (invert rel-conf))))
      
      (when (is-free cspace goal)
	(map 'list #'srest	
	     (visibility-roadmap-paths cspace start goal *roadmap-num-confs*))))))
	
    

(defun pickup-paths (d o comp-set rel-conf)
  (let ((cspace (construct-cspace d comp-set))
	(start (check-not-null (init-conf (pss-confs comp-set))))
	(object-conf (check-not-null (init-conf (elt (pss-objects comp-set) o)))))
    (let ((goal (compose-transformations object-conf (invert rel-conf))))
      (when (is-free cspace goal)
	(map 'list #'srest	
	     (visibility-roadmap-paths cspace start goal *roadmap-num-confs*))))))


(defun construct-complete-cspace (d cset)
  "construct-cspace ENV CSET.  Return a cspace, such that motion planning in this cspace gives an outer bound on reachability for any particular state in CSET."
  ;; For now, just sample a single state and use it.  This does not satisfy the above descirption, of course.
  (get-cspace (cspace-family d) (sample-mode cset)))

(defun sample-mode (cs)
  (let ((object-confs (map 'vector #'sample-uniformly (pss-objects cs)))
	(held-conf (awhen (pss-held cs)
		     (sample-uniformly (object-constraint cs it)))))
    
    (make-instance '<simple-cs-mode>
		   :held (pss-held cs)
		   :confs object-confs
		   :held-object-conf held-conf)))

(defun discretize (s)
  (let ((f (vdc-sequence s nil)))
    (mapset 'list #'(lambda (i) (declare (ignore i)) (funcall f)) *disc*)))

(defun get-regions (bounds region-size)
  "divide rectangle into regions of given size"
  (dbind ((xmin ymin) (xmax ymax)) bounds
    (let* ((width (- xmax xmin))
	   (height (- ymax ymin))
	   (region-width (/ width (ceiling width region-size)))
	   (region-height (/ height (ceiling height region-size)))
	   (nc (floor width region-width))
	   (nr (floor height region-height)))
      
      (mapset 'vector #'identity 
	      (ndlet ((c nc)
		      (r nr))
		(let ((x0 (+ xmin (* c region-width)))
		      (y0 (+ ymin (* r region-height))))
		  (let ((x1 (+ x0 region-width))
			(y1 (+ y0 region-height)))
		    (make-instance '<polygon>
		      :vertices (list (list x0 y0) (list x0 y1) (list x1 y1) (list x1 y0))))))))))

(defun unique-configuration (cs)
  "Return the unique robot configuration in CSET, or nil if it's not unique."
  (init-conf (pss-confs cs)))

(defun default-cspace (h)
  "Return the cspace of the initial state of the planning problem of this hierarchy."
  (cspace (init-state (planning-problem h))))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Graspable confs for side i of object o
;; Given o and i, there's a region G s.t. robot has to
;; intersect region G to pickup o at side i
;; Compute mapping from object to robot frame that
;; makes this intersection happen
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun precompute-grasp-confs (objects r)
  (mapset 'vector
	  #'(lambda (o)
	      (mapset 'vector
		      #'(lambda (s)
			  (compute-graspable-confs o s r))
		      (length (vertices o))))
	  objects))

(defun graspable-confs (gc o i)
  "Set of phi, mapping from object to robot frame, s.t. robot can pick up side I of object O."
  (if (vectorp gc)
      (aref (aref gc o) i)
    (graspable-confs (grasp-confs gc) o i)))

(defun all-graspable-confs (h o)
  (ndlet ((i (num-sides (planning-problem h) o)))
    (graspable-confs h o i)))
    

(defun compute-graspable-confs (o i r)
  (let ((gp (graspable-positions o i)))
    (filter ':implicit
	    (transformations-intersecting gp r 'rigid-2d)
	    #'(lambda (tr)
		(and (intersects (transform tr gp) r)
		     (not (intersects (transform tr o) r)))))))


(defun graspable-positions (p i &aux (vertices (vertices p)))
  "Set of points from which side I of polygon P can be grasped."
  (let ((v1 (aref-mod vertices (1- i)))
	(v2 (aref-mod vertices i))
	(v0 (aref-mod vertices (- i 2)))
	(v3 (aref-mod vertices (1+ i))))
    
    (flet ((generate-vertex (w0 w1 w2)
	     (let ((d1 (a- w0 w1))
		   (d2 (a- w0 w2)))
	       (a+ w0 (a* *pickup-tol*
			  (unit-vector (a+ (unit-vector d1) (unit-vector d2))))))))
      
      (make-instance '<polygon> :unordered-vertices (list v1 v2 (generate-vertex v1 v0 v2) (generate-vertex v2 v1 v3))))))

