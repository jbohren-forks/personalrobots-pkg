(in-package pick-place)

(defstruct (pps-set (:conc-name pss-))
  "A pick-place state set consists of:
- held - this is either an integer referring to the held object, or nil.  Note we're assuming this is known.
- confs - set of possible configurations of robot.
- objects - vector containing, for the ith object, either the set of configurations of object in world if object is not held, or nil if object is held.
- constraints - a list, which for now must be a singleton.  Elements of form (o . s) where o is an object number, and s is a set of relative configurations mapping from object frame to robot frame."
  (held nil)
  (confs t)
  objects
  (constraints nil))

(defun object-constraint (s i)
  "Return the set of possible relative configurations for object I.  Error if the constraint set doesn't mention I."
  (evaluate (pss-constraints s) i))

(defmethod make-state-set ((d <pick-place-env>) (s <pick-place-state>))
  (flet ((make-singleton (conf)
	   (make-instance '<rigid-2d-motions> :conf conf)))
    (let ((i (held s)))
      (make-pps-set
       :held i
       :objects (mapset 'vector #'(lambda (c) (when c (make-singleton c))) (confs s))
       :confs (make-singleton (conf s))
       :constraints (when i (list (cons i (make-singleton (held-object-conf s)))))))))

(defun empty-state-set (d)
  (make-pps-set :confs nil :objects (make-array (length (movable-objects d)) :initial-element nil)))

(defmethod intersects ((s pps-set) (s2 pps-set))
  ;; TODO this doesn't check the constraints
  (assert (member t (list (pss-confs s) (pss-confs s2))) nil
	  "Currently intersection only implemented when at least one of the conf sets allows all rigid motions")
  (and (eql (pss-held s) (pss-held s2))
       (every #'intersects (pss-objects s) (pss-objects s2))))

(defmethod is-empty ((s pps-set))
  ;; Can sometimes incorrectly return false because it doesn't check the constraints
  (or (is-empty (pss-confs s))
      (do-elements (obj-confs (pss-objects s) nil i)
	(when (and (is-empty obj-confs) (not (eql i (pss-held s))))
	  (return t)))))

(defmethod member? ((s <pick-place-state>) (ps pps-set))
  (let ((object-confs (confs s))
	(robot-conf (conf s)))
    (and (eql (pss-held ps) (held s))
	 (member? robot-conf (pss-confs ps))
	 (every #'member? object-confs (pss-objects ps))
	 (every #'(lambda (c)
		    (dsbind (i relative-confs) c
		      (member? (compose-transformations (invert robot-conf) (elt object-confs i))
			       relative-confs)))
		(pss-constraints ps)))))

(defun unique-state (s d)
  "If conf set is a singleton, return the unique element.  Otherwise, return nil."
  (when s
    (let ((conf (init-conf (pss-confs s)))
	  (object-confs (map 'vector #'(lambda (cs) (when cs (init-conf cs))) (pss-objects s)))
	  (i (pss-held s)))
      (when i
	(setf (aref object-confs i) (init-conf (object-constraint s i))))
      (when (and conf (every #'identity object-confs))
	(let ((m (make-instance '<simple-cs-mode> :held i :confs object-confs :held-object-conf (when i (aref object-confs i)))))
	  (make-instance '<pick-place-state> :domain d :cspace-family (cspace-family d)
			 :mode m :conf conf))))))


