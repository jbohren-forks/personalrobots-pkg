(in-package motion-planning)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Special vars
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *default-path-collision-resolution* .1)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Generic operations to be implemented by cspaces
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric is-free (cspace conf)
  (:documentation "is-free CONFIGURATION-SPACE CONFIGURATION.  Return t iff this configuration does not involve any collisions."))

(defgeneric get-path (cspace c1 c2)
  (:documentation "get-path CSPACE CONF1 CONF2.  Return a path (not necessarily free) between the two configurations.  The path is a function of one real argument in [0,1] where path(0) = CONF1 and path(1) = CONF2.  For efficiency, the returned function may reuse the same object when called multiple times."))

(defgeneric distance (cspace c1 c2)
  (:documentation "distance CSPACE CONF1 CONF2.  Distance (according to some metric depending on the configuration space) between the configurations."))

(defgeneric min-conf-distance (cspace cs1 cs2)
  (:documentation "min-conf-distance CSPACE CONF-SET1 CONF-SET2.  Minimum distance between configuration sets.")
  (:method (cspace (cs1 (eql t)) cs2) (declare (ignore cspace cs2)) 0.0)
  (:method (cspace cs1 (cs2 (eql t))) (declare (ignore cspace cs1)) 0.0)
  (:method (cspace (cs1 null) cs2) (declare (ignore cspace cs2)) 'infty)
  (:method (cspace cs1 (cs2 null)) (declare (ignore cspace cs1)) 'infty)
  (:method (cspace (cs1 (eql t)) (cs2 null)) (declare (ignore cspace)) 'infty)
  (:method (cspace (cs1 null) (cs2 (eql t))) (declare (ignore cspace)) 'infty))



(defgeneric sampler (cspace)
  (:documentation "sampler CSPACE.  Returns a function that generates eventually dense samples from the cspace."))

(defgeneric all-confs (cspace)
  (:documentation "Returns the set of legal (though not necessarily free) configurations in cspace.  Defaults to 'universal-set.")
  (:method (cspace) (declare (ignore cspace)) t))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Other operations built on the generic ones
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun free-space-sampler (cspace)
  "free-space-sampler CSPACE.  Returns a function that generates eventually dense samples from the free space of CSPACE."
  (let ((f (sampler cspace)))
    #'(lambda ()
	(loop
	  (let ((x (funcall f)))
	    (when (is-free cspace x)
	      (return x)))))))

(defun edge-biased-sampler (cspace &key (resolution *default-path-collision-resolution*))
  "edge-biased-sampler CSPACE &key (RESOLUTION *default-path-collision-resolution*).  Return a new sampler which modifies the base sampler of cspace to be more biased towards edges, by sampling a pair of configurations from SAMPLER, and calling closest-conf."
  (let ((s (free-space-sampler cspace)))
    #'(lambda () 
	(closest-conf (funcall s) (funcall s) cspace resolution))))


(defun path-collides (c1 c2 cspace &optional (resolution *default-path-collision-resolution*))
  "path-collides C1 C2 CSPACE &optional (RESOLUTION *default-path-collision-resolution*)

Check that the path between C1 and C2 in cspace is collision-free.  Doubling RESOLUTION halves the number of points considered.  T is returned only if a collision is found, but may fail to find a collision if the resolution is not small enough.  The resolution is in units of the distance metric of CSPACE (based on conf-dist)."

  (let ((generator (vdc-generator))
	(path (get-path cspace c1 c2))
	(dist (distance cspace c1 c2)))
    (and (> dist 0)
	 (let* ((path-resolution (/ dist resolution))
		(num-points (expt 2 (max 2 (ceiling (log path-resolution 2))))))
	   
	   (dotimes (i num-points nil)
	     (let ((conf (funcall path (funcall generator))))
	       (unless (is-free cspace conf)
		 (return conf))))))))
  

(defun directly-reachable-confs (cspace conf)
  "directly-reachable-confs CSPACE CONF.  Implicit representation of the set of configurations that can be connected to CONF by the local planner. Behavior also depends on *default-path-collision-resolution*."
  (filter ':implicit (all-confs cspace) #'(lambda (c) (not (path-collides conf c cspace)))))


(defun closest-conf (c1 c2 cspace &optional (resolution *default-path-collision-resolution*))
  "closest-conf C1 C2 &optional (RESOLUTION *default-path-collision-resolution*)

Consider the path from C1 to C2.  The path either reaches C2 or collides with an obstacle.  Imagine stopping the path right before a collision.  This function returns such a configuration (upto the given resolution)."
  (let ((n (ceiling 1 resolution))
	(path (get-path cspace c1 c2)))
    (flet ((get-point (i)
	     (funcall path (/ i n))))
      
      (assert (is-free cspace c1))
      (dotimes (i n c2)
	(unless (is-free cspace (get-point (1+ i)))
	  (return (get-point i)))))))


