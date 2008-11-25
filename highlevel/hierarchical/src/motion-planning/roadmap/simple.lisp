(in-package mplan)

(defstruct (simple-roadmap-node (:conc-name nil))
  configuration)

(defclass <simple-roadmap> (<adjacency-list-graph>)
  ())

(defparameter *num-connections-considered* 10)


(defun construct-simple-roadmap (cspace n)
  "construct-simple-roadmap CSPACE NUM-POINTS
The simplest roadmap construction function.  Sample points from the free subset of CSPACE and connect them (possibly incompletely) using simple paths.  Returns an undirected graph over configurations."
  (let ((s (free-space-sampler cspace))
	(g (make-instance '<simple-roadmap> 
	     :nodes (make-instance '<indexed-set> :test #'eq) 
	     :adjacency-lists #())))
    (track-connected-components g)
    (repeat n
      (add-to-roadmap cspace g (funcall s)))
    g))


(defmethod add-to-roadmap (cspace (g <simple-roadmap>) c)
  (let* ((distances
	  (sort
	   (mapset 'vector #'(lambda (n) (cons n (distance cspace c (configuration n)))) (nodes g))
	   #'< :key #'cdr))
	 (new-node (add-node g (make-simple-roadmap-node :configuration c))))
    (dotimes (i (min *num-connections-considered* (length distances)) new-node)
      (let ((n (car (aref distances i))))
	(unless (or (eq (connected-component n g) (connected-component new-node g))
		    (path-collides (configuration n) c cspace))
	  (add-edge g n new-node))))))




