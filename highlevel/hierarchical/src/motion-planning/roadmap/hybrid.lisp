(in-package hplan)


(defun construct-hybrid-roadmap (d num-confs init-state &key (mode-growth-function #'sqrt)
							     (sampling-fn #'free-space-sampler))
  
  
  ;; Initialize
  (let ((i 1)
	(modes (list (make-mode-entry init-state sampling-fn)))
	(roadmap (make-instance '<adjacency-list-graph>
		   :nodes (make-instance '<indexed-set> :test #'eq)
		   :adjacency-lists #()))
	(mode-queue (make-queue)))
    (track-connected-components roadmap)
    (add-node roadmap init-state)
    
    (flet ((add-new-modes (new-state)
	     (do-elements (a (avail-symbolic-actions d new-state))
	       (let ((s (action-result d new-state a)))
		 (debug-prompt 0 "Enqueuing new mode state ~a" 
			       (display-state d s))
		 (when (member? s (goal d))
		   (print "Goal")
		   (break)))
	       (enqueue (cons (action-result d new-state a) new-state) mode-queue))))
	
      ;; Main loop
      (until (>= i num-confs)
      
	(if (and (<= (length modes) (funcall mode-growth-function i)) (not (queue-empty mode-queue)))
	      
	    ;; If we don't have enough modes, and there exist modes on the queue, get the next one
	    (dsbind (new-mode-state . prev-state) (dequeue mode-queue)
	    
	      (debug-prompt 0 "~%~%Considering new mode state ~a" (display-state d new-mode-state))

	      ;; Unless we already have states with this mode, add it
	      (unless (find (mode new-mode-state) modes :test #'same-mode :key #'me-mode)
	      
		(debug-print 0 "Adding to roadmap")
		(push (make-mode-entry new-mode-state sampling-fn) modes)
		(add-node roadmap new-mode-state)
		(add-new-modes new-mode-state)
		(when prev-state
		  (add-edge roadmap prev-state new-mode-state))
		(incf i)))
		  
	
	  ;; Otherwise, pick the mode with the fewest configurations and sample a new configuration
	  (let* ((entry (minimizing-element modes #'num-configs))
		 (conf (funcall (me-sampler entry)))
		 (cspace (me-cspace entry))
		 (new-state (make-instance (class-of init-state) 
			      :mode (me-mode entry) :conf conf 
			      :domain d :cspace cspace)))
	  
	    (debug-prompt 0 "~%~%Adding state ~a" (display-state d new-state))
	    (incf i)
	    (connect-to-existing-states cspace roadmap new-state (me-states entry))
	    (add-new-modes new-state)
	    (vector-push-extend new-state (me-states entry))
	    ))))))

(defparameter *max-num-connections* 15)

(defun connect-to-existing-states (cspace roadmap state states &aux (conf (conf state)))
  (let ((distances
	 (sort
	  (mapset 'vector #'(lambda (n) (cons n (distance cspace conf (conf n)))) states)
	  #'< :key #'cdr)))
    (add-node roadmap state)
    (dotimes (i (min *max-num-connections* (length distances)))
      (let ((n (car (aref distances i))))
	(unless (or (eq (connected-component n roadmap) (connected-component state roadmap))
		    (path-collides (conf n) conf cspace))
	  (add-edge roadmap n state))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (mode-entry (:conc-name me-) (:constructor create-mode-entry (mode cspace sampler states)))
  mode cspace sampler states n)

(defun make-mode-entry (state sampling-fn)
  (create-mode-entry (mode state) (cspace state) (funcall sampling-fn (cspace state))
		     (make-adjustable-array :initial-contents (list state))))

(defun num-configs (mode-entry)
  (length (me-states mode-entry)))
