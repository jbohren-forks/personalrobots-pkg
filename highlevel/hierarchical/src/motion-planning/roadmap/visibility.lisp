(in-package mplan)


(defclass <visibility-roadmap-node> ()
  ((configuration :reader conf :initarg :conf)
   (node-type :reader node-type :initarg :node-type)))

(defclass <visibility-roadmap> (<adjacency-list-graph> <union-find-graph>)
  ()
  (:documentation "Class <visibility-roadmap>

In a visibility roadmap, nodes are either guards or connectors.  When a new node is added, if no guards are visible to it, it is added as a guard.  Else, if at least two connected components of the current roadmap are visible to it , it is added as a connector.  Else, it is not added.

"))


(defun visibility-roadmap-paths (cspace c1 c2 n)
  (if (path-collides c1 c2 cspace)
      (let* ((s (free-space-sampler cspace))
	     (g (make-instance '<visibility-roadmap>))
	     (start-node (check-not-null (add-to-visibility-roadmap cspace g c1)))
	     (end-node (check-not-null (add-to-visibility-roadmap cspace g c2))))
	(repeat n (add-to-visibility-roadmap cspace g (funcall s)))
	(let ((path (unweighted-shortest-path g start-node end-node)))
	  (when path (list (mapcar #'conf path)))))
    
    (list (list c1 c2))))


(defun add-to-visibility-roadmap (cs g c)
  (let ((visible-nodes nil))
    
    ;; For each component
    (do-elements (comp (connected-components g))
      
      ;; Iterate over nodes in component
      (do-elements (n comp)
	
	;; When you find a visible guard, note it and move to the next component
	(when (and (eq (node-type n) 'guard)
		   (not (path-collides c (conf n) cs)))
	  (push n visible-nodes)
	  (return nil))))
    
    ;; Add the node as a guard if it sees no other guards, a connector if it sees two or more, and don't add otherwise
    (let ((node-type
	   (cond ((null visible-nodes) 'guard)
		 ((length-exceeds visible-nodes 1) 'connector)
		 (t nil))))
      (when node-type
	;;(draw-roadmap g)
	(debug-print 2 "~&Adding ~a as ~a" c node-type)
	(let ((new-node (make-instance '<visibility-roadmap-node>
			  :node-type node-type :conf c)))
	  (add-node g new-node)
	  (dolist (n visible-nodes new-node)
	    (add-edge g n new-node)))))))