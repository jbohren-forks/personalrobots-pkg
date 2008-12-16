(in-package mplan)

(defgeneric add-to-roadmap (cspace roadmap conf)
  (:documentation "Add configuration CONF to ROADMAP."))

(defun connect-using-roadmap (cspace g start end)
  "connect-using-roadmap CSPACE ROADMAP START END.  Add START and END to the roadmap, then find a path between them."
  (let ((confs 
	 (mapcar 
	  #'configuration 
	  (unweighted-shortest-path g (add-to-roadmap cspace g start) (add-to-roadmap cspace g end)))))
    
    (assert confs nil "Couldn't find path from ~a to ~a using roadmap" start end)
    (confs->path cspace confs)))


(defun confs->path (cspace confs)
  (let ((path-segments
	 (let ((s 0))
	   (mapcar
	    #'(lambda (c1 c2)
		(list (get-path cspace c1 c2) s (incf s (distance cspace c1 c2))))
	    confs (cdr confs)))))
    
    ;; Normalize
    (let ((total-distance (third (slast path-segments))))
      (dolist (x path-segments)
	(divf (third x) total-distance)
	(divf (second x) total-distance)))
    
    ;; Return path
    (values 
     confs
     #'(lambda (x)
	 (let ((entry (check-not-null (find-if #'(lambda (entry) (between x (second entry) (third entry)))
					       path-segments))))
	   (dbind (path start end) entry
	     (funcall path (/ (- x start) (- end start)))))))))
    
    

