(defpackage :visual-nav
  (:use :roslisp :cl :visual_nav-msg)
  (:export :teleop-exec))

(in-package :visual-nav)

		 
		 
       

(defun teleop-exec ()
  (with-ros-node ("teleop-exec")
    (advertise "visual_nav_goal" "visual_nav/VisualNavGoal")
    (loop
       (format t "~&Enter visual nav command: ")
       (let ((cmd (read)))
	 (if (symbolp cmd)
	     (let ((cmd-name (string-upcase (symbol-name cmd))))
	       (cond
		 ((equal cmd-name "G") (handle-goto))
		 ((equal cmd-name "A") (handle-annotate))
		 ((equal cmd-name "Q") (return))
		 (t (ros-debug visual-nav "Unrecognized command ~a" cmd-name))))
	     (ros-debug visual-nav "Unrecognized command ~a" cmd))))))


(defun handle-goto ()
  (publish "visual_nav_goal" (make-message "visual_nav/VisualNavGoal" :goal (read))))

(defun handle-annotate ()
  (format t "~&Dummy annotating current node with ~a" (read)))
    