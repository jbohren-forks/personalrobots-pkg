(defpackage :visual-nav
  (:use :roslisp :cl :visual_nav-msg :std_msgs-msg)
  (:export :teleop-exec))

(in-package :visual-nav)

		 
		 
       

(defun teleop-exec ()
  (with-ros-node ("teleop-exec")
    (advertise "visual_nav_goal" "visual_nav/VisualNavGoal")
    (advertise "name_node" "std_msgs/String")
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
  (let ((dest (read)))
    (publish "visual_nav_goal" 
	     (make-message "visual_nav/VisualNavGoal" 
			   goal (if (numberp dest) dest -1)
			   named_goal (if (numberp dest) "" (symbol-name dest))))))

(defun handle-annotate ()
  (publish "name_node" (make-message "std_msgs/String" :data (symbol-name (read))))
  )
    