(in-package :env)


(defvar *hist*)

(defun io-interface (e)
  (setq *hist* (make-adjustable-array))
  (let ((agent (make-instance '<prompt-agent> :print-prefix t :choice-fn #'(lambda (s) (avail-actions e s)))))
    (handler-case
	(loop
	   (let* ((s (get-state e))
		  (a (make-choice-at-state agent s)))
	     (mvbind (r p term) (do-action e a)
	       (format t "~&Performed action ~a~&  Reward: ~a~&  Observed: ~a"
		       a r p)
	       (when term
		 (format t "~&Environment terminated.")
		 (return)))))
      (choose-to-abort (c) (declare (ignore c)) (format t "~&Aborting io-interface.")))))


  