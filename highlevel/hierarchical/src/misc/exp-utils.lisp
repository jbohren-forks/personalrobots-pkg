;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; misc/exp-utils.lisp
;; utilities for running experiments
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package utils)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; random states
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *saved-random-state*)

(defun srs ()
  "save the current random state.  Given a piece of randomized code FOO, calling (progn (srs) (foo)), and later (assuming no intervening calls to srs), (progn (rrs) (foo)) will yield the same result."
  (setf *saved-random-state* (make-random-state))
  (values))

(defun rrs ()
  "see srs"
  (setf *random-state* (make-random-state *saved-random-state*))
  (values))


(defmacro randomized-trials (n &body body)
  "randomized-trials N &res BODY.
Run BODY N times and return 1) the results of each run in a vector 2) the sample average of the results 3) the sample standard deviation."
  (with-gensyms (l i avg)
    `(let ((,l nil))
       (dotimes (,i ,n)
	 
	 (push (progn ,@body) ,l))
       (let ((,avg (apply #'average ,l)))
	 (values (coerce ,l 'vector) ,avg (apply #'std ,avg ,l))))))




(defgeneric average (arg1 &rest args))

(defmethod average ((arg1 number) &rest args)
  (float (/ (reduce #'+ (cons arg1 args)) (1+ (length args)))))

(defmethod average ((arg1 list) &rest args)
  (apply #'mapcar #'average arg1 args))

(defmethod average ((arg1 vector) &rest args)
  (apply #'map 'vector #'average arg1 args))

(defgeneric std (avg arg1 &rest args))

(defmethod std (avg (arg1 number) &rest args)
      (sqrt
       (- (/ (loop
		 for x in (cons arg1 args)
		 sum (* x x))
	     (1+ (length args)))
	  (* avg avg))))


(defmethod std (avg (arg1 list) &rest args)
  (apply #'mapcar #'std avg arg1 args))

(defmethod std (avg (arg1 vector) &rest args)
  (apply #'map 'vector #'std avg arg1 args))
     

