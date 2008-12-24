(in-package :dependency-graph)


(defun make-simple-update-fn (f)
  "Return an update-fn suitable for add-variable.  The function ignores its second, third, and fourth arguments and calls F on its first argument, and returns V, (new-val-diff V), nil, t where V is the return value.  In other words, the function saves no state, just uses the parent values rather than diffs, and computes the updated value in a single step."
  #'(lambda (l &rest args)
      (declare (ignore args))
      (let ((v (funcall f l)))
	(values v (new-val-diff v) nil t))))

(defun aggregator (f)
  "Returns a function that takes in an alist and reduces the values using F."
  #'(lambda (l) (reduce f l :key #'cdr)))

(defun make-simple-aggregator (f)
  (make-simple-update-fn (aggregator f))) 

(defmacro make-simple-alist-updater (vars &body body)
  `(make-simple-update-fn (make-alist-function ,vars ,@body)))

;; Just return the first parent val
(defun copier (&rest args)
  (assert (= (length (car args)) 1))
  (let ((val (cdaar args)))
    (values val (new-val-diff val) nil t)))