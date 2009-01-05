(defpackage :test-two-by-n
  (:use :two-by-n :cl :utils :env-user :lookahead :set :decomp :vb-node :dependency-graph))

(in-package :test-two-by-n)

(defparameter costs #3A(((1 0) (1 6) (7 1)) ((0 0) (8 4) (6 5))))
(defparameter e (make-instance '<two-by-n> :costs costs))
(defparameter h (make-instance '<two-by-n-hierarchy> :planning-domain e))
(defparameter descs (make-instance '<two-by-n-descriptions> :hierarchy h))
(defparameter p (two-by-n-abstract-planning-problem descs))

(defun make-prob (n)
  (let* ((e (random-two-by-n-env n))
	 (h (make-instance '<two-by-n-hierarchy> :planning-domain e))
	 (descs (make-instance '<two-by-n-descriptions> :hierarchy h)))
    (values descs (two-by-n-abstract-planning-problem descs))))
	   

(reset-debug-level :decomp 2 :valuation 0)

(defun reinitialize (i)
  (let ((n (top-node descs)))
    (repeat i (compute-cycle n))
    n))

(defun test (n k)
  (let ((probs nil) (descs nil))
    (repeat k
      (mvbind (desc prob) (make-prob n)
	(push desc descs)
	(push prob probs)))

    (time
     (dolist (desc descs)
       (print (find-optimal-plan desc))))
    
    (time 
     (dolist (prob probs)
       (print (aha* prob))))))

       


(defparameter n (top-node descs))

