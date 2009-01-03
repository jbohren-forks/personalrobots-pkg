(defpackage :test-two-by-n
  (:use :two-by-n :cl :utils :env-user :lookahead :set :decomp :vb-node :dependency-graph))

(in-package :test-two-by-n)

(defparameter costs #3A(((1 0) (1 6) (7 1)) ((0 0) (8 4) (6 5))))
(defparameter e (make-instance '<two-by-n> :costs costs))
(defparameter h (make-instance '<two-by-n-hierarchy> :planning-domain e))
(defparameter descs (make-instance '<two-by-n-descriptions> :hierarchy h))

(reset-debug-level :decomp 2 :valuation 0)
(setq 
 *progress-optimistic-counts* 0
 *progress-pessimistic-counts* 0
 *regress-optimistic-counts* 0
 *regress-pessimistic-counts* 0)

(defun counts ()
  (list *progress-optimistic-counts* *progress-pessimistic-counts* *regress-optimistic-counts* *regress-pessimistic-counts*))

(defun reinitialize (i)
  (let ((n (top-node)))
    (repeat i (compute-cycle n))
    n))




(defparameter n (top-node))

