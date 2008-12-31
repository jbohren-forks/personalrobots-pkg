(defpackage :test-two-by-n
  (:use :two-by-n :cl :utils :env-user :lookahead :set :decomp :vb-node :dependency-graph))

(in-package :test-two-by-n)

(defparameter costs #3A(((1 0) (1 6) (7 1)) ((0 0) (8 4) (6 5))))
(defparameter e (make-instance '<two-by-n> :costs costs))
(defparameter h (make-instance '<two-by-n-hierarchy> :planning-domain e))
(defparameter descs (make-instance '<two-by-n-descriptions> :hierarchy h))

(set-debug-level :decomp 2)

(defun top-node ()
  (let ((n (make-instance '<sequence-node> :action '(top) :descs descs :parent nil))
	(init (new-val-diff (initial-valuation e)))
	(final (new-val-diff (final-valuation e))))
    (update-external-variable n 'initial-optimistic init)
    (update-external-variable n 'initial-pessimistic init)
    (update-external-variable n 'final-optimistic final)
    (update-external-variable n 'final-pessimistic final)
    n))


(defparameter n (top-node))

