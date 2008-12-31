(defpackage :decomp-driver
  (:use :lookahead
	:decomp
	:utils
	:blocks
	:cl
	:decomp
	:mapping
	:env-user
	:dependency-graph
	:set
	:vb-node
	:prop-logic)
  (:import-from
   :vb-node
   :children
   :child
   
   :children-progressed-optimistic
   :children-progressed-pessimistic
   :children-regressed-optimistic
   :children-regressed-pessimistic
   :child-progressed-optimistic
   :child-progressed-pessimistic
   :child-regressed-optimistic
   :child-regressed-pessimistic

   :status)
  (:import-from
   :dep-graph 
   :graph)
  )

(in-package :decomp-driver)

(defvars nr nc dom hier descs)

(set-debug-level :decomp 2)

(setf nr 4
      nc 4
      dom (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
					  '(2 3) '(and (on a c) (on c :t1)))
      hier (make-instance '<blocks-hierarchy> :domain dom)
      descs (make-instance '<blocks-descriptions> :domain dom :heuristic (dist-heuristic dom) :hierarchy hier))
      
      

 



(defun top-node ()
  (let ((n (make-instance '<or-node> :action '(act) :descs descs :hierarchy hier :domain dom :parent nil))
	(init-exact (new-val-diff (make-simple-valuation (init-state-set dom) 0)))
	(final-exact (new-val-diff (make-simple-valuation (goal dom) 0)))) 
    (update-external-variable n 'initial-optimistic init-exact)
    (update-external-variable n 'initial-pessimistic init-exact)
    (update-external-variable n 'final-pessimistic final-exact)
    (update-external-variable n 'final-optimistic final-exact)
    n))