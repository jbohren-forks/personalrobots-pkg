(in-package :vb-node)

(defun find-optimal-plan (descs)
  (let ((n (top-node descs))
	(s (init-state (planning-domain descs))))
  (loop
    (compute-cycle n)
    (let ((v (node-optimistic-value-regressed n)))
      (when (eql v (node-pessimistic-value-regressed n))
	(mvbind (plan successor reward) (primitive-plan-with-pessimistic-future-value-above n s v)
	  (declare (ignore successor))
	  (when plan (return (values plan reward)))))))))

(defun find-satisficing-plan (descs r)
  (let ((n (top-node descs))
	(s (init-state (planning-domain descs))))
  (loop
    (compute-cycle n)
    (if (my< (node-optimistic-value-regressed n) r)
      (return nil)
      (let ((plan (primitive-plan-with-pessimistic-future-value-above n s r)))
	(awhen plan (return it)))))))