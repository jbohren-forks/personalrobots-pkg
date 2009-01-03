(in-package :valuation-bound-node)

(defclass <primitive-node> (<node>) ())

(define-debug-topic :primitive-node :vb-node)

(defmethod compute-cycle ((n <primitive-node>))
  (debug-out :primitive-node 1 t "~&In compute cycle of primitive node ~a" (action n))
  (do-all-updates n))

(defmethod action-node-type ((c (eql :primitive)))
  '<primitive-node>)

(defmethod primitive-plan-with-pessimistic-future-value-above ((n <primitive-node>) s v)
  (with-readers (action planning-domain) n
    (let ((successor (action-result planning-domain s action))
	  (reward (reward planning-domain s action)))
      (when (my>= (my+ reward (evaluate-valuation (current-value n 'final-pessimistic) successor)) v)
	(values (vector action) successor reward)))))

