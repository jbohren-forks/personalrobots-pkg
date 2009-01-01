(in-package :valuation-bound-node)

(defclass <primitive-node> (<node>) ())

(define-debug-topic :primitive-node :vb-node)


(defmethod compute-cycle ((n <primitive-node>))
  (debug-out :primitive-node 1 t "~&In compute cycle of primitive node ~a" (action n))
  (do-all-updates n))

(defmethod action-node-type ((c (eql :primitive)))
  '<primitive-node>)