(in-package :valuation-bound-node)

(defclass <primitive-node> (<node>) ())


(defmethod compute-cycle ((n <primitive-node>))
  (do-all-updates n))

(defmethod action-node-type ((c (eql :primitive)))
  '<primitive-node>)