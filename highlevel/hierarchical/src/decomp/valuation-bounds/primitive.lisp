(in-package :valuation-bound-node)

(defclass <primitive-node> (<node>) ())

;; For primitive nodes, just use the internal progressed/regressed valuations as output
(defmethod optimistic-progressor ((n <primitive-node>))
  #'copier)

(defmethod pessimistic-progressor ((n <primitive-node>))
  #'copier)

(defmethod optimistic-regressor ((n <primitive-node>))
  #'copier)

(defmethod pessimistic-regressor ((n <primitive-node>))
  #'copier)

(defmethod compute-cycle ((n <primitive-node>))
  (do-all-updates n))


