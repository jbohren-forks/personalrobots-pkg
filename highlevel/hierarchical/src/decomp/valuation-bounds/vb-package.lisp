(defpackage :valuation-bound-node
    (:nicknames :vb-node)
  (:use :cl :mapping :utils :set :dependency-graph)
  (:export)
  (:documentation "Valuation bound nodes are decomposed planning nodes that implement the following internode protocol:

- A node has input variables {initial|final}-{optimistic|pessimistic} from its parent.
- A node computes variables {progressed|regressed}-{optimistic|pessimistic} for its parent.


"))
   
