(defpackage :valuation-bound-node
    (:nicknames :vb-node)
  (:use :cl :mapping :utils :set :dependency-graph :decomp :lookahead)
  (:export
   :<or-node>
   :<primitive-node>
   :<sequence-node>

   :compute-cycle
   :child

   :action-description
   :planning-domain

   :progress-optimistic
   :progress-pessimistic
   :regress-optimistic
   :regress-pessimistic

   :initial-optimistic
   :final-optimistic
   :initial-pessimistic
   :final-pessimistic
   :progressed-optimistic
   :progressed-pessimistic
   :regressed-optimistic
   :regressed-pessimistic
   :my-progressed-optimistic
   :my-progressed-pessimistic
   :my-regressed-optimistic
   :my-regressed-pessimistic
   :child-progressed-optimistic
   :child-progressed-pessimistic
   :child-regressed-optimistic
   :child-regressed-pessimistic
   :children-progressed-optimistic
   :children-progressed-pessimistic
   :children-regressed-optimistic
   :children-regressed-pessimistic
   

   :print-children
   :node-inputs
   :node-outputs
   :*progress-optimistic-counts*
   :*progress-pessimistic-counts*
   :*regress-optimistic-counts*
   :*regress-pessimistic-counts*
   )
  (:documentation "Valuation bound nodes are decomposed planning nodes that implement the following internode protocol:

- A node has input variables {initial|final}-{optimistic|pessimistic} from its parent.
- A node computes variables {progressed|regressed}-{optimistic|pessimistic} for its parent.


"))
   

(in-package :vb-node)

(define-debug-topic :vb-node :decomp)