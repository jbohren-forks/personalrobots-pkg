(defpackage :valuation-bound-node
    (:nicknames :vb-node)
  (:use :cl :mapping :utils :set :dependency-graph :decomp :lookahead)
  (:export
   :<or-node>
   :<primitive-node>
   :<sequence-node>

   :<vb-descriptions>
   :top-node-type
   :top-action
   :top-node

   :find-optimal-plan
   :find-satisficing-plan

   :compute-cycle
   :child
   :cycle-number

   :action-description
   :planning-domain
   :hierarchy
   :initial-valuation
   :final-valuation
   :minimal-valuation
   :maximal-valuation

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
   :node-optimistic-value-regressed
   :node-pessimistic-value-regressed
   

   :print-children
   :node-inputs
   :node-outputs
   )
  (:documentation "Valuation bound nodes are decomposed planning nodes that implement the following internode protocol:

- A node has input variables {initial|final}-{optimistic|pessimistic} from its parent.
- A node computes variables {progressed|regressed}-{optimistic|pessimistic} for its parent.


"))
   

(in-package :vb-node)

(define-debug-topic :vb-node :decomp)