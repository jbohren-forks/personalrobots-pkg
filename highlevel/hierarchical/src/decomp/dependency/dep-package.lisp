(defpackage :dependency-graph
    (:use :cl :utils :set :graph :mapping :queue)
  (:nicknames :dep-graph)
  (:export 
   :make-dependency-graph
   :<dependency-graph>
   :add-variable
   :add-dependency
   :update-external-variable
   :up-to-date?
   :variables
   :out-of-date-variables 
   :do-all-updates
   :do-next-update
   :current-value
   :up-to-date-value
   :apply-diff
   :new-val-diff
   :new-val
   :tie-variables

   :make-simple-update-fn
   :make-simple-aggregator
   :make-simple-alist-updater
   :aggregator
   :copier

   :*dep-graph-debug-level*
)
  (:documentation "Contains the dependency graph ADT.  A dependency graph represents a computational node with a set of variables, some of which are externally set, and others of which the node is responsible for keeping up-to-date.  You can also have multiple communicating dependency graphs.  This implemented by tying external variables of one graph to variables of another graph."))


(in-package :dependency-graph)

(define-debug-topic :dep-graph :decomp)