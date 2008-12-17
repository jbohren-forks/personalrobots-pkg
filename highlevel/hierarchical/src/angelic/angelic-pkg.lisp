(defpackage :lookahead
  (:documentation "Package lookahead (hla)
Planning problems
-----------------
<planning-problem>
primitive-action-description
succ-state
action-result
goal
goal?
init-state
same-state
reward
action-seq-result
succeeds?
all-actions
init-state-set
<propositional-domain>
propositions
prop-domain-state
make-prop-domain-state
pds-props
pds-domain
lookup-type
pd-object-type
fluents
make-prop-state-set
make-state-set
lookup-action


Hierarchies
-----------
make-variable-hierarchy
<variable-hierarchy>
planning-problem
action-type
applicable-refinements
applicable-top-level-actions
high-level
primitive
unknown-action
product
act
finish

Abstract planning problem
-------------------------
<abstract-planning-problem>
<prop-abstract-planning-problem>
sound-complete-forward-search
*refinement-set-type*
sound-result
complete-result
finish-desc

Lookahead-tree
--------------
refine
initial-tree
get-best-plan

Lookahead agent
---------------
hierarchical-real-time
hrt-agent
ahrta*
ahrta*-agent
default-priority-fn
hierarchical-forward-search
aha*
ahss
is-subsumed
add-entry
clear-subsumption-checker
make-hash-subsumption-checker
clear-entry

Valuations
----------
simple-valuation
<simple-description>
sv-s
sv-v


Abstract action descriptions
----------------------------
successor-set

Propositional planning problems
-------------------------------

action-name
action-args
holds
holds-background


Nondeterministic conditional STRIPS
-----------------------------------
strips
make-strips
make-strips-schema
nstrips
make-nstrips
*nstrips-noop*
ncstrips
make-ncstrips
make-ncstrips-schema
make-ncstrips-schemas
vacuous-sound-ncstrips-schemas
vacuous-complete-ncstrips-schemas
for-all
?complete-set
?sound-set
?set
pss-domain

Propositional hierarchies
-------------------------
<hierarchy>
<prop-hierarchy>
make-flat-prop-hierarchy
make-implementation
make-hla-schema
ground-hla-pred

CSPs
----
make-csp
solve-csp-complete
make-and-solve-csp

Stats
-----
*num-refinements*
*num-extensions*

Debugging
---------
is-well-formed

Parameters
----------
*random-refinement-ordering*

Other symbols
-------------
high-level
primitive
complete-set

")

  (:use :cl
	:utils
	:set
	:prod-set
	:queue
	:create-env
	:prob
	:mapping
	:prop-logic
	:tree)
  (:nicknames :hla)
  (:export

   :<planning-problem>
   :primitive-action-description
   :succ-state
   :action-result
   :goal
   :goal?
   :init-state
   :same-state
   :reward
   :init-state-set
   :action-seq-result
   :all-actions
   :succeeds?
   :<propositional-domain>
   :propositions
   :prop-domain-state
   :make-prop-domain-state
   :pds-props
   :pds-domain
   :lookup-type
   :pd-object-type
   :fluents
   :make-prop-state-set
   :make-state-set
   :lookup-action
  
  
   :*num-refinements*
   :*num-extensions*
   
   :is-well-formed

   
   :make-variable-hierarchy
   :<variable-hierarchy>
   :planning-problem
   :action-type
   :applicable-refinements
   :applicable-top-level-actions
   :high-level
   :primitive
   :unknown-action
   :product
   :act
   :finish

   :<hierarchy>
   :<prop-hierarchy>
   :make-flat-prop-hierarchy
   :make-implementation
   :make-hla-schema
   :ground-hla-pred

   :make-csp
   :solve-csp-complete
   :make-and-solve-csp

   :<abstract-planning-problem>
   :<prop-abstract-planning-problem>
   :sound-complete-forward-search
   :*refinement-set-type*
   :sound-result
   :complete-result
   :finish-desc

   :refine
   :initial-tree
   :get-best-plan
   
   :hierarchical-real-time
   :hrt-agent
   :ahrta*
   :ahrta*-agent
   :default-priority-fn
   :hierarchical-forward-search
   :aha*
   :ahss
   :is-subsumed
   :add-entry
   :clear-subsumption-checker
   :clear-entry
   :make-hash-subsumption-checker

   :simple-valuation
   :<simple-description>
   :sv-s
   :sv-v
   
   :successor-set

   :action-name
   :action-args
   :holds
   :holds-background
   
   :strips
   :make-strips
   :make-strips-schema
   :nstrips
   :make-nstrips
   :*nstrips-noop*
   :ncstrips
   :make-ncstrips
   :make-ncstrips-schema
   :make-ncstrips-schemas
   :vacuous-sound-ncstrips-descriptions
   :vacuous-complete-ncstrips-descriptions
   
   :for-all
   :?complete-set
   :?sound-set
   :?set
   :pss-domain
   
   :*random-refinement-ordering*
   
   :high-level
   :primitive
   :complete-set
   )
  )
