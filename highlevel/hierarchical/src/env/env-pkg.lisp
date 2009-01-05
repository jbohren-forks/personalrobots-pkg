(defpackage :env
  (:documentation "Internal package for environments.  Externally used packages are env-user and create-env.")
  (:use :common-lisp
	:utils
	:agent)

  (:export 
   :<env> :<fully-observable-env> :do-action :reset :reset-to-state :at-terminal-state :get-actions :avail-actions :io-interface :get-last-percept :get-state :set-state :current-effectors 
   :sample-percept :sample-init :same :is-terminal-state :sample-next :sample-init-percept :same-state :trajectory-env :effectors
   :trajectory-states :trajectory-rewards :trajectory-actions :make-trajectory :num-transitions :transitions :is-valid-trajectory
   :untranslatable-action :translate-action :*hist*))

(defpackage :env-user
 (:documentation "Package used when interacting with an environment.

Types
-----
<env>
<fully-observable-env>
trajectory
untranslatable-action

Operations for all <env>s
--------------------------
do-action
reset
same-state
reset-to-state
at-terminal-state
get-actions
avail-actions
io-interface
get-last-percept
current-effectors
translate-action

Operations for <fully-observable-env>s
--------------------------------------
get-state

Operations on trajectories
--------------------------
trajectory-env
trajectory-states
trajectory-actions
trajectory-rewards
make-trajectory
is-valid-trajectory
num-transitions
transitions


")
 (:use :common-lisp
       :env
       )

  (:export
   :<env> :<fully-observable-env> :do-action :reset :reset-to-state :same-state :at-terminal-state :get-actions :avail-actions :io-interface :get-last-percept 
   :get-state :current-effectors :trajectory-env :trajectory-states :trajectory-rewards :trajectory-actions :make-trajectory :num-transitions :transitions 
   :is-valid-trajectory :untranslatable-action :translate-action
))



(defpackage :create-env
  (:documentation "Package used when making new types of environments.

Types
-----
<env>
<fully-observable-env>

Operations to extend
--------------------
sample-next or do-action.  Required.
sample-init or reset.  Required.
Possibly reset-to-state.
sample-percept.  Required, except for fully observable envs.
sample-init-percept.  Required, except for fully observable envs.
is-terminal-state or at-terminal-state.  Optional.
avail-actions.  Required for any algorithm that doesn't have some other way of knowing what actions are allowed.
io-interface.  Optional.

Operations for env states
-------------------------

IMPORTANT : it is assumed throughout the rest of the code that environment states are immutable objects, i.e., once one is created, it and any objects it refers to stay the same forever.  So the environment code should never modify the existing state object in the sample-next or sample-init methods, but create a new object instead.

It is recommended that you define a type for states for your environment, to allow methods to specialize on them.  In this case, the following methods should be defined for that type.

print-object.  Optional but useful.
effectors.  Required if the current-effectors function is ever called on the environment.
clone.  No longer required due to above note.
same-state.  Optional (defaults to #'equal).
canonicalize.  Required if you ever use an algorithm that uses environment states as hash keys (e.g. tabular function approximators).

Operations that child classes can use
-------------------------------------
get-last-percept
get-state
set-state.  Warning - if you need to use set-state, and the environment is not fully observable, make sure to call set-last-percept as well, otherwise the overall state of the object could be inconsistent.
set-last-percept.
")
  (:use :common-lisp :env)
  (:export
   :<env> :<fully-observable-env> :sample-next :do-action :sample-init :reset :reset-to-state :sample-percept :sample-init-percept :is-terminal-state 
   :at-terminal-state :avail-actions :io-interface :print-object :same-state :effectors :get-last-percept :set-last-percept :get-state :set-state))


