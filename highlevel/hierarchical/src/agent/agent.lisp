(defpackage :agent
  (:documentation "General ops for agents.  An agent is something that receives observations and chooses actions.")
  (:use :cl :utils)
  (:export 
   :make-choice-at-state
   
   :choose-to-abort
   :<prompt-agent>))

(in-package :agent)


(defgeneric make-choice-at-state (agent s)
  (:documentation "Return the agent's chosen action at this state.  Does not assume that S is a state resulting from doing the agent's chosen action (or any action) in the last seen state.  But, may assume that the agent is still in the same environment, i.e., might reuse computations done on previous calls.")
  (:method ((agent function) s)
    (funcall agent s)))
    

   