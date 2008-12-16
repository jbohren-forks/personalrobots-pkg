(in-package set)

(defclass <recursive-enumeration> (<numbered-set>)
  ((initial-state-function :initarg :init-state-fn :reader initial-state-fn)
   (state-transition-function :initarg :trans-fn :reader trans-fn)
   (output-function :initarg :output-fn :reader output-fn))
  (:documentation "Class <recursive-enumeration> (<numbered-set>).  Initargs
:init-state - any object.
:state-transition-function - takes in a state and returns another one.
:output-function - takes in a state and returns an element of the set.

State-transition function, when applied starting at the initial state, must return a sequence of different states.  It may also return the symbol :no-more-elements.  Output-function must be a 1-1 function."))

(defmethod iterator ((s <recursive-enumeration>))
  (let ((state (funcall (initial-state-fn s))))
    #'(lambda ()
	(case state
	  (:no-more-elements (iterator-done))
	  (otherwise (prog1 
			 (iterator-not-done (funcall (output-fn s) state))
		       (setf state (funcall (trans-fn s) state))))))))


(defmethod item-number (item (s <recursive-enumeration>) &aux (test (equality-test s)))
  ;; will run forever if the set is infinite and does not contain the item
  (do-elements (x s (error 'item-not-in-set :item item :set s) i)
    (when (funcall test x item)
      (return i))))

(defmethod item (num (s <recursive-enumeration>))
  (do-elements (x s (error 'index-out-of-bounds :ind num :set s :max-ind (1- i)) i)
    (when (eq i num)
      (return x))))
      
  
(defmethod member? (item (s <recursive-enumeration>))
  ;; may run forever
  (item-number item s))
		       
