(in-package :agent)

(define-condition choose-to-abort ()
  ())

(defclass <prompt-agent> ()
  ((advisors :type list :reader advisors :initarg :advisors :initform nil)
   (choice-fn :type function :reader choice-fn :initarg :choice-fn)
   (print-prefix :reader print-prefix :initarg :print-prefix :initform t))
  (:documentation "A dummy agent that prompts user for what to do next.

Initargs:
:choice-fn - Function that is called on a state and returns set of available choices
:advisors - list of advisors whose advice will be printed at each choice (see print-advice).  nil by default.
:print-prefix - If this equals ':state, then the state, available choices and advisor output is printed to standard output before prompting for a choice.  If it is any other true value, then just the available choices and advisor output are printed.

Behavior also affected by
cl:*print-length* - how many choices to print
"))

(defmethod make-choice-at-state ((agent <prompt-agent>) s)
  (let ((choices (funcall (choice-fn agent) s))
	(str t))
    (when (print-prefix agent)
      (format str "~&~%")
      (pprint-logical-block (str nil)
	(format str "Prompt-agent")
	(when (eq (print-prefix agent) ':state)
	  (format str "~5I~:@_State:~15T~W" s))
	(format str "~5I~:@_Choice set:~15T~W~:@_~50@{~A~:*~}" choices #\-)
	(set:do-elements (a (advisors agent) nil i)
	  (format str "~:@_Advisor ~W:~5I~:@_" i)
	  (print-advice a s choices str)
	  (format str "~:@_~50@{~A~:*~}" #\-))))
    
    (prompt-for-choice choices)))




(defun prompt-for-choice (choices)
  (format t "~&Please enter choice, or nil to terminate. ")
  ;; if one of the choices is nil, you're out of luck...
  (let ((ch (read))
	is-member reason)
    (assert
     (progn
       (unless ch
	 (error 'choose-to-abort))
       (if (eql (set:size choices) 1)
	   (progn
	     (unless (set:member? ch choices)
	       (warn "~a is not a valid choice, but using default value ~a instead."
		     ch (set:item 0 choices))
	       (setf ch (set:item 0 choices)))
	     (setf is-member t))
	   (multiple-value-setq (is-member reason)
	     (set:member? ch choices)))
       is-member)
     (ch)
     (set:get-full-explanation-string reason))
    ch))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Advice
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric print-advice (advisor state choices str)
  (:documentation "Used to customize the behavior of prompt-agent by printing information before prompting for choice."))

(defmethod print-advice ((advisor function) state choices str)
  (pprint-logical-block (str nil)
    (write (funcall advisor state choices) :stream str)))
  



    