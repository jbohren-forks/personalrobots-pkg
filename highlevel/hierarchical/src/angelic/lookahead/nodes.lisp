;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Various kinds of nodes in ALTs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package lookahead)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node annotated with action sound and complete rewards
;; and a complete Q-value
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <node-with-action-rewards> (<alt-node>)
  ((sound-reward :accessor sound-reward :initform '-infty)
   (complete-reward :accessor complete-reward :initform 'infty)
   (complete-q :accessor complete-q :initform '-infty)
   (sound-q :accessor sound-q :initform '-infty))
  (:documentation "Extension to standard nodes that labels actions with a numeric sound-reward, complete-reward and q-value.  These quantities are defined using the difference between the maxima of the adjacent valuations, which may not be so reasonable for nonsimple valuations.

The nodes then store complete and sound q-values, which are computed backwards (i.e. starting from the leaf up to the root) whenever changes are made.
"))

(defmethod update-forward :after ((n <node-with-action-rewards>) parent)
  (setf (sound-reward n) (max-valuation-change (sound-valuation parent) (sound-valuation n))
	(complete-reward n) (max-valuation-change (complete-valuation parent) (complete-valuation n))))


(defmethod update-backward :after ((n <node-with-action-rewards>) known-values &aux (children (active-child-nodes n)))
  (unless (root? n)
    (let ((new-complete-q 
	   (my+ (complete-reward n) 
		(if (is-empty children)
		    (cond ((known-value n)
			   (or (lookup-known-value 'state (check-not-null (state n)) 'complete-value known-values) 0.0))
			  ((subsumed n) '-infty)
			  (t 0.0))
		  (reduce-set #'mymax children :key #'complete-q))))
	  (new-sound-q
	   (my+ (sound-reward n) 
		(if (is-empty children)
		    (cond ((known-value n)
			   (or (lookup-known-value 'state (check-not-null (state n)) 'sound-value known-values) '-infty))
			  ((subsumed n) '-infty)
			  (t 0.0))
		  (reduce-set #'mymax children :key #'sound-q)))))
      (when (change-if-necessary (complete-q n) new-complete-q)
	(push 'complete-q (changed n)))
      (when (change-if-necessary (sound-q n) new-sound-q)
	(push 'sound-q (changed n))))))
      

(defmethod update-upward :after ((n <node-with-action-rewards>) &aux (spans (downward-spans n)))
  (flet ((span-summer (fn)
	   #'(lambda (span)
	       (do ((start (car span))
		    (n (cdr span) (parent-node n))
		    (s 0))
		   ((eq n start) s)
		 (_f my+ s (funcall fn n))))))
  
    (let ((new-sound (mymax (sound-reward n) (reduce-set #'mymin spans :key (span-summer #'sound-reward))))
	  (new-complete (mymin (complete-reward n) (reduce-set #'mymax spans :key (span-summer #'complete-reward)))))
      (when (or (eq new-sound 'infty) (eq new-complete '-infty))
	;;(break)
	)
      (when (change-if-necessary (sound-reward n) new-sound)
	(push 'sound-reward (changed n)))
      (when (change-if-necessary (complete-reward n) new-complete)
	(push 'complete-reward (changed n))))))


(defmethod print-object ((l <node-with-action-rewards>) str)
  (print-unreadable-object (l str :type nil :identity nil)
    (format str "~a (~:[in~;~]active).  [~a, ~a].  ~:[~;OODF ~]~:[Complete Q: ~a~;~*OODB ~]"
	    (action l) (eq (status l) 'active) (sound-reward l) (complete-reward l)
	    (out-of-date-forward-flag l) (out-of-date-backward-flag l) (complete-q l))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Node with a priority value that depends on action,
;; complete and sound rewards
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <node-with-priority> (<node-with-action-rewards>)
  ((priority-fn :accessor priority-function :initarg :priority-fn)
   (priority :accessor priority :initform nil)
   (priority-q :accessor priority-q :initform '-infty)))

(defun priority-fn (n)
  (priority-function (root n)))

(defmethod update-forward :after ((n <node-with-priority>) parent)
  (declare (ignore parent))
  (update-priority n))

(defun update-priority (n)
  (let ((new-priority (funcall (priority-fn n) (action n) (complete-reward n) (sound-reward n))))
    (when (change-if-necessary (priority n) new-priority)
      (push 'priority (changed n)))))

(defmethod update-backward :after ((n <node-with-priority>) known-values &aux (children (active-child-nodes n)))
  (unless (root? n)
    (update-priority n)
    (let ((new-q (my+ (priority n)
		      (if (is-empty children)
			  (cond 
			   ((known-value n) (lookup-known-value 'state (check-not-null (state n)) 'priority known-values))
			   ((subsumed n) '-infty)
			   (t (assert (member (action n) '(act finish)) nil 
				"At leaf node with unknown state ~a, action was ~a instead of act or finish"
				(state n) (action n))
			      0.0))
			(reduce-set #'mymax children :key #'priority-q)))))
      (when (change-if-necessary (priority-q n) new-q)
	(push 'priority-q (changed n))))))
	
