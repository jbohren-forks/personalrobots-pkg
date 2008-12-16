(defpackage hybrid-planning
  (:documentation "
Package hplan

Types
-----
<hybrid-planning-problem>
<hybrid-state>

Operations on planning problems
-------------------------------
mode
init-mode
same-mode
conf
domain
init-conf
state-type
cspace-family
cspace
avail-symbolic-actions
all-symbolic-actions
mode-transition

Roadmaps
--------
construct-hybrid-roadmap

")
  
  (:export
   <hybrid-planning-problem>
   <hybrid-state>
   
   mode
   conf
   init-mode
   same-mode
   domain
   init-conf
   state-type
   cspace-family
   cspace
   avail-symbolic-actions
   all-symbolic-actions
   mode-transition
   
   construct-hybrid-roadmap
   )
  (:use mplan
	cl
	utils
	set
	graph
	queue
	geometry
	env
	lin-alg
	hla)
  (:nicknames hplan)

  )

(in-package hplan)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hybrid states
;; the abstract class just ensures that the cspace field
;; is appropriately set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <hybrid-state> ()
  ((conf :initarg :conf :reader conf)
   (cspace :accessor cspace :initarg :cspace)
   (domain :accessor domain :initarg :domain)
   (mode :initarg :mode :reader mode)))

(defmethod initialize-instance :after ((s <hybrid-state>) &rest args &key cspace-family mode cspace)
  (declare (ignore args))
  (assert (xor cspace-family cspace))
  (when cspace-family
    (setf (cspace s) (get-cspace cspace-family mode))))

(defmethod print-object ((s <hybrid-state>) str)
  (print-unreadable-object (s str :type t :identity nil) 
    (format str "Mode ~a.  Conf ~a." (mode s) (conf s))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hybrid planning problems
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <hybrid-planning-problem> (<planning-problem>)
  ((cspace-family :accessor cspace-family)
   (state-type :accessor state-type)
   (init-mode :accessor init-mode)
   (init-conf :accessor init-conf)
   (transition-function :reader trans-fn :initarg :trans-fn))
  (:documentation "Class <hybrid-planning-problem> (<planning-problem>)

Subclasses must set the slots cspace-family, init-mode, state-type, init-conf, and implement avail-symbolic-actions and mode-transition, and goal.

The domain works as follows:
- The initial state is obtained by making an instance of state-type with arguments :mode and :conf using the given initial values, and cspace obtained by calling cspace-family with the given mode
- The available actions at a state are:
 - A (free) configuration from the current cspace.  This results in a new state with the same mode and cspace but new configuration.
 - the result of calling avail-symbolic-actions on the current state.  For each such action, get a state with the same configuration, but mode obtained by calling transition-function, and corresponding cspace.  It is assumed that these actions are lists."))


(defmethod init-state ((d <hybrid-planning-problem>))
  (make-instance (state-type d) :domain d :mode (init-mode d) :conf (init-conf d) :cspace-family (cspace-family d)))

(defmethod avail-actions ((d <hybrid-planning-problem>) s)
  (disjoint-union (avail-symbolic-actions d s)
		  (directly-reachable-confs (cspace s) (conf s))))

(defmethod all-actions ((d <hybrid-planning-problem>))
  (disjoint-union (all-symbolic-actions d) (all-confs (cspace (init-state d)))))
		  

(defmethod primitive-action-description ((d <hybrid-planning-problem>) a)
  (if (listp a) (make-symbolic-action-description d a) a))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Moves
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod succ-state ((s <hybrid-state>) dest-conf)
  (if (typep dest-conf 'finish-desc)
      (call-next-method)
      (let ((cspace (cspace s)))
	(if (path-collides (conf s) dest-conf cspace)
	    s
	    (make-instance (class-of s) :domain (domain s) :mode (mode s) :conf dest-conf :cspace cspace)))))

(defstruct (symbolic-action-description (:conc-name sd-) (:constructor make-symbolic-action-description (d a)))
  d a)

(defmethod succ-state ((s <hybrid-state>) (desc symbolic-action-description))
  (let ((domain (sd-d desc)))
    (make-instance (class-of s) :domain domain :conf (conf s) :mode (mode-transition domain s (sd-a desc))
		   :cspace-family (cspace-family domain))))

(defmethod reward :around ((d <hybrid-planning-problem>) s a)
  (if (listp a)
      (call-next-method)
    (- (distance (cspace s) (conf s) a))))
  

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; To implement by subclasses
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric mode-transition (domain s a)
  (:documentation "Return new mode after doing A in S."))

(defgeneric avail-symbolic-actions (domain s)
  (:documentation "Return set of available symbolic actions in s."))

(defgeneric all-symbolic-actions (domain)
  (:documentation "Set of all possible symbolic actions in this hybrid domain."))

(defgeneric same-mode (m1 m2)
  (:documentation "same-mode M1 M2.  Return t iff M1 and M2, understood as modes of a hybrid planning problem, are the same.")
  (:method (m1 m2) (equalp m1 m2)))