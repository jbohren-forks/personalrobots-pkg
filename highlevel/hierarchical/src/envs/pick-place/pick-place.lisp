(defpackage pick-place
  (:documentation "

Types
-----
<pick-place-env>
<simple-cspace-family>
<simple-cs-mode>

Actions
-------
putdown
pickup

Hierarchies
-----------
simple-hierarchy

HLAs
----
grasp-side
transfer
transfer-to

Descriptions
------------
simple-hierarchy-sound-desc
simple-hierarchy-complete-desc


Predicates
----------
held

State sets
----------
make-pps-set
pss-objects
pss-confs

")
     
 
  (:export
   <pick-place-env>
   <simple-cspace-family>
   <simple-cs-mode>
   
   putdown
   pickup
   
   simple-hierarchy
   
   grasp-side
   transfer
   transfer-to
   
   simple-hierarchy-sound-desc
   simple-hierarchy-complete-desc

   held
   
   make-pps-set
   pss-objects
   pss-confs

   )
  
   (:import-from mapping evaluate)
  (:use
   cl
   lin-alg
   utils
   prob
   set
   hplan
   hla
   geom
   mplan))


(in-package pick-place)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; the cspace-family
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <simple-cs-mode> ()
  ((held :accessor held :initarg :held)
   (confs :accessor confs :initarg :confs)
   (held-object-conf :accessor held-object-conf :initarg :held-object-conf )))

(defclass <simple-cspace-family> ()
  ((objects :accessor objects)
   (robot :accessor robot :initarg :robot)
   (bounds :accessor bounds :initarg :bounds :initform '(#(-infty -infty) #(infty infty))))
  (:documentation "Simple type of cspace family.  Corresponds to a set of simple-cspaces, with a fixed set of n objects and robot.  The parameters of a particular cspace are (HELD-OBJECT HELD-OBJECT-CONF C(0) ... C(n-1)) where 
- HELD-OBJECT is either an integer in 0:n-1 or nil
- HELD-OBJECT-CONF is the relative configuration of the held object to the robot (maps from object frame into robot frame)
- C(i) designates the configuration of object i (of type rigid-2d)

The initargs are
:objects - vector of objects, each of which is a <polygon>.  Note that the coordinate frame here is not absolute as objects can move.
:robot, :bounds - as in <simple-cspace>
"))

(defmethod initialize-instance :after ((f <simple-cspace-family>) &rest args &key objects)
  (setf (objects f) (coerce objects 'vector)))

(defmethod initialize-instance :after ((m <simple-cs-mode>) &rest args)
  (when (held m) (assert (slot-boundp m 'held-object-conf))))

(defmethod get-cspace ((f <simple-cspace-family>) params)
  (with-slots (objects robot bounds) f
    (with-slots (held confs held-object-conf) params
      
      ;; Get the list of transformed nonheld objects
      (let ((obstacles (make-adjustable-array)))
	(do-elements (c confs nil i)
	  (vector-push-extend (unless (eql i held) (transform c (aref objects i))) obstacles))
	
	;; Make a simple-cspace with the nonheld objects, and possibly augmented robot
	(make-instance '<simple-cspace>
	  :bounds bounds :obstacles obstacles
	  :robot (if held
		     (implicit-union robot (transform held-object-conf (aref objects held)))
		   robot))))))

(defmethod same-mode ((m1 <simple-cs-mode>) (m2 <simple-cs-mode>))
  (and (eql (held m1) (held m2))
       (let ((c1 (confs m1))
	     (c2 (confs m2)))
	 (and (= (length c1) (length c2))
	      (each (length c1) #'(lambda (x) (or (eql x (held m1)) (equal-transformations (elt c1 x) (elt c2 x)))))
	      (aif (held-object-conf m1) 
		  (equal-transformations it (held-object-conf m2))
		t)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; pick place state
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <pick-place-state> (<hybrid-state>)
  ((gripper-dists :accessor pps-gripper-dists)
   (world-robot :reader pps-world-robot :writer set-world-robot :initform ':unassigned)))

(defmethod initialize-instance :after ((s <pick-place-state>) &rest args)
  (declare (ignore args))
  (let* ((m (mode s))
	 (n (length (confs m))))
    (setf (pps-gripper-dists s) (make-array n :initial-element ':unassigned))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Accessors, some of which are
;; memoized for derived predicates
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod held ((s <pick-place-state>))
  (held (mode s)))

(defmethod confs ((s <pick-place-state>))
  (confs (mode s)))

(defmethod held-object-conf ((s <pick-place-state>))
  (held-object-conf (mode s)))

(define-memoized-reader 
    world-robot #'pps-world-robot #'set-world-robot
    #'(lambda (s) (transform (conf s) (robot (domain s)))))


(define-memoized-reader gripper-dist #'(lambda (s i) (aref (pps-gripper-dists s) i))
  #'(lambda (v s i) (setf (elt (pps-gripper-dists s) i) v))
  #'(lambda (s i) (min-distance (world-robot s) (elt (obstacles (cspace s)) i))))

(defmethod print-object ((s <pick-place-state>) str)
  (let ((obs (obstacles (cspace s))))
    (print-unreadable-object (s str :type t :identity nil)
      (format str "Conf ~a.  Held ~a~&Object positions ~a~&Distances ~a" (conf s) (held s) 
	      (subseq obs 0 (- (length obs) 4))
	      (mapset 'list #'(lambda (i) (gripper-dist s i)) (- (length obs) 4))))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; pick place domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *pickup-tol* .5)

(defclass <pick-place-env> (<hybrid-planning-problem>)
  ((init-conf :initarg :init-conf)
   (num-objects :accessor num-objects)
   (state-type :initform '<pick-place-state>)
   (robot :initarg :robot :accessor robot)
   (pickup-tol :accessor pickup-tol)
   (bounds :initarg :bounds :accessor bounds)
   (grasp-cost :initform 1 :initarg :grasp-cost :reader grasp-cost))
  (:documentation "Class <pick-place-env> (<hybrid-planning-problem>)

Initargs
:init-conf - initial robot configuration
:object-confs - array of initial object configurations
:robot, :objects, :bounds - as in <simple-cspace-family>
:grasp-cost - cost of a grasp/ungrasp.  Defaults to 1.
"))

(defmethod initialize-instance :after ((p <pick-place-env>) &rest args &key objects robot bounds object-confs)
  (declare (ignore args))
  (setf (cspace-family p) (make-instance '<simple-cspace-family> :objects (coerce objects 'vector) :robot robot :bounds bounds)
	(num-objects p) (length objects)
	(pickup-tol p) *pickup-tol*
	(init-mode p) (make-instance '<simple-cs-mode> :held nil :confs object-confs)))

(defmethod all-symbolic-actions ((p <pick-place-env>))
  (cons '(putdown) (mapset 'list #'(lambda (i) `(pickup ,i)) (num-objects p))))

(defmethod avail-symbolic-actions ((p <pick-place-env>) (s <pick-place-state>))
  (if (held s)
      '((putdown))
    (loop for i below (num-objects p) when (< (gripper-dist s i) (pickup-tol p)) collect (list 'pickup i))))

(defmethod mode-transition ((p <pick-place-env>) (s <pick-place-state>) (a list))
  (dsbind (act &rest params) a
    (ecase act
      (pickup (pickup-transition s (first params)))
      (putdown (putdown-transition s)))))

(defmethod reward ((p <pick-place-env>) s a)
  ;; Note that the superclass takes care of move actions, so this is just for grasp/ungrasp
  (declare (ignore s a))
  (- (grasp-cost p)))

(defun pickup-transition (s i)
  (if (and (not (held s)) (< (gripper-dist s i) (pickup-tol p)))
      (make-instance '<simple-cs-mode>
		     :held i
		     :confs (copy-and-modify (confs s) i nil)
		     :held-object-conf (transform-to-robot-coords (conf s) (elt (confs s) i)))
      (mode s)))

(defun putdown-transition (s)
  (make-instance '<simple-cs-mode>
    :held nil
    :confs (copy-and-modify (confs s) (held s) (transform-to-world-coords (conf s) (held-object-conf s)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Other info about domain
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun movable-objects (d)
  "movable-objects D.  Return the set of IDs of movable objects."
  (objects (cspace-family d)))

(defun num-movable-objects (d)
  (size (movable-objects d)))


  
(defun num-sides (d o)
  "num-sides D I.  How many sides does object I have?"
  (length (vertices (item o (objects (cspace-family d))))))

(defun object (d i)
  "object D I.  Return the (point set corresponding to the) Ith movable object (represented in its coordinate frame)."
  (aref (objects (cspace-family d)) i))

(defmethod all-confs ((d <pick-place-env>))
  (all-confs (cspace (init-state d))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun copy-and-modify (a i x)
  "a copy of sequence A with the Ith element changed to X"
  (let ((b (copy-seq a)))
    (setf (elt b i) x)
    b))

(defun transform-to-robot-coords (robot-conf conf)
  (compose-transformations (invert robot-conf) conf))

(defun transform-to-world-coords (robot-conf conf)
  (compose-transformations robot-conf conf))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod print-object ((m <simple-cs-mode>) str)
  (print-unreadable-object (m str :type nil :identity nil)
    (format str "Held ~a.  Confs ~a.  Held object conf ~a." (held m) (confs m) (held-object-conf m))))