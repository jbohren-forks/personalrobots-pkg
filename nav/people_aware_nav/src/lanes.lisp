(roslisp:ros-load-message-types "robot_msgs/PointCloud" "robot_msgs/Point" "deprecated_msgs/Pose2DFloat32" 
				"robot_msgs/PoseStamped" "std_msgs/Empty"
				"people_aware_nav/ConstrainedGoal" "people_aware_nav/ConstrainedMoveBaseState"
				)
(roslisp:ros-load-service-types "people_aware_nav/PersonOnPath")

(defpackage :lane-following
  (:nicknames :lanes)
  (:use :roslisp :cl :transform-2d :sb-thread)
  (:export :main)
  (:import-from :robot_msgs
		:<Point>
		:<ControllerStatus>
		:x-val
		:y-val
		:z-val
		:<Point32>
		:<Polygon3D>
		:<PointCloud>
		:pts-val)
  (:import-from :deprecated_msgs
		:<Pose2DFloat32>)
  (:import-from :roslib
		:<Header>))


(in-package :lane-following)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Params
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *robot-radius* .32 "Circumscribed radius of robot in metres")
(defparameter *wall-buffer* .2 "Additional buffer distance that we'd like to keep from wall")
(defvar *person-on-path-use-stub* nil)
(defvar *global-frame*)
(defvar *master-uri* (make-uri "prg2" 11311))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (hallway (:constructor make-hallway (transform width)) (:type list))
  transform width)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *robot-pose* nil "Robot's current 3d pose.  Set by pose-callback.")
(defvar *hallway* (make-hallway nil nil) "Configuration of the hallway we're in.  Set by hallway-callback.")
(defvar *move-base-result* nil "Did move-base succeed?.  Set by send-move-goal and state-callback.")
(defvar *current-goal* '(0 0) "Current nav goal.  Set by send-move-goal.")
(defvar *new-goal* nil "A new nav goal received on the goal topic.  Set by goal-callback and (to nil) by main")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun main ()
  (with-ros-node ("lane_changer" :master-uri *master-uri*)
    (setup-node)
    (loop-at-most-every 1
	 (when *new-goal* (apply #'goto *new-goal*)))))


(defun setup-node ()
  (subscribe "hallway_points" "robot_msgs/PointCloud" #'hallway-callback)
  (subscribe "robot_pose" "deprecated_msgs/Pose2DFloat32" #'pose-callback)
  (subscribe "move_base_node/feedback" "people_aware_nav/ConstrainedMoveBaseState" #'state-callback)
  (subscribe "goal" "robot_msgs/PoseStamped" #'goal-callback)
  (advertise "move_base_node/activate" "people_aware_nav/ConstrainedGoal")
  (advertise "move_base_node/preempt" "std_msgs/Empty")
  (setq *global-frame* (get-param "global_frame_id" "map")
	*person-on-path-use-stub* (get-param "~person_on_path_use_stub" *person-on-path-use-stub*)))

(defun goto (x y theta)
  (ros-info "Initiating hallway move to goal ~a ~a ~a" x y theta)
  (setq *new-goal* nil *move-base-result* nil)
  (unwind-protect
       (progn
	 ;; Initial move
	 (send-move-goal x y theta)
	 (loop-at-most-every 1
	      (cond
		(*move-base-result* (return-from goto *move-base-result*))
		(*new-goal* (return-from goto :preempted))
		((person-on-path) (return))))

	 ;; If person is on path
	 (move-to-right)
	 (send-move-goal x y theta :constrained t)
	 (loop-at-most-every 1
	      (cond
		(*move-base-result* (return-from goto *move-base-result*))
		(*new-goal* (return-from goto :preempted)))))

    ;; Cleanup: always disable nav before exiting
    (disable-nav)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callouts
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



(defun move-to-right ()
  (ros-info "Moving to right")
  (let* ((pose (waiting-pose *robot-pose* *hallway*))
	 (position (pose-position pose))
	 (theta (pose-orientation pose)))
    (send-move-goal (aref position 0) (aref position 1) theta))
  (loop-at-most-every .1
     (when *move-base-result* 
       (return *move-base-result*))))


(defun send-move-goal (x y th &key (constrained nil))
  (setq *current-goal* (list x y)
	*move-base-result* nil)
  (publish-on-topic "/move_base_node/activate" 
		    (make-instance 'people_aware_nav:<ConstrainedGoal>
				   :header (make-instance '<Header> :frame_id *global-frame*)
				   :x x :y y :th th :forbidden (make-boundary *robot-pose* constrained))))

(defun disable-nav ()
  (publish-on-topic "/move_base_node/preempt" (make-instance 'std_msgs:<Empty>))
  (ros-info "Disabling nav"))


(defun person-on-path ()
  (if *person-on-path-use-stub*
      (y-or-n-p "Is person on path?")
      (let ((v (not (= 0 (people_aware_nav:value-val (call-service "is_person_on_path" 'people_aware_nav:PersonOnPath))))))
	(when v (format t "~&Person is on path at time ~a" (ros-time)))
	v)))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callbacks
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *abort* (symbol-code 'robot_actions:<ActionStatus> :aborted))
(defparameter *succeed* (symbol-code 'robot_actions:<ActionStatus> :success))

;; When state msg refers to current goal, set *move-base-result*
(defun state-callback (state)
  (when (and (not *move-base-result*) (refers-to-goal state *current-goal*))
    (with-fields ((status (value status))) state
      (setq *move-base-result*
	    (cond
	      ((= status *succeed*) :success)
	      ((= status *abort*) :aborted)
	      (t *move-base-result*))))))
  

(defun hallway-callback (m)
  (declare (<PointCloud> m))
  (let ((points (pts-val m)))
    (if (= (length points) 3)

	(setf *hallway* (hallway-info (make-point (aref points 0))
				      (make-point (aref points 1))
				      (make-point (aref points 2))))

	(ros-error "Hallway cloud ~a had incorrect length.  Skipping." points))))


(defun goal-callback (m)
  (with-fields ((frame (frame_id header)) 
		(x (x position pose)) 
		(y (y position pose))
		(w (w orientation pose))) m
    (let ((theta (* 2 (acos w))))
      (if (equal frame *global-frame*)
	  (setq *new-goal* (list x y theta)) 
	  (ros-error "Ignoring goal ~a ~a ~a as frame id ~a does not equal ~a"
		     x y theta frame *global-frame*)))))
  

(defun pose-callback (pose)
  (declare (<Pose2DFloat32> pose) (values pose))
  (with-fields ((x :x) (y :y) (theta :th)) pose
    (setq *robot-pose* (make-pose (vector x y) theta))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun refers-to-goal (msg current-goal)
  (with-fields (goal) msg
    (with-fields (x y) goal
      (< (max (abs (- x (first current-goal))) (abs (- y (second current-goal)))) .1))))


(defun hallway-info (p1 p2 p3)
  "Return 1) the transform from map to hallway frames 2) the hallway width, given that p1 and p2 are points on the left wall, and p3 is on the right wall"
  (declare (point p1) (point p2) (point p3) (values rigid-transformation real))
  (let* ((d1 (a- p2 p1))
	 (d2 (a- p3 p1))
	 (length (inner-product d2 (unit-vector (mv* (rotation-matrix (/ pi -2)) d1))))
	 (flip (< length 0)))
    (make-hallway (transform-between (make-pose p1 (vector-angle d1)) (make-pose (vector 0.0 0.0) (/ pi (if flip -2 2)))) (abs length))))


(defun waiting-pose (current-pose hallway)
  "Return the pose corresponding to 'shifting to the right lane' in the map frame"
  (declare (pose current-pose) (values pose))
  (ros-info "Computing waiting pose given current pose ~a" current-pose)
  (let* ((hallway-pose (transform-pose (hallway-transform hallway) current-pose))
	 (hallway-frame-position (pose-position hallway-pose))
	 (facing-forward (<= 0.0 (pose-orientation hallway-pose) pi))
	 (target-wall-offset (if facing-forward
				 (- (hallway-width hallway) *robot-radius* *wall-buffer*)
				 (+ *robot-radius* *wall-buffer*)))
	 (target-wall-y (+ (aref hallway-frame-position 1)
			   (* (if facing-forward 4 -4)
			      (abs (- target-wall-offset (aref hallway-frame-position 0))))))
	 (global-pose (transform-pose (inverse (hallway-transform hallway)) 
				      (make-pose (vector target-wall-offset target-wall-y) 
						 (/ pi (if facing-forward 2 -2))))))
    (ros-info "Waiting pose is ~a" global-pose)
    global-pose))

    
(defun make-point (p)
  (declare (<Point32> p) (values point))
  (let ((a (make-array 2 :element-type 'float :initial-element 0.0)))
    (setf (aref a 0) (x-val p) (aref a 1) (y-val p))
    a))


(defparameter *angle-offset* (+ (/ pi 2) .19))
(defparameter *offset-length* 2)

(defun make-boundary (pose constrained?)
  (if constrained?
      (let ((pos (pose-position pose))
	    (theta (pose-orientation pose)))
	(make-instance '<Polygon3D>
		       :points (vector (get-offset-point pos (+ theta *angle-offset*))
				       (get-offset-point pos (+ theta pi))
				       (get-offset-point pos (- theta *angle-offset*)))))
      ;; if not constrained, return an empty polygon
      (make-instance '<Polygon3D>)))



(defun get-offset-point (pos theta)
  (let ((x (aref pos 0))
	(y (aref pos 1)))
    (make-instance '<Point32> :x (+ x (* *offset-length* (cos theta)))
		   :y (+ y (* *offset-length* (sin theta))))))