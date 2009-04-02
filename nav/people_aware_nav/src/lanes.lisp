(roslisp:ros-load-message-types "robot_msgs/PointCloud" "robot_msgs/Point" "deprecated_msgs/Pose2DFloat32" 
				"robot_actions/Pose2D" "robot_actions/MoveBaseState")


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
		:goal-val
		:<PointCloud>
		:pts-val)
  (:import-from :deprecated_msgs
		:<Pose2DFloat32>))


(in-package :lane-following)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Params
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defparameter *robot-radius* .32 "Circumscribed radius of robot in metres")
(defparameter *wall-buffer* .1 "Additional buffer distance that we'd like to keep from wall")
(defparameter *move-base-timeout-multiplier* 10 "Given a goal D metres away, timeout on move-base after D times this many seconds")
(defparameter *wait-inc* .5)
(defparameter *path-clear-wait-time* 6)
(defparameter *person-timeout-threshold* 4)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (corridor (:constructor make-corridor (transform width)) (:type list))
  transform width)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *state-changes* nil)
(defvar *last-state* nil)
(defvar *node-lock* (make-mutex :name "lane changer lock"))
(defvar *robot-pose* nil)
(defvar *person-pos* nil)
(defvar *hallway* (make-corridor nil nil))
(defvar *global-frame*)
(defvar *last-time-person-seen* 0.0)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun main ()
  (with-ros-node ("lane_changer")

    ;; 1. setup
    (setup-node)
    (loop
       (format t "~&Corridor state is ~a" *hallway*)
       (format t "~&Robot pose is ~a" *robot-pose*)
       (format t "~&Enter next goal: ")
       (let ((x (read)))
	 (if (listp x)
	     (goto (first x) (second x) (or (third x) 0.0))
	     (move-to-right))))))

(defun setup-node ()
  (subscribe "hallway_points" "robot_msgs/PointCloud" #'hallway-callback)
  (subscribe "person_position" "robot_msgs/Point" #'person-callback)
  (subscribe "robot_pose" "deprecated_msgs/Pose2DFloat32" #'pose-callback)
  (subscribe "state" "robot_actions/MoveBaseState" #'state-callback)
  (advertise "goal" "robot_actions/Pose2D")
  (setq *global-frame* (get-param "global_frame_id"))
  (ros-info "Global frame is ~a with type ~a" *global-frame* (type-of *global-frame*)))

			 
(defun goto (x y theta)
  (unless (eq :succeeded (move-to x y theta))
    (if (< (time-since-person-seen) *person-timeout-threshold*)
	(wait-then-move x y theta)
	(ros-info "Path failed, and person has not been seen recently"))))
    


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun wait-then-move (x y theta)
  (move-to-right)
  (ros-info "Waiting ~a seconds for path to become clear" *path-clear-wait-time*)
  (sleep *path-clear-wait-time*)
  (move-to x y theta 0))

(defun move-to-right ()
  (ros-info "Shifting to right lane")
  (move-action (waiting-pose *robot-pose* *hallway*)))

(defun move-to (x y theta &optional (timeout 1))
  (let ((m (make-instance '<Pose2DFloat32>)))
    (setf (deprecated_msgs:x-val m) x
	  (deprecated_msgs:y-val m) y
	  (deprecated_msgs:th-val m) theta)
    (move-action m timeout)))




(defun move-action (m &optional (timeout 1))
  "Encapsulates durative action as a blocking call.  Returns :succeeded or :aborted (preemption is considered an error)"
  (declare (<Pose2DFloat32> m))
  (with-message-fields ((x :x) (y :y) (theta :th)) m
    (with-mutex (*node-lock*)
      (setq *state-changes* nil)
      (send-nav-goal x y theta timeout))
    (ros-info "Waiting for move to ~a, ~a, ~a to become active" x y theta)
    (let ((count 0))
      (loop-at-most-every *wait-inc*
	 (when (member-if #'(lambda (state-msg)
			      (and (= (status state-msg) (symbol-code '<ControllerStatus> :active))
				   (refers-to-goal state-msg x y)))
			  *state-changes*)
	   (return))
	 (when (> (incf count) (/ 2 *wait-inc*))
	   (ros-info "Aborting as move-base did not become active in time")
	   (return-from move-action))))
    (let ((count 0)
	  (max-count (timeout x y *wait-inc*)))
      (ros-info "Waiting at most ~a seconds for move to ~a, ~a, ~a to become inactive" (* max-count *wait-inc*) x y theta)
      (loop-at-most-every *wait-inc*
	 (let ((msg (find-if #'(lambda (state-msg)
				 (and (or (succeeded state-msg) (aborted state-msg))
				      (refers-to-goal state-msg x y)))
			     *state-changes*)))
	   (when msg
	     (cond
	       ((succeeded msg)
		(ros-info "Succeeded with message ~a" msg)
		(return-from move-action :succeeded))
	       ((aborted msg)
		(ros-info "Plan could not be found for move; aborting")
		(return-from move-action :aborted))))
	   (when (> (incf count) max-count)
	     (ros-info "Aborting as timeout exceeded.  This shouldn't have been reached due to timeout to goal!")
	     (disable-nav)
	     (return-from move-action :aborted)))))))
				  

(defun send-nav-goal (x y th timeout)
  (let ((m (make-instance 'robot_actions:<Pose2D>)))
    (setf (roslib:frame_id-val (robot_actions:header-val m)) *global-frame*
	  (robot_actions:x-val m) x
	  (robot_actions:y-val m) y
	  (robot_actions:th-val m) th)
    (publish-on-topic "/move_base_node/activate" m)))

(defun disable-nav ()
  (let ((m (make-instance 'robot_actions:<Pose2D>)))
    (setf (roslib:frame_id-val (robot_actions:header-val m)) *global-frame*)
    (publish-on-topic "/move_base_node/preempt" m)))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callbacks
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; We don't actually look at the message - just keep track of when we saw them
(defun person-callback (m)
  (declare (ignore m))
  (setq *last-time-person-seen* (ros-time)))



(defun state-callback (state)
  (with-mutex (*node-lock*)
    (when (not (and *last-state* (equal (status *last-state*) (status state))))
      (push state *state-changes*))
    (setq *last-state* state)))
  

(defun hallway-callback (m)
  (declare (<PointCloud> m))
  (let ((points (pts-val m)))
    (if (= (length points) 3)

	(setf *hallway* (corridor-info (make-point (aref points 0))
				       (make-point (aref points 1))
				       (make-point (aref points 2))))

	(ros-error "Hallway cloud ~a had incorrect length.  Skipping." points))))



(defun pose-callback (pose)
  (declare (<Pose2DFloat32> pose) (values pose))
  (with-message-fields ((x :x) (y :y) (theta :th)) pose
    (setq *robot-pose* (make-pose (vector x y) theta))))







;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun time-since-person-seen ()
  (- (ros-time) *last-time-person-seen*))



(defun timeout (x y inc)
  (let ((x2 (aref (pose-position *robot-pose*) 0))
	(y2 (aref (pose-position *robot-pose*) 1)))
    (/ (* *move-base-timeout-multiplier* (+ (abs (- x x2)) (abs (- y y2)))) inc)))


(defun corridor-info (p1 p2 p3)
  "Return 1) the transform from map to corridor frames 2) the corridor width, given that p1 and p2 are points on the left wall, and p3 is on the right wall"
  (declare (point p1) (point p2) (point p3) (values rigid-transformation real))
  (let* ((d1 (a- p2 p1))
	 (d2 (a- p3 p1))
	 (length (inner-product d2 (unit-vector (mv* (rotation-matrix (/ pi -2)) d1))))
	 (flip (< length 0)))
    (make-corridor (transform-between (make-pose p1 (vector-angle d1)) (make-pose (vector 0.0 0.0) (/ pi (if flip -2 2)))) (abs length))))


(defun waiting-pose (current-pose corridor)
  "Return the pose corresponding to 'shifting to the right lane' in the map frame"
  (declare (pose current-pose) (values <Pose2DFloat32>))
  (let* ((corridor-pose (transform-pose (corridor-transform corridor) current-pose))
	 (corridor-frame-position (pose-position corridor-pose))
	 (facing-forward (<= 0.0 (pose-orientation corridor-pose) pi))
	 (target-wall-offset (if facing-forward
				 (- (corridor-width corridor) *robot-radius* *wall-buffer*)
				 (+ *robot-radius* *wall-buffer*)))
	 (target-wall-y (+ (aref corridor-frame-position 1)
			   (* (if facing-forward 1 -1)
			      (abs (- target-wall-offset (aref corridor-frame-position 0))))))
	 (global-pose (transform-pose (inverse (corridor-transform corridor)) 
				      (make-pose (vector target-wall-offset target-wall-y) 
						 (/ pi (if facing-forward 2 -2)))))
	 (m (make-instance '<Pose2DFloat32>)))
    (setf (deprecated_msgs:x-val m) (aref (pose-position global-pose) 0)
	  (deprecated_msgs:y-val m) (aref (pose-position global-pose) 1)
	  (deprecated_msgs:th-val m) (pose-orientation global-pose))
    m))

    
(defun make-point (p)
  (declare (robot_msgs:<Point32> p) (values point))
  (let ((a (make-array 2 :element-type 'float :initial-element 0.0)))
    (setf (aref a 0) (x-val p) (aref a 1) (y-val p))
    a))


(defun refers-to-goal (msg x y)
  (and (< (abs (- (deprecated_msgs:x-val (goal-val msg)) x)) .1)
       (< (abs (- (deprecated_msgs:y-val (goal-val msg)) y)) .1)))

(defun status (m)
  (robot_msgs:value-val (robot_msgs:status-val m)))


(defun aborted (m)
  (eq (status m) (symbol-code '<ControllerStatus> :aborted)))

(defun succeeded (m)
  (eq (status m) (symbol-code '<ControllerStatus> :success)))
    
