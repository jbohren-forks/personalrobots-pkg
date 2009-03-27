(roslisp:ros-load-message-types "robot_msgs/PointCloud" "robot_msgs/Point" "deprecated_msgs/Pose2DFloat32" 
				"robot_msgs/Planner2DGoal" "robot_msgs/Planner2DState")


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
(defparameter *wall-buffer* .2 "Additional buffer distance that we'd like to keep from wall")
(defparameter *move-base-timeout-multiplier* 10 "Given a goal D metres away, timeout on move-base after D times this many seconds")
(defparameter *wait-inc* .5)

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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun main ()
  (with-ros-node ("lane_changer")

    ;; 1. setup
    (subscribe "hallway_points" "robot_msgs/PointCloud" #'hallway-callback)
    (subscribe "person_position" "robot_msgs/Point" (store-message-in *person-pos*))
    (subscribe "robot_pose" "deprecated_msgs/Pose2DFloat32" #'pose-callback)
    (subscribe "state" "robot_msgs/Planner2DState" #'state-callback)
    (advertise "goal" "robot_msgs/Planner2DGoal")
    (setq *global-frame* (get-param "global_frame_id"))
    (ros-info "Global frame is ~a with type ~a" *global-frame* (type-of *global-frame*))
    (loop
       (format t "~&Corridor state is ~a" *hallway*)
       (format t "~&Robot pose is ~a" *robot-pose*)
       (format t "~&Enter next goal: ")
       (let ((x (read)))
	 (if (listp x)
	     (move-action (first x) (second x) 0.0)
	     (move-to-right))))))
			 
			 

    
#|
    ;; 2. executive
    (with-highlevel-controller (move-base "state" "robot_msgs/Planner2DState" "goal" "robot_msgs/Planner2DGoal")
      (ecase (controller-result move-base (goal-message *original-goal*))
	(succeeded (print "done!"))
	(aborted 
	 (ecase (controller-result move-base (goal-message (waiting-pose current-pose corridor-width)))
	   (aborted (print "Could not move to waiting position --- failing!"))
	   (succeeded (wait-for-person-to-move)
		      (ecase (controller-result move-base (goal-message *original-goal*))
			(succeeded (print "done!"))
			(aborted (print "Failed even after lane-change."))))))))))

  |#    

(defun start-node ()
  (roslisp:start-ros-node "nav")
  (subscribe "person_position" "robot_msgs/Point" #'print)
  (subscribe "robot_pose" "deprecated_msgs/Pose2DFloat32" #'print)
  (advertise "goal" "robot_msgs/Planner2DGoal"))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Actions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun move-to-right ()
  (move-action (waiting-pose *robot-pose* *hallway*)))

(defun move-to (x y theta)
  (let ((m (make-instance '<Pose2DFloat32>)))
    (setf (deprecated_msgs:x-val m) x
	  (deprecated_msgs:y-val m) y
	  (deprecated_msgs:th-val m) theta)
    (move-action m)))

(defun move-action (m)
  "Encapsulates durative action as a blocking call.  Returns :succeeded or :aborted (preemption is considered an error)"
  (declare (<Pose2DFloat32> m))
  (with-message-fields ((x :x) (y :y) (theta :th)) m
    (with-mutex (*node-lock*)
      (setq *state-changes* nil)
      (send-nav-goal x y theta))
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
      (ros-info "Waiting ~a seconds for move to ~a, ~a, ~a to become inactive" (* max-count *wait-inc*) x y theta)
      (loop-at-most-every *wait-inc*
	 (let ((msg (find-if #'(lambda (state-msg)
				 (and (= (status state-msg) (symbol-code '<ControllerStatus> :success))
				      (refers-to-goal state-msg x y)))
			     *state-changes*)))
	   (when msg
	     (ros-info "Succeeded with message ~a" msg)
	     (return-from move-action :succeeded))
	   (when (> (incf count) max-count)
	     (ros-info "Aborting as timeout exceeded")
	     (disable-nav)
	     (return-from move-action :aborted)))))))
				  

(defun send-nav-goal (x y th)
  (let ((m (make-instance 'robot_msgs:<Planner2DGoal>)))
    (setf (robot_msgs:enable-val m) 1
	  (roslib:frame_id-val (robot_msgs:header-val m)) *global-frame*
	  (deprecated_msgs:x-val (robot_msgs:goal-val m)) x
	  (deprecated_msgs:y-val (robot_msgs:goal-val m)) y
	  (deprecated_msgs:th-val (robot_msgs:goal-val m)) th)
    (publish-on-topic "goal" m)))

(defun disable-nav ()
  (let ((m (make-instance 'robot_msgs:<Planner2DGoal>)))
    (setf (robot_msgs:enable-val m) 0
	  (roslib:frame_id-val (robot_msgs:header-val m)) *global-frame*)
    (publish-on-topic "goal" m)))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Callbacks
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;



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
	 (facing-forward (<= 0.0 (pose-orientation corridor-pose) pi))
	 (target-wall-offset (if facing-forward
				 (- (corridor-width corridor) *robot-radius* *wall-buffer*)
				 (+ *robot-radius* *wall-buffer*)))
	 (global-pose (transform-pose (inverse (corridor-transform corridor)) (make-pose (vector target-wall-offset (aref (pose-position corridor-pose) 1)) (/ pi (if facing-forward 2 -2)))))
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

    
    
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Communication with highlevel controllers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

#|(defmacro with-highlevel-controller ((name state-topic state-type goal-topic goal-type) &body body)
  `(let ((,state-message-queue (make-queue)))
     (subscribe state-topic state-type (add-to-queue ,state-message-queue))
     (advertise goal-topic goal-type)
     ;; not quite right
     (flet ((controller-result (name message)
	      (controller-result message goal-topic state-message-queue)))
       ,@body)))


(defun controller-result (goal-pos topic response-queue)
  (publish topic (make-instance '<Planner2DGoal> :position goal-pos))
  (dequeue-wait response-queue)) 

|#
    
