(roslisp:ros-load-message-types "robot_msgs/PointCloud" "robot_msgs/Point" "deprecated_msgs/Pose2DFloat32" 
				"robot_actions/Pose2D" "robot_actions/MoveBaseState" "deprecated_msgs/Point2DFloat32")
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
(defparameter *path-clear-wait-time* 6 "How many seconds robot will wait for person to move")
(defvar *global-frame*)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (hallway (:constructor make-hallway (transform width)) (:type list))
  transform width)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; State
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *node-lock* (make-mutex :name "lane changer lock"))
(defvar *robot-pose* nil "Robot's current 3d pose.  Set by pose-callback.")
(defvar *hallway* (make-hallway nil nil) "Configuration of the hallway we're in.  Set by hallway-callback.")
(defvar *move-base-result* nil "Did move-base succeed?.  Set by send-move-goal and state-callback.")
(defvar *person-on-path* nil "Is a person on the path?.") ;; Maybe get rid of this?
(defvar *current-goal* '(0 0) "Current nav goal.  Set by send-move-goal.")

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Main
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun main ()
  (with-ros-node ("lane_changer")

    ;; 1. setup
    (setup-node)
    (loop
       (format t "~&Hallway state is ~a" *hallway*)
       (format t "~&Robot pose is ~a" *robot-pose*)
       (format t "~&Enter next goal: ")
       (let ((x (read)))
	 (format t "~&Result is: ~a"
		 (if (listp x)
		     (goto (first x) (second x) (or (third x) 0.0))
		     (move-to-right)))))))


(defun setup-node ()
  (subscribe "hallway_points" "robot_msgs/PointCloud" #'hallway-callback)
  (subscribe "robot_pose" "deprecated_msgs/Pose2DFloat32" #'pose-callback)
  (subscribe "/move_base_node/feedback" "robot_actions/MoveBaseState" #'state-callback)
  (advertise "/move_base_node/activate" "robot_actions/Pose2D")
  (advertise "/move_base_node/preempt" "robot_actions/Pose2D")
  (setq *global-frame* (get-param "global_frame_id")))

			 
(defun goto (x y theta)
  (unwind-protect 
       (progn
	 (send-move-goal x y theta)
	 (loop-at-most-every 1
	      (format t ".")
	      (when *move-base-result* 
		(return-from goto *move-base-result*))
	      (when (person-on-path)
		(return)))

	 ;; Person on path
	 (move-to-right)
	 (constrained-move x y theta)
	 (loop-at-most-every .1
	    (when *move-base-result*
	      (return *move-base-result*))))
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

(defun constrained-move (x y theta)
  "Move to x, y, theta, with a timeout parameter set, and with a constraint not to move backwards.  Backwards is defined by drawing a line behind the robot, perpendicular to its current heading."
  (ros-info "Initiating constrained move")
  (setq *current-goal* (list x y)
	*move-base-result* nil)
  (publish-on-topic "/move_base_node/activate"
		    (make-instance 'robot_actions:<Pose2D>
				   :header (make-instance 'roslib:<Header> :frame_id *global-frame*)
				   :x x :y y :th theta
				   :boundary (line-behind *robot-pose*))))

(defun send-move-goal (x y th)
  (setq *current-goal* (list x y)
	*move-base-result* nil)
  (publish-on-topic "/move_base_node/activate" 
		    (make-instance 'robot_actions:<Pose2D>
				   :header (make-instance 'roslib:<Header> :frame_id *global-frame*)
				   :x x :y y :th th)))

(defun disable-nav ()
  (ros-info "Disabling nav")
  (let ((m (make-instance 'robot_actions:<Pose2D>)))
    (setf (roslib:frame_id-val (robot_actions:header-val m)) *global-frame*)
    (publish-on-topic "/move_base_node/preempt" m)))

(defun person-on-path ()
  (let ((v (not (= 0 (people_aware_nav:value-val (call-service "is_person_on_path" 'people_aware_nav:PersonOnPath))))))
    (when v (format t "~&Person is on path at time ~a" (ros-time)))
    v))

(defun spin-around ()
  (send-move-goal (aref (pose-position *robot-pose*) 0) (aref (pose-position *robot-pose*) 1)
		  (+ 3.14 (pose-orientation *robot-pose*))))


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
			   (* (if facing-forward 3 -3)
			      (abs (- target-wall-offset (aref hallway-frame-position 0))))))
	 (global-pose (transform-pose (inverse (hallway-transform hallway)) 
				      (make-pose (vector target-wall-offset target-wall-y) 
						 (/ pi (if facing-forward 2 -2))))))
    (ros-info "Waiting pose is ~a" global-pose)
    global-pose))

    
(defun make-point (p)
  (declare (robot_msgs:<Point32> p) (values point))
  (let ((a (make-array 2 :element-type 'float :initial-element 0.0)))
    (setf (aref a 0) (x-val p) (aref a 1) (y-val p))
    a))


(defparameter *angle-offset* (+ (/ pi 2) .19))

(defun line-behind (pose)
  (let ((pos (pose-position pose))
	(theta (pose-orientation pose)))
    (let ((th1 (+ theta *angle-offset*))
	  (th2 (+ theta (- *angle-offset*))))
      (make-instance 'robot_msgs:<Polyline2D>
		     :points (vector (get-offset-point pos th1) (get-offset-point pos th2))))))

(defparameter *offset-length* 5)

(defun get-offset-point (pos theta)
  (let ((x (aref pos 0))
	(y (aref pos 1)))
    (make-instance 'deprecated_msgs:<Point2DFloat32> :x (+ x (* *offset-length* (cos theta)))
		   :y (+ y (* *offset-length* (sin theta))))))