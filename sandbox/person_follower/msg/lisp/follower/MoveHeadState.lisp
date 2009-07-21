; Auto-generated. Do not edit!

(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros/core/roslib/msg/lisp/roslib/Header.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/stacks/common/robot_actions/msg/lisp/robot_actions/ActionStatus.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/MoveHeadGoal.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros/std_msgs/msg/lisp/std_msgs/Empty.lisp")

(in-package follower-msg)


;//! \htmlinclude MoveHeadState.msg.html

(defclass <MoveHeadState> (ros-message)
  ((header
    :accessor header-val
    :initarg :header
    :initform (make-instance 'roslib-msg:<Header>))
   (status
    :accessor status-val
    :initarg :status
    :initform (make-instance 'robot_actions-msg:<ActionStatus>))
   (goal
    :accessor goal-val
    :initarg :goal
    :initform (make-instance 'follower-msg:<MoveHeadGoal>))
   (feedback
    :accessor feedback-val
    :initarg :feedback
    :initform (make-instance 'std_msgs-msg:<Empty>)))
)
(defmethod serialize ((msg <MoveHeadState>) ostream)
  "Serializes a message object of type '<MoveHeadState>"
  (serialize (slot-value msg 'header) ostream)
  (serialize (slot-value msg 'status) ostream)
  (serialize (slot-value msg 'goal) ostream)
  (serialize (slot-value msg 'feedback) ostream)
)
(defmethod deserialize ((msg <MoveHeadState>) istream)
  "Deserializes a message object of type '<MoveHeadState>"
  (deserialize (slot-value msg 'header) istream)
  (deserialize (slot-value msg 'status) istream)
  (deserialize (slot-value msg 'goal) istream)
  (deserialize (slot-value msg 'feedback) istream)
  msg
)
(defmethod ros-datatype ((msg (eql '<MoveHeadState>)))
  "Returns string type for a message object of type '<MoveHeadState>"
  "follower/MoveHeadState")
(defmethod md5sum ((type (eql '<MoveHeadState>)))
  "Returns md5sum for a message object of type '<MoveHeadState>"
  #x42f188c09f43b7b9c1dfd7b8285de0d7)
(defmethod message-definition ((type (eql '<MoveHeadState>)))
  "Returns full string definition for message of type '<MoveHeadState>"
  (format nil "Header header~%robot_actions/ActionStatus status~%follower/MoveHeadGoal goal~%std_msgs/Empty feedback~%================================================================================~%MSG: roslib/Header~%#Standard metadata for higher-level flow data types~%#sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: robot_actions/ActionStatus~%# This message defines the expected format for robot action status messages~%# Embed this in the feedback state message of robot actions~%~%# The action is inactive, and has just been reset.~%byte RESET=0~%~%# The action has successfuly completed and is now inactive~%byte SUCCESS=1~%~%# The action has failed and given up. It is now inactive~%byte ABORTED=2~%~%# The action has been preempted. It is now inactive~%byte PREEMPTED=3~%~%# The action is active to accomplish a requested goal~%byte ACTIVE=4~%~%# Status of the controller = {UNDEFINED, SUCCESS, ABORTED, PREEMPTED, ACTIVE}~%byte value~%~%#Comment for debug~%string comment~%================================================================================~%MSG: follower/MoveHeadGoal~%~%================================================================================~%MSG: std_msgs/Empty~%~%~%~%"))
(defmethod serialization-length ((msg <MoveHeadState>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     (serialization-length (slot-value msg 'status))
     (serialization-length (slot-value msg 'goal))
     (serialization-length (slot-value msg 'feedback))
))
(defmethod ros-message-to-list ((msg <MoveHeadState>))
  "Converts a ROS message object to a list"
  (list '<MoveHeadState>
    (cons ':header (ros-message-to-list (header-val msg)))
    (cons ':status (ros-message-to-list (status-val msg)))
    (cons ':goal (ros-message-to-list (goal-val msg)))
    (cons ':feedback (ros-message-to-list (feedback-val msg)))
))
