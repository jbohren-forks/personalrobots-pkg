; Auto-generated. Do not edit!

(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package.lisp")

(in-package follower)


;//! \htmlinclude MoveHeadGoal.msg.html

(defclass <MoveHeadGoal> (ros-message)
  ()
)
(defmethod serialize ((msg <MoveHeadGoal>) ostream)
  "Serializes a message object of type '<MoveHeadGoal>"
)
(defmethod deserialize ((msg <MoveHeadGoal>) istream)
  "Deserializes a message object of type '<MoveHeadGoal>"
  msg
)
(defmethod ros-datatype ((msg (eql '<MoveHeadGoal>)))
  "Returns string type for a message object of type '<MoveHeadGoal>"
  "follower/MoveHeadGoal")
(defmethod md5sum ((type (eql '<MoveHeadGoal>)))
  "Returns md5sum for a message object of type '<MoveHeadGoal>"
  #xd41d8cd98f00b204e9800998ecf8427e)
(defmethod message-definition ((type (eql '<MoveHeadGoal>)))
  "Returns full string definition for message of type '<MoveHeadGoal>"
  (format nil "~%~%~%"))
(defmethod serialization-length ((msg <MoveHeadGoal>))
  (+ 0
))
(defmethod ros-message-to-list ((msg <MoveHeadGoal>))
  "Converts a ROS message object to a list"
  (list '<MoveHeadGoal>
))
