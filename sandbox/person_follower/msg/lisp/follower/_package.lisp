(defpackage follower-msg
  (:use cl
        roslisp)
  (:export
   "<MOVEHEADGOAL>"
   "<WAITACTIONSTATE>"
   "<WAITACTIONGOAL>"
   "<MOVEHEADSTATE>"
  ))

(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package_MoveHeadGoal.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package_WaitActionState.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package_WaitActionGoal.lisp")
(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package_MoveHeadState.lisp")
