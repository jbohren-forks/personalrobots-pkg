;;;; -*- Mode: LISP -*-


(in-package :asdf)	

(defsystem "people_aware_nav/people-aware-nav"
  :name "people-aware-nav"
  :components
  ((:file "transform-2d")
   (:file "lanes" :depends-on ("transform-2d")))
  :depends-on (:roslisp :roslib-msg :robot_msgs-msg :deprecated_msgs-msg :people_aware_nav-msg :people_aware_nav-srv :roslisp-utils))

;;;; eof
