;;;; -*- Mode: LISP -*-


(in-package :asdf)

(defsystem "visual_nav/visual-nav"
  :name "visual-nav"

  :components
  ((:file "teleop-exec"))

  :depends-on (:roslisp :visual_nav-msg))

;;;; eof
