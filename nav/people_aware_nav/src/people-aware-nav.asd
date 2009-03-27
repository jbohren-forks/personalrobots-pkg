;;;; -*- Mode: LISP -*-


(in-package :asdf)	

(defsystem :people-aware-nav
  :name "people-aware-nav"
  :components
  ((:file "transform-2d")
   (:file "lanes" :depends-on ("transform-2d")))
  :depends-on (:roslisp))

;;;; eof
