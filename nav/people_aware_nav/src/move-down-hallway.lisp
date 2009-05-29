(handler-bind
    ((style-warning #'muffle-warning)
     (warning #'muffle-warning))
  (load (merge-pathnames "transform-2d.lisp" *load-pathname*) :verbose t)
  (load (merge-pathnames "lanes.lisp" *load-pathname*) :verbose t))

(in-package :lane-following)

(set-debug-level '(roslisp) :info)
(set-debug-level '(pan) :info)
