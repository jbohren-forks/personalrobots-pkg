(handler-bind
    ((style-warning #'muffle-warning)
     (warning #'(lambda (c) #|(format t "~&Warning: ~a" c)|# (muffle-warning))))
  (load (merge-pathnames "transform-2d.lisp" *load-pathname*) :verbose t)
  (load (merge-pathnames "lanes.lisp" *load-pathname*) :verbose t))

(in-package :lane-following)
