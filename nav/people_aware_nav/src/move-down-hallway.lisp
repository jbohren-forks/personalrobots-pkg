(handler-bind
    ((style-warning #'muffle-warning)
     (warning #'(lambda (c) (format t "~&Warning: ~a" c) (muffle-warning))))
  (asdf:operate 'asdf:load-op :people-aware-nav))

(in-package :lane-following)
