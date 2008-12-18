(asdf:defsystem "ros-hplan"
  :depends-on ("roslisp" "hierarchical-planning")
  :components
  ((:module "ros-env" :pathname "ros/"
	    :components ((:file "ros-env")))))





;; Local variables:
;; mode:lisp
;; outline-regexp:"\\s-*..module"
;; End:
