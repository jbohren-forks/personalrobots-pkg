(in-package :blocks)


(defclass <blocks-descriptions> ()
  ((domain :initarg :domain :reader domain)
   (heuristic :accessor heuristic)))

(defmethod initialize-instance :after ((d <blocks-descriptions>) &rest args)
  (setf (heuristic d) (dist-heuristic (domain d))))

(make-simple-descriptions (d <blocks-descriptions>) (a s v)

  (act () 
       :progress-optimistic (values (goal (domain d)) (funcall (heuristic d) s))))








