(in-package :blocks)


(defclass <blocks-descriptions> ()
  ((domain :initarg :domain :reader desc-domain)
   (heuristic :accessor heuristic :initarg :heuristic)))

(defmethod initialize-instance :after ((d <blocks-descriptions>) &rest args)
  (declare (ignore args))
  (setf (heuristic d) (dist-heuristic (desc-domain d))))

(make-simple-descriptions (d <blocks-descriptions>) (a s v) 
  (act () 
       :progress-optimistic (values (goal (desc-domain d)) (funcall (heuristic d) s))
       :progress-pessimistic (values (intersect s (goal (desc-domain d))) 0)))


