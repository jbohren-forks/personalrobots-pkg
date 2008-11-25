(in-package prob)

(defclass <uniform> (<continuous-prob-dist>)
  ((l :accessor l :initarg :l)
   (u :accessor u :initarg :u))
  (:documentation "<uniform> (<probability-distribution>)

Initargs
:l - a floating point number
:u - ditto

Represents the uniform distribution over [l,u)."))

(defmethod initialize-instance :after ((d <uniform>) &rest args)
  (declare (ignore args))
  (setf (l d) (coerce (l d) 'float))
  (setf (u d) (coerce (u d) 'float)))

(defmethod sample ((p <uniform>))
  (+ (random (- (u p) (l p))) (l p)))

