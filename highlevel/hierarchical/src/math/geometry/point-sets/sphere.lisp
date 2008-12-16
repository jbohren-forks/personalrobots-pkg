(in-package geom)

(defclass <sphere> (<point-set>)
  ((centre :initarg :centre :accessor centre)
   (radius :initarg :radius :accessor radius)
   (norm-p :initarg :norm-p :reader norm-p :initform 2))
  (:documentation "Class <sphere> (<point-set>)
Represents a (closed) lp-sphere in R^n

Initargs
:centre
:radius
:norm-p - what L^p norm to use.  Default 2."))

(defmethod member? (x (s <sphere>))
  (<= (l2-dist x (centre s)) (radius s)))

(defmethod min-distance ((s1 <sphere>) (s2 <sphere>))
  (assert (and (= 2 (norm-p s1)) (= 2 (norm-p s2))) nil
    "Unable to compute min-distance ~a and ~a as it is only implemented for 2-norm spheres." s1 s2)
  (max 0 (- (l2-dist (centre s1) (centre s2)) (+ (radius s1) (radius s2)))))

(defmethod bounding-sphere ((s <sphere>))
  s)

(defmethod same-point-set ((s <sphere>) (s2 <sphere>) &optional (tol *tol*))
  (and (close-to (centre s) (centre s2) tol)
       (< (abs-diff (radius s) (radius s2)) tol)))