(in-package geometry)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; generic operations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric transform (trans s)
  (:documentation "transform TRANSFORMATION POINT-SET.  Return the image of POINT-SET under TRANSFORMATION.")
  (:method (trans (s list)) (mapcar #'(lambda (x) (transform trans x)) s)))

(defgeneric inverse-transform (trans s)
  (:documentation "inverse-transform TRANSFORMATION POINT-SET.  Return the inverse image of POINT-SET under TRANSFORMATION.  See also transform.")
  (:method (trans s) (transform (invert trans) s)))

(defgeneric invert (trans)
  (:documentation "Return the inverse transformation, or nil.  In the second case, there is a second return value that equals ':non-invertible or ':unknown."))

(defgeneric equal-transformations (t1 t2)
  (:documentation "equal-transformations TRANS1 TRANS2.  Are TRANS1 and TRANS2 the same transformation (to within *tol*).  May not always detect equality.")
  (:method (t1 t2) (eq t1 t2)))

(defgeneric compose-transformations (t1 t2)
  (:documentation "compose-transformations TRANS1 TRANS2.  Return a new transformation that applies TRANS2 then TRANS1."))
  

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Identity transform for arbitrary sets
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod transform ((trans (eql #'identity)) s)
  s)

(defmethod invert ((trans (eql #'identity))) trans)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Generic transformations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod transform (trans (s <implicit-union>))
  (apply #'implicit-union (mapcar #'(lambda (s) (transform trans s)) (union-sets s))))

