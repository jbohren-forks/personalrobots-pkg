(in-package geometry)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Composition
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric compose-transformation-sets (s1 s2)
  (:documentation "Return set of transformations obtained by composing a member of S1 with a member of S2.  S1 and/or S2 may also be single transformations."))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Transformations-into
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric transformations-into (s region type &optional bound-type)
  (:documentation "Return set of transformations mapping point-set S *into* point set REGION.

BOUND-TYPE can be ':inner/':inner-bound, ':outer/':outer-bound, or ':exact denoting the kinds of bounds allowed (since this operation is rarely possible to do exactly).  Default is ':inner.")
  (:method (s region type &optional (bound-type ':inner))
    (funcall
     (ecase bound-type
       ((:inner :inner-bound) #'transformations-into-inner-bound)
       ((:outer :outer-bound) #'transformations-into-outer-bound)
       ((:exact) #'transformations-into-exact))
     s region type)))

(defgeneric transformations-into-inner-bound (s r type)
  (:documentation "transformations-into-inner-bound S REGION TYPE
Return an inner bound on the set of transformations of type TYPE that map S into REGION."))

(defgeneric transformations-into-outer-bound (s r type)
  (:documentation "transformations-into-outer-bound S REGION TYPE
Return an outer bound on the set of transformations of type TYPE that map S into REGION."))

(defclass <transformations-into> (<filtered-set>)
  ((s :initarg :s :reader s)
   (region :initarg :region :reader region)
   (type :initarg :type :reader trans-type)))

(defgeneric transformations-into-exact (s r type)
  (:documentation "transformations-into-outer-bound S REGION TYPE
Return exactly the set of transformations of type TYPE that map S into REGION.")
  (:method (s r type) (make-instance '<transformations-into> 
				     :base-set (transformations-into-outer-bound s r type)
				     :predicate #'(lambda (tr) (subset (transform tr s) r))
				     :s s :region r :type type)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Transformations intersecting
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric transformations-intersecting (s r type)
  (:documentation "transformations-intersecting S REGION TYPE

S is a <point-set>.
REGION is a <point-set>.
TYPE is a symbol naming a transformation type.

Return (an outer bound on) the set of transformations of type TYPE that map S into a set intersecting REGION."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Swept regions
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric swept-region (confs s)
  (:documentation "swept-region TRANSFORMATIONS POINT-SET

Returns an outer bound on the set of points obtained by applying a member of TRANSFORMATIONS to a member of POINT-SET.")
  (:method ((confs <transformations-into>) s)
	   (if (same-point-set s (s confs))
	       (region confs)
	     (call-next-method))))




