(in-package geometry)

(defun point-set (l)
  "point-set L.  A language for conveniently representing point sets.  Currently supports lists of the form (polygon v1 ... vn) and of the form (union ps1 ... ps_n). If L is already a <point-set> or <implicit-union>, it is just returned."
  (if (typep l '(or <point-set> <implicit-union>))
      l
    (dbind (obj-type &rest args) l
      (ecase obj-type
	(polygon (make-instance '<polygon> :unordered-vertices args))
	(line-segment (make-instance '<line-segment> :a (coerce (first args) 'vector) :b (coerce (second args) 'vector)))
	(union (apply #'implicit-union (mapcar #'point-set args)))))))
  


(defparameter *deg-rad-mult* (/ pi 180))
(defun transformation (l)
  "transformation L.  Language for conveniently representing transformations.  Supports lists of the form (rigid-2d X Y THETA) where THETA is in *degrees*.  Also, if L is of type RIGID-2D it is just returned."
  (if (typep l 'rigid-2d)
      l
    (dbind (trans-type &rest args) l
      (ecase trans-type
	(rigid-2d (make-instance 'rigid-2d :v (let ((a (make-array 2 :element-type 'float :initial-element 0.0)))
						(setf (aref a 0) (first args)
						      (aref a 1) (second args))
						a)
				 :theta (* (third args) *deg-rad-mult*)))))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Set up readtable
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvar *geometry-readtable* (copy-readtable nil))

(define-read-dispatch #\G *geometry-readtable* #'point-set)
(define-read-dispatch #\T *geometry-readtable* #'transformation)
(define-read-dispatch #\P *geometry-readtable* #'(lambda (x) (point-set `(polygon ,@x))))
(define-read-dispatch #\L *geometry-readtable* #'(lambda (x) (point-set `(line-segment ,@x))))
(define-read-dispatch #\R *geometry-readtable* #'(lambda (x) (transformation `(rigid-2d ,@x))))
