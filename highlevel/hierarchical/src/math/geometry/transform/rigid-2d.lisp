(in-package geometry)


(defparameter *all-angles* (make-instance '<interval> :a 0 :b *2pi* :left-open nil :right-open t))

(defclass rigid-2d ()
  ((centered-v :accessor centered-v :reader rigid-2d-centered-v)
   (center :initarg :center :initform #(0 0) :reader center)
   (theta :initform nil :writer set-theta :reader rigid-2d-theta)
   (sin-theta :accessor stheta)
   (cos-theta :accessor ctheta)
   (uncentered-v :initform nil :writer set-uncentered-v :reader rigid-2d-v))
  (:documentation "rigid-2d - represents a rigid 2d transformation (x,y,theta) where theta is clockwise rotation.  The rotation is done first, then the translation.

Initargs
:theta - angle
:v - offset
:center - point about which to rotate.  Defaults to #(0 0).

The reader and writer methods (set-)rigid-2d-theta/v are all defined w.r.t. center."))

(defmethod shared-initialize :after ((trans rigid-2d) names &rest args &key theta v)
  (declare (ignore names args))
  (when theta
    (set-theta (mod theta *2pi*) trans))
  (when v
    (set-uncentered-v v trans))
  (recenter trans))

(defmethod set-theta :after (theta (trans rigid-2d))
  (setf (stheta trans) (sin theta)
	(ctheta trans) (cos theta)))


(defun recenter (trans)
  (with-slots (theta uncentered-v center) trans
    (when (and theta uncentered-v)
      (setf (centered-v trans) (new-v trans uncentered-v center #(0 0))))))

(defun new-v (trans old-v old-center new-center)
  (let ((diff (a- new-center old-center)))
    (a+ old-v (a- (cached-rotate trans diff) diff))))

(defun set-rigid-2d-theta (theta trans)
  (set-theta theta trans)
  (recenter trans))

(defun set-rigid-2d-v (v trans)
  (set-uncentered-v v trans)
  (recenter trans))

  
(defun make-rigid-2d (&rest args)
  (apply #'make-instance 'rigid-2d args))


(defun cached-rotate (trans x)
  (with-accessors ((s stheta) (c ctheta)) trans
    (dbind (x0 x1) x
      (vector (+ (* c x0) (* s x1)) (- (* c x1) (* s x0))))))

(defun 2d-rotate (theta x)
  (let ((c (cos theta))
	(s (- (sin theta)))
	(v (make-array 2)))
    (dbind (x0 x1) x
      (setf (aref v 0) (- (* c x0) (* s x1))
	    (aref v 1) (+ (* s x0) (* c x1)))
      v)))


(defmethod invert ((trans rigid-2d))
  (let ((new-theta (- (rigid-2d-theta trans))))
    (make-rigid-2d :center (center trans) :theta new-theta 
		   :v (a- (2d-rotate new-theta (rigid-2d-v trans))))))

  
(defmethod transform ((trans rigid-2d) (x vector))
  (a+ (cached-rotate trans x) (centered-v trans)))

(defmethod equal-transformations ((t1 rigid-2d) (t2 rigid-2d))
  (and (close-to (rigid-2d-v t1) (rigid-2d-v t2))
       (close-to (center t1) (center t2))
       (< (abs-diff (mod (rigid-2d-theta t1) *2pi*)
		    (mod (rigid-2d-theta t2) *2pi*))
	  *tol*)))

(defmethod compose-transformations ((t1 rigid-2d) (t2 rigid-2d))
  (let ((c1 (center t1))
	(th1 (rigid-2d-theta t1))
	(v1 (rigid-2d-v t1))
	(c2 (center t2))
	(th2 (rigid-2d-theta t2))
	(v2 (rigid-2d-v t2)))
    
    (make-rigid-2d
     :theta (+ th1 th2) :center c2
     :v (let ((c (a- c1 c2)))
	  (a+ (cached-rotate t1 (a- v2 c)) v1 c)))))
		 
  



(defmethod print-object ((trans rigid-2d) str)
  (print-unreadable-object (trans str :type t :identity nil)
    (with-slots (theta centered-v) trans
	(format str "(~0,2f, ~0,2f, ~D)" (aref centered-v 0) (aref centered-v 1) (round (* (/ 180 pi) theta))))))


