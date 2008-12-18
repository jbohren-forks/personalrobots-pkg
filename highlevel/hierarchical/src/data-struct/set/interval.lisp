(in-package set)


(defclass <interval> (<set>)
  ((a :accessor left-bound :initarg :a)
   (b :accessor right-bound :initarg :b)
   (left-open :accessor left-open :initarg :left-open :initform nil)
   (right-open :accessor right-open :initarg :right-open :initform nil))
  (:documentation "Represents an interval in the real-line.  

Initargs
:a, :b - bounds of the interval.  Extended reals.
:left-open, :right-open.  Nil by default.  It is an error for these to be nil for +-infinite bounds.

"))


(defmethod initialize-instance :after ((i <interval>) &rest args)
  (declare (ignore args))
  (unless (slot-boundp i 'a)
    (setf (left-bound i) '-infty
	  (left-open i) t))
  (unless (slot-boundp i 'b)
    (setf (right-bound i) 'infty
	  (right-open i) t))
  (assert (or (left-open i) (not (infinite (left-bound i))))
      nil "Interval must either be left-open or the left bound must be finite.")
  (assert (or (right-open i) (not (infinite (right-bound i))))
      nil "Interval must either be right-open or the right bound must be finite."))
  
(defun make-closed-interval (a b)
  (make-instance '<interval> :a a :b b))

(defmethod member? (x (s <interval>))
  (let ((a (left-bound s))
	(b (right-bound s)))
    (and (typep x 'real)
	 (if (left-open s)
	     (my> x a)
	   (my>= x a))
	 (if (right-open s)
	     (my< x b)
	   (my<= x b)))))

(defun interval-length (i)
  (my- (right-bound i) (left-bound i)))

(defmethod binary-intersection ((s <interval>) (s2 <interval>))
  (let ((a (left-bound s))
	(a2 (left-bound s2))
	(b (right-bound s))
	(b2 (right-bound s2)))
    (make-instance '<interval>
      :a (mymax a a2)
      :b (mymin b b2)
      :left-open (cond
		  ((my> a a2) (left-open s))
		  ((my> a2 a) (left-open s2))
		  (t (or (left-open s) (left-open s2))))
      :right-open (cond
		   ((my< b b2) (right-open s))
		   ((my< b2 b) (right-open s2))
		   (t (or (right-open s) (right-open s2)))))))


(defmethod intersects ((s <interval>) (s2 <interval>))
  (when (my> (left-bound s) (left-bound s2))
    (rotatef s s2))
  (if (eql (right-bound s) (left-bound s2))
      (not (or (right-open s) (left-open s2)))
    (my> (right-bound s) (left-bound s2))))

(defmethod subset ((s1 <interval>) (s2 <interval>))
  (with-accessors ((a1 left-bound) (b1 right-bound)) s1
    (with-accessors ((a2 left-bound) (b2 right-bound)) s2
      (and (or (my< a2 a1)
	       (and (eql a2 a1) (or (left-open s1) (not (left-open s2)))))
	   (or (my> b2 b1)
	       (and (eql b1 b2) (or (right-open s1) (not (right-open s2)))))))))

(defmethod is-empty ((s <interval>))
  (with-accessors ((a left-bound) (b right-bound)) s
    (or (my> a b) (and (eql a b) (or (left-open s) (right-open s))))))

(defmethod vdc-sequence ((s <interval>) &optional constant-space (base 0))
  (declare (ignore constant-space))
  (let* ((f (vdc-generator base))
	 (a (left-bound s))
	 (l (- (right-bound s) a)))
    (repeat 2 (funcall f)) ;; get rid of endpoints
    (values
     #'(lambda ()
	 (+ a (* l (funcall f))))
     (incf base))))
  

(defmethod print-object ((s <interval>) str)
  (if (left-open s)
      (print-unreadable-object (s str :type t :identity nil)
	(format str "~:[[~;(~]~a, ~a~:[]~;)~]" 
		(left-open s) (left-bound s) (right-bound s) (right-open s)))
      (format str "~:[[~;(~]~a, ~a~:[]~;)~]" 
	      (left-open s) (left-bound s) (right-bound s) (right-open s))))
      

