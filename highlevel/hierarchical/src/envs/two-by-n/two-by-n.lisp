(defpackage :two-by-n
  (:use :create-env :utils :lookahead :cl :decomp :set :vb-node) 
  (:export  
   :<two-by-n>
   :<two-by-n-hierarchy>
   :<two-by-n-descriptions>
   :two-by-n-abstract-planning-problem
   :random-two-by-n-env

   :top
   :traverse

   
   :n))



(in-package :two-by-n)


(defclass <two-by-n> (<planning-problem>)
  ((costs :type (array integer 3) :initarg :costs :reader costs)
   (all-actions :reader all-actions :initform 2)
   (universal-set :accessor universal-set))
  (:documentation "Two-by-n illustrative planning problem.

Represents a world where states are form the form (0,i) and (1,i) where i ranges from 0 to n-1.  From any state (x,i) you can do action 0 or 1, transitioning, respectively, to (0,i+1) or (1,i+1).  The :costs initarg must be specified at initialization, and is a 2xnx2 array where costs(m,i,j) is the cost of doing j in (m,i)."))

(defun n (p)
  (array-dimension (costs p) 1))

(defmethod initialize-instance :after ((p <two-by-n>) &rest args)
  (declare (ignore args))
  (setf (universal-set p) 
	(direct-product 'list 2 (1+ (n p)))))

(defmethod goal ((p <two-by-n>))
  (let ((n (n p)))
    `((0 ,n) (1 ,n))))

(defmethod init-state ((p <two-by-n>))
  '(0 0))

(defmethod avail-actions ((p <two-by-n>) s)
  (declare (ignore s))
  '(0 1))

(defmethod reward ((p <two-by-n>) s a)
  (my- (aref (costs p) (first s) (second s) a)))

(defmethod primitive-action-description ((p <two-by-n>) a)
  #'(lambda (s)
      (list a (1+ (second s)))))

(defun random-two-by-n-env (n)
  "Return a random two by n environment, with costs uniformly distributed integers between 0 and 10."
  (let ((costs (make-array (list 2 n 2) :element-type 'float)))
    (dotimes (i 2 (make-instance '<two-by-n> :costs costs))
      (dotimes (j n)
	(dotimes (k 2)
	  (setf (aref costs i j k) (random 10)))))))

