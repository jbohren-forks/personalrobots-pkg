(defpackage :two-by-n
  (:use :create-env :utils :lookahead :cl :decomp :set :vb-node) 
  (:export  
   :<two-by-n>
   :<two-by-n-hierarchy>
   :<two-by-n-descriptions>

   :top
   :traverse

   
   :n))



(in-package :two-by-n)


(defclass <two-by-n> (<planning-problem>)
  ((costs :type (array integer 3) :initarg :costs :reader costs)
   (universal-set :accessor universal-set))
  (:documentation "Two-by-n illustrative planning problem.

Represents a world where states are form the form (0,i) and (1,i) where i ranges from 0 to n-1.  From any state (x,i) you can do action 0 or 1, transitioning, respectively, to (0,i+1) or (1,i+1).  The :costs initarg must be specified at initialization, and is a n-1x2x2 array where costs(m,i,j) is the cost of doing j in (m,i)."))

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


    
  