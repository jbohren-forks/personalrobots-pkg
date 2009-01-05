(in-package :hla)

(defclass <functional-description> ()
  ((optimistic-progressor :initarg :optimistic-progressor :accessor optimistic-progressor)
   (pessimistic-progressor :initarg :pessimistic-progressor :accessor pessimistic-progressor)
   (optimistic-regressor :initarg :optimistic-regressor :accessor optimistic-regressor)
   (pessimistic-regressor :initarg :pessimistic-regressor :accessor pessimistic-regressor)))

(defmethod progress-sound-valuation ((d <functional-description>) val)
  (funcall (pessimistic-progressor d) val))

(defmethod progress-complete-valuation ((d <functional-description>) val)
  (funcall (optimistic-progressor d) val))

(defmethod regress-sound-valuation ((d <functional-description>) val1 val2)
  (funcall (pessimistic-regressor d) val1 val2))

(defmethod regress-complete-valuation ((d <functional-description>) val1 val2)
  (funcall (optimistic-regressor d) val1 val2))
 
  