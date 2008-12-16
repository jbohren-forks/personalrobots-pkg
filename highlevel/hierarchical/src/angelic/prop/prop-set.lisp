(in-package lookahead)

(defclass <prop-domain-state> (<numbered-set>)
  ((fluents :initarg :fluents :accessor fluents)
   (nonfluents :initarg :nonfluents :reader nonfluents)
   (all-props :accessor all-props)))

(defmethod initialize-instance :after ((s <prop-domain-state>) &rest args &key fluents nonfluents)
  (setf (all-props s) (disjoint-union fluents nonfluents)))

(defmethod member? (item (s <prop-domain-state>))
  (member? item (all-props s)))


(defmethod iterator ((s <prop-domain-state>))
  (iterator (all-props s)))

