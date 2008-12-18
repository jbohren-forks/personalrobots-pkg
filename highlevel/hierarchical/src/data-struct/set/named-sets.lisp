;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; data-struct/set/named-sets.lisp
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package set)


(defmethod is-empty ((s symbol))
  (not s))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; natural numbers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (item (s (eql 'natural-numbers)))
  (and (typep item 'integer)
       (>= item 0)))

(defmethod iterator ((s (eql 'natural-numbers)))
  (let ((current 0))
    (lambda ()
      (let ((ret current))
	(incf current)
	(iterator-not-done ret)))))

(defmethod size ((s (eql 'natural-numbers)) &optional (constant-time nil))
  (declare (ignore constant-time))
  'infty)

(defmethod item-number (item (s (eql 'natural-numbers)))
  (if (member? item s)
      item
    (error 'item-not-in-set :item item :set s)))


(defmethod item (num (s (eql 'natural-numbers)))
  "Todo condition"
  num)



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; real numbers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (item (s (eql 'real-numbers)))
  (typep item 'real))

(defmethod size ((s (eql 'real-numbers)) &optional (constant-time nil))
  (declare (ignore constant-time))
  'infty)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; t denotes the universal set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (item (s (eql t)))
  (declare (ignore item))
  t)


(defmethod intersects (s1 (s2 (eql t)))
  (not (is-empty s1)))

(defmethod intersects ((s1 (eql t)) s2)
  (not (is-empty s2)))

(defmethod binary-intersection ((s1 (eql t)) s2)
  s2)

(defmethod binary-intersection (s1 (s2 (eql t)))
  s1)

