;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Code for dealing with diffs of a variable's value
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package :dependency-graph)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; To overload for specific diff types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defgeneric apply-diff (diff old-val)
  (:documentation "Return the new value after applying DIFF to OLD-VAL."))

(defgeneric compose-diffs (diff1 diff2)
  (:documentation "Return a new diff consisting of applying DIFF2 followed by DIFF1."))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Diffs that just say what the new value is
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass new-val-diff ()
  ((new-val :initarg :new-val :reader new-val)))

(defun new-val-diff (v)
  (make-instance 'new-val-diff :new-val v))

(defmethod apply-diff ((d new-val-diff) v)
  (declare (ignore v))
  (new-val d))

(defmethod compose-diffs ((d new-val-diff) d2)
  (declare (ignore d2))
  d)

(defmethod print-object ((diff new-val-diff) str)
  (print-unreadable-object (diff str :type nil :identity nil)
    (format str "New val ~a" (new-val diff))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; A list is treated as a sequence of diffs
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod apply-diff ((d list) v)
  (if d
    (apply-diff (first d) (apply-diff (rest d) v))
    v))

(defmethod compose-diffs (d (d2 list))
  (cons d d2))

;; When composing with the empty list, no need to create a singleton list  
(def-symmetric-method compose-diffs ((d null) d2)
  d2)