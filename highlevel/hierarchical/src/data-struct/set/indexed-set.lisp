(defpackage indexed-set
  (:documentation "Internally used package.  See set:<indexed-set>.")
  (:use set
	utils
	common-lisp))

(in-package indexed-set)
   

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Class definition
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defclass <indexed-set> (<numbered-set>)
  ((s :type [numbered-set] :accessor s)
   (table :type hash-table :accessor table))

  (:documentation "An <indexed-set> consists of a [numbered-set] together with a hashtable, based on the equality test of the set, that maps set elements to their numbers.  Allows the item-number operation to be hashtable lookup, which is basically O(1). The remaining operations are forwarded to the underlying set.  A typical use of this is for the underlying set to be a vector, so that the number->item mapping is fast, and the index makes the item->number mapping and the member? operation fast.  When calling the ADD or ADDF functions on an indexed set, you must use POS argument equal to nil (don't care) or t (don't change order).  Does not support item deletion, and assumes that henceforth, the underlying set is only accessed through the indexed set.

Keywords

One of 
:s - set to build index on.  Defaults to empty vector using equality test :test.
or
:test - equality test.

:hash-fn - hash function from elements to integers (must be compatible with equality test of set, in that equal elements under test map to the same integer).  Must be supplied unless test is a standard equality test."))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; constructors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod initialize-instance :after ((is <indexed-set>) &rest args &key (s nil ssupp) (test nil tsupp) hash-fn)
  (declare (ignore args))
  (assert (xor ssupp tsupp))
  (assert (or (is-standard-equality-test test) hash-fn))
  (setf (s is) (orf s (cons test #())))

  (let ((h (setf (table is)
	     (if hash-fn
		 (make-hash-table* :test #'eql :size (size s t) :hash-fn hash-fn)
	       (make-hash-table :size (size s t) :test test)))))
    (do-elements (x s nil i)
      (setf (gethash* x h) i))))

(defmethod print-object ((s <indexed-set>) str)
  (print-unreadable-object (s str :type t :identity nil)
    (format str "over ~a" (s s))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; operations from set
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod member? (x (s <indexed-set>))
  (hash-table-has-key (table s) x))

(defmethod item (num (s <indexed-set>))
  (item num (s s)))

(defmethod iterator ((s <indexed-set>))
  (iterator (s s)))

(defmethod size ((s <indexed-set>) &optional (constant-time nil))
  (declare (ignore constant-time))
  (hash-table-count* (table s)))

(defmethod add ((s <indexed-set>) x &optional (pos nil) &aux (table (table s)))
  (assert (member pos '(nil t)))
  (assert (not (member? x s)))
  (addf (s s) x t)
  (setf (gethash* x table) (hash-table-count* table))
  s)

(defmethod item-number (x (s <indexed-set>))
  (or (gethash* x (table s)) (error 'item-not-in-set :item x :set s)))

(defmethod equality-test ((s <indexed-set>)) (equality-test (s s)))




