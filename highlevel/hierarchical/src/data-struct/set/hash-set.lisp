;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; data-struct/set-hash-set.lisp
;; treating (generalized) hash tables as sets
;; 
;; There are two allowed cases
;; 1. Each element is mapped to a boolean
;; 2. Each element is mapped to a fixnum (in this case the set is numbered)
;;
;; When add is called, it will continue with the option that has been followed
;; so far.  If the table is empty, it will assume it's numbered.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package set)

(defmethod size ((s hash-table) &optional (constant-time nil))
  (declare (ignore constant-time))
  (hash-table-count s))

(defmethod size ((s gen-hash-table) &optional (constant-time nil))
  (declare (ignore constant-time))
  (hash-table-count* s))


(defmethod equality-test ((s hash-table))
  (hash-table-test* s))

(defmethod equality-test ((s gen-hash-table))
  (hash-table-test* s))

(defmethod item-number (item (s hash-table))
  (hs-item-number item s))

(defmethod item-number (item (s gen-hash-table))
  (hs-item-number item s))

(defun hs-item-number (item s)
  (multiple-value-bind (num present?)
      (gethash* item s)
    (if present?
	(typecase num
	  (boolean (assert nil nil "Hash set is not numbered"))
	  (otherwise num))
      (error 'item-not-in-set :item item :set s))))


(defmethod member? (item (s hash-table))
  (hs-member? item s))

(defmethod member? (item (s gen-hash-table))
  (hs-member? item s))

(defun hs-member? (item s)
  (multiple-value-bind (num present?)
      (gethash* item s)
    (declare (ignore num))
    present?))

(defun is-numbered-hash-set (h)
  (let ((keys (hash-keys h)))
    (or (not keys)
	(typep (gethash (first keys) h) 'number))))

(defmethod add ((s hash-table) item &optional pos)
  (hs-add s item pos))

(defmethod add ((s gen-hash-table) item &optional pos)
  (hs-add s item pos))

(defun hs-add (s item pos)
  (assert (member pos '(nil t)))
  (unless (hash-table-has-key s item)
    (setf (gethash* item s) (if (is-numbered-hash-set s) (hash-table-count* s) t)))
  s)

(defmethod iterator ((s hash-table))
  (hs-iterator s))

(defmethod iterator ((s gen-hash-table))
  (hs-iterator s))

(defun hs-iterator (s  &aux (keys (hash-keys s)))
  (if (is-numbered-hash-set s)
      (let ((n (length keys))
	    (i 0))

	#'(lambda ()
	    (if (>= i n)
		(iterator-done)
	      (let ((key (find-if #'(lambda (k) (eq (gethash* k s) i)) keys)))
		(assert (eq (gethash* key s) i) nil "Could not find element ~a of ~a" i s)
		(incf i)
		key))))
    (iterator keys)))
	  

