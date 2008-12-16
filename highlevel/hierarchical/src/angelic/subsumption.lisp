(in-package lookahead)

(defgeneric add-entry (subsumption-checker sound-valuation)
  (:documentation "add-entry SUBSUMPTION-CHECKER SOUND-VALUATION
Add an entry for this valuation.")
  (:method ((checker null) val)
	   (declare (ignore val))))
  

(defgeneric is-subsumed (subsumption-checker complete-valuation)
  (:documentation "best-subsumer SUBSUMPTION-CHECKER COMPLETE-VALUATION
Does there exist a subsumer of COMPLETE-VALUATION? If so, return one.  Otherwise, return nil.")
  (:method ((checker null) val)
	   (declare (ignore val))
	   nil)
  )

(defgeneric clear-subsumption-checker (checker)
  (:documentation "Remove all entries from the subsumption checker.")
  (:method ((checker null)))
  )

(defun reset-subsumption-checker (c s hpp)
  (clear-subsumption-checker c)
  (add-entry c (make-simple-valuation (make-state-set hpp s) 0)))

  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; hash-subsumption-checker
;; Only checks equality
;; Only works for simple valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defstruct (hash-subsumption-checker (:constructor create-hash-subsumption-checker) (:conc-name hsc-))
  key
  table)

(defun make-hash-subsumption-checker (key)
  "Key must be a function that maps state sets into a key for which #'equal correctly tests equality."
  (create-hash-subsumption-checker :key key :table (make-hash-table :test #'equal)))

(defmethod add-entry ((c hash-subsumption-checker) (val simple-valuation))
  (with-struct (hsc- key table) c
    (let ((k (funcall key (sv-s val))))
      (setf (gethash k table) (mymax (sv-v val) (or (gethash k table) '-infty))))))


(defmethod is-subsumed ((c hash-subsumption-checker) (val simple-valuation))
  (awhen (gethash (funcall (hsc-key c) (sv-s val)) (hsc-table c))
    (when (my> it (sv-v val))
      (make-simple-valuation (sv-s val) it))))

(defmethod clear-subsumption-checker ((c hash-subsumption-checker))
  (setf (hsc-table c) (make-hash-table :test #'equal)))
  
  