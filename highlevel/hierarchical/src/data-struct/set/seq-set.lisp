;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; data-struct/set/seq-set.lisp
;; code relating to sequences or pairs viewed as sets
;; Note that there are three cases.  A list or vector denotes the set of
;; elements, compared using #'equal.  A cons of a function and a vector denotes
;; the set of elements in the vector, but compared using the given function.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package set)

(defmethod member? (item (s cons))
  (if (nonlist s)
      (find item (cdr s) :test (car s))
    (find item s :test #'equal)))

(defmethod member? (item (s null))
  (declare (ignore item))
  nil)

(defmethod member? (item (s vector))
  (find item s :test #'equal))

(defmethod iterator ((s cons))
  (etypecase (cdr s)
    (list (let ((current s))
	    (lambda ()
	      (if current
		  (let ((next (first current))
			(remaining (rest current)))
		    (setf current remaining)
		    (iterator-not-done next))
		(iterator-done)))))
    (vector (iterator (cdr s)))))

(defmethod iterator ((s null))
  #'(lambda () (iterator-done)))

(defmethod iterator ((s vector))
  (let ((ind 0))
    (lambda ()
      (if (< ind (length s))
	  (let ((next (aref s ind)))
	    (incf ind)
	    (iterator-not-done next))
	(iterator-done)))))


(defmethod mapset ((result-type (eql 'same)) fn (set1 list) &rest sets)
  (assert (not (nonlist set1)))
  (apply #'mapset 'list fn set1 sets))

(defmethod mapset ((result-type (eql 'same)) fn (set1 vector) &rest sets)
  (apply #'mapset 'vector fn set1 sets))
  

(defmethod size ((s cons) &optional (constant-time nil))
  (etypecase (cdr s)
    (list (if constant-time ':unknown (length s)))
    (vector (size (cdr s)))))

(defmethod size ((s null) &optional (constant-time nil))
  (declare (ignore  constant-time))
  0)
  

(defmethod size ((s vector) &optional (constant-time nil))
  (declare (ignore constant-time))
  (length s))

(defmethod item-number (item (s cons))
  (let ((pos 
	 (if (nonlist s)
	     (position item (cdr s) :test (car s))
	   (position item s :test #'equal))))
    (aif pos it (error 'item-not-in-set :item item :set s))))

(defmethod item-number (item (s vector))
  (position item s :test #'equal))

(defmethod item-number (item (s null))
  (error 'item-not-in-set :item item :set s))

(defun nonlist (x)
  (and (consp x) (vectorp (cdr x))))

(defmethod item (num (s cons))
  (if (nonlist s)
      (item num (cdr s))
    (let ((p (nthcdr num s)))
      (aif p
	  (car it)
	(error 'index-out-of-bounds :ind num :set s :max-ind (list-length s))))))

(defmethod item (num (s null))
  (error 'index-out-of-bounds :ind num :set s :max-ind -1))

(defmethod item (num (s vector))
  (aref s num))


(defmethod add ((s vector) item &optional (pos nil))
  (add-to-vector s item pos #'equal))

(defun add-to-vector (s item pos test)
  (assert (member pos '(t nil)))
  (if (find item s :test test)
      s
    (if (and (adjustable-array-p s)
	     (array-has-fill-pointer-p s))
	(progn
	  (vector-push-extend item s)
	  s)
      (let ((new-s (make-array (1+ (length s))
			       :adjustable t
			       :fill-pointer 0)))
	(map nil
	  (lambda (x) (vector-push x new-s))
	  s)
	(vector-push item new-s)
	new-s))))

(defmethod add ((s cons) item &optional (pos nil))
  (if (nonlist s)
      (cons (car s) (add-to-vector (cdr s) item pos (car s)))
    (if (member item s :test #'equal) 
	s 
      (cond ((null pos) (push item s))
	    ((eq pos t) (nconc s (list item)))
	    (t (error "POS must be nil or t in add to list"))))))

(defmethod add ((s null) item &optional (pos nil))
  (assert (member pos '(nil t)))
  (list item))

; (defmethod destructive-union ((s list) (s2 list))
;   (assert (not (or (nonlist s) (nonlist s2))))
;   (nconc s s2))


(defmethod intersect ((s1 cons) s2)
  (if (nonlist s1)
      (dsbind (test &rest v) s1
	(cons test (filter ':vector v (membership-predicate s2))))
    (filter ':list s1 (membership-predicate s2))))

(defmethod intersect ((s1 vector) s2)
  (filter ':vector s1 (membership-predicate s2)))

(defmethod intersect (s1 (s2 cons))
  (intersect s2 s1))

(defmethod intersect (s1 (s2 vector))
  (intersect s2 s1))

(defmethod equality-test ((s cons))
  (if (nonlist s)
      (car s)
    #'equal))

