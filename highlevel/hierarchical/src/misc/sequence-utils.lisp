(in-package utils)

(defparameter *default-tail-length* 10)

(defun stail (s &optional (tail-length *default-tail-length*))
  "stail SEQ &optional (LENGTH 10).  Return last LENGTH elements of SEQ."
  (if (> (length s) tail-length)
      (subseq s (- (length s) tail-length))
    s))

(defun slast (s)
  "slast SEQ.  Last element of SEQ."
  (assert (> (length s) 0))
  (elt s (1- (length s))))


(defmacro def-seq-accessor (name pos)
  (let ((s (gensym)))
    `(defun ,name (,s) (elt ,s ,pos))))

(def-seq-accessor sfirst 0)
(def-seq-accessor ssecond 1)
(def-seq-accessor sthird 2)
(def-seq-accessor sfourth 3)
(def-seq-accessor sfifth 4)
(def-seq-accessor ssixth 5)

(defgeneric srest (s)
  (:documentation "srest S.  Works like rest for lists and arrays, without creating new objects (i.e. uses displaced arrays in the second case).  Not setf-able.")
  (:method ((s list)) (rest s))
  (:method ((s array)) (make-array (1- (length s)) :displaced-to s :displaced-index-offset 1)))
    
    
(defgeneric is-sorted (l &optional pred)
  (:documentation "is-sorted SEQUENCE &OPTIONAL (PRED #'<=).  Do every pair of adjacent elements in SEQUENCE satisfy PRED?  (We're assuming PRED is transitive)")
  (:method ((l list) &optional (pred #'<=))
	   (loop
	       for rem on l
	       always (or (not (cdr rem)) (funcall pred (car rem) (cadr rem)))))
  (:method ((l vector) &optional (pred #'<=))
	   (loop
	       for i below (1- (length l))
	       always (funcall pred (aref l i) (aref l (1+ i))))))


(defun extract-subsequence (seq init skip &key (result-type 'vector) &aux (n (length seq)))
  "extract-subsequence SEQUENCE INITIAL SKIP
Return the sequence consisting of SEQUENCE(INITIAL), SEQUENTIAL(INITIAL+SKIP), ..."

  (ecase result-type
    (list (loop
	      for x from init below n by skip
	      collect (elt seq x)))
    (vector (loop
		with a = (make-array (ceiling n skip) :fill-pointer 0)
		for x from init below n by skip
		do (vector-push (elt seq x) a)
		finally (return a)))))


(defun round-sequence (l &optional (d 2))
  "round-sequence S &optional (D 2)"
  (map (if (listp l) 'list 'vector)
    #'(lambda (x) (round-decimal x d)) l))




(defun contiguous-blocks (seq &key (test #'eql) (key #'identity) (start 0) (end (length seq)))
  "contiguous-blocks SEQUENCE &key (TEST #'eql) (KEY #'identity) (START 0) (END (length SEQ))

Suppose you have the sequence (0 0 1 2 1 1 2)
This function would return the list ((0 0 2) (1 2 3) (2 3 4) (1 4 6) (2 6 7))
The meaning of an item (I J K) this list is that, starting from position j until position k, the elements have key values equal to i (according to the test)."
  
  (let ((l (etypecase seq
	     (list 
	      (loop
		  with prev-k = nil
		    
		  for i from start below end
		  for first-time = t then nil
		  for x in (nthcdr start seq)
		  for k = (funcall key x)

		  when (or first-time (not (funcall test k prev-k)))
		  collect (cons k i)
		 
		  do (setf prev-k k)))
     
    
	     (vector
	      (loop
		  with prev-k = nil
		  for i from start below end
		  for first-time = t then nil
		  for x = (aref seq i)
		  for k = (funcall key x)
		  when (or first-time (not (funcall test k prev-k)))
		  collect (cons k i)
		 
		  do (setf prev-k k)))))
	(n (length seq)))
    
    
    (maplist #'(lambda (l2)
		 (dsbind (pair . rem) l2
		   (list (car pair)
			 (cdr pair)
			 (aif rem (cdar it) n))))
	     l)))
      
      

(defun mapcans (f &rest sequences)
  "mapcans FUNCTION &rest SEQUENCES.  Like mapcan, except can operate on lists or vectors"
  (when sequences
    (let ((n (reduce #'min sequences :key #'length)) ;; could be made more efficient if lengths vary a lot
	  (iterators (mapcar #'(lambda (seq) 
				 (if (listp seq) 
				     (let ((l seq)) #'(lambda () (pop l))) 
				   (let ((i -1)) #'(lambda () (aref seq (incf i))))))
			     sequences)))
      (loop
	  repeat n
	  append (apply f (mapcar #'funcall iterators))))))
      
      
    








