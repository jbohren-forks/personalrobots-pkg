(defpackage skiplist
  (:use
   cl
   set
   utils)
  (:export
   make-skiplist
   
   find-item
   insert
   del
   comparator
   empty?
   minimum
   predecessor
   successor

   print-skiplist
   consistent?
   ))

(in-package skiplist)


(defstruct (node (:conc-name nil) (:constructor create-node))
  (items nil)
  ptrs)

(defstruct (skiplist (:conc-name nil) (:constructor create-skiplist))
  level
  num-levels
  header
  comparator)

(defun make-node (item level)
  (create-node :items (list item) :ptrs (make-array (1+ level) :initial-element 'dummy)))

(defun make-skiplist (&key (num-levels 20) (comparator 'not-provided))
  "Creates a skiplist.

NUM-LEVELS is the max number of levels in the list
COMPARATOR is a function of two arguments that returns '<, '=, or '=.  If not provided, individual calls to skiplist operations must specify it.

COMPARATOR will always be called with the element in the skiplist as the first argument, and the searched-for element as the second argument.  It is possible to use different comparators on the same skiplist, so long as, at the time that the operation is called, the elements in the skiplist are ordered compatible with the comparator.  The comparator of a skiplist may also be directly setf-ed instead of passing it in each time."

  (create-skiplist :level 0 :num-levels num-levels :comparator comparator
		 :header (create-node :items 'header :ptrs (make-array num-levels :initial-element 'dummy))))


(defun find-item (l x &key (comparator (comparator l)))
  "find-item L X &key (COMPARATOR (comparator L))

If the skip list contains elements which are the same as X under COMPARATOR, return 1) one of them 2) the list of all of them.  Otherwise, return nil and nil."
  (let ((node (predecessor-node l x comparator)))
    (setf node (next-node node 0))
    
    (if (node-equivalent comparator node x)
	(let ((items (items node)))
	  (values (first items) items))
	(values nil nil))))

(defun predecessor (l x &key (comparator (comparator l)))
  "predecessor L X &key (COMPARATOR (comparator L))

If the skip list contains elements which are less than X according to COMPARATOR, the biggest one (ties broken arbitrarily).  Otherwise, return nil.  Note X need not be in L."
  (let ((items (items (predecessor-node l x comparator))))
    (unless (eq items 'header)
      (first items))))

(defun successor (l x &key (comparator (comparator l)))
  "Like predecessor."
  (let ((n (predecessor-node l x comparator)))
    (setf n (next-node n 0))
    (while (node-equivalent comparator n x)
      (setf n (next-node n 0)))
    (unless (eq n 'dummy)
      (first (items n)))))

	


(defun del (l x &key (comparator (comparator l)))
  "Remove all items that are equal to X according to comparator.  Return the list of such items."
  (mvbind (node update) (find-item-and-prev-nodes l x comparator)
    (when (node-equivalent comparator node x)
      (delete-node l node update))))

(defun delete-node (l node update)
  (let ((removed-items (items node))
	(ptrs (ptrs node))
	(header (header l)))
    (dotimes (i (1+ (level l)))
      (if (eq (aref (ptrs (aref update i)) i) node)
	  (setf (aref (ptrs (aref update i)) i) (aref ptrs i))
	  (return)))
    (while (and (> (level l) 0) 
		(not (node-p (next-node header (1- (level l))))))
      (decf (level l)))
    removed-items))



(defun empty? (l)
  (eq 'dummy (next-node (header l) 0)))


(defun insert (l x &key (comparator (comparator l)) (unique t))
  "insert L X &key (COMPARATOR (comparator l)) (UNIQUE t)

Insert X into L.  If UNIQUE is true, then replace any existing elements that are equal to X under comparator, otherwise, add X to the list.  Returns the list of equivalent items to X after it was added."
  (mvbind (node update) (find-item-and-prev-nodes l x comparator)

    (if (node-equivalent comparator node x)
	(if unique
	    (setf (items node) (list x))
	    (push x (items node)))
	(let ((v (random-level l)))
	  (when (>= v (level l))
	    (for-loop (i (1+ (level l)) (1+ v))
	      (setf (aref update i) (header l)))
	    (setf (level l) v))

	  (let* ((new-node (make-node x v))
		 (new-node-ptrs (ptrs new-node)))
	    (dotimes (i (1+ v) (list x))
	      (setf (aref new-node-ptrs i) (aref (ptrs (aref update i)) i)
		    (aref (ptrs (aref update i)) i) new-node)))))))


(defun minimum (l &optional (remove? nil))
  (assert (not (empty? l)) nil "Can't find minimum of empty skiplist ~a" l)
  (let* ((header (header l))
	 (node (next-node header 0)))
    (let ((items (if remove?
		     (delete-node l node (make-array (1+ (level l)) :initial-element header))
		     (items node))))
      (values (first items) items))))


(defmethod iterator ((l skiplist))
  "Iterate over the elements in order."
  (let ((n (header l))
	(items nil))
    #'(lambda ()
	(block iter
	  (until items
	    (setf n (next-node n 0))
	    (if (eq n 'dummy)
		(return-from iter (values nil t))
		(setf items (items n))))
	  (values (pop items) nil)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun find-item-and-prev-nodes (l x comp)
  (let ((update (make-array (num-levels l) :initial-element 'debug))
	(node (header l)))
    (for-loop (level (level l) 0 -1 #'<)
      (loop
	 (let ((next (next-node node level)))
	   (if (node-precedes comp next x)
	       (setf node next)
	       (return))))
      (setf (aref update level) node))
    (setf node (next-node node 0))
    (values node update)))


(defun predecessor-node (l x comp)
  (let ((node (header l)))
    (for-loop (level (level l) 0 -1 #'<)
      (loop
	 (let ((next (next-node node level)))
	   (if (node-precedes comp next x)
	       (setf node next)
	       (return)))))
    node))


(defun random-level (l)
  (let ((m (1- (num-levels l)))
	(i 0))
    (loop
       (if (or (= i m) (< (random 1.0) .5))
	   (return i)
	   (incf i)))))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Elementary accessors
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defun node-precedes (comp n x)
  (and (not (eq n 'dummy))
       (eq '< (funcall comp (first (items n)) x))))

(defun node-equivalent (comp n x)
  (and (not (eq n 'dummy))
       (eq '= (funcall comp (first (items n)) x))))

(defun next-node (n i)
  (aref (ptrs n) i))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun print-skiplist (&rest args)
  (bind-pprint-args (str l) args
    (let ((v (make-adjustable-array))
	  (n (header l)))
      (until (eq n 'dummy)
	(vector-push-extend n v)
	(setf n (next-node n 0)))
      (pprint-logical-block (str nil :prefix "[" :suffix "]")
	(do-elements (n v nil i)
	  (format str "~:[~:@_~;~]~a. Node with items ~a.  Forward pointers ~a." (zerop i) i (items n)
		  (map 'vector #'(lambda (p) (or (position p v) p)) 
		       (subseq (ptrs n) 0 (position 'dummy (ptrs n))))))))))
	      

(set-pprint-dispatch 'skiplist #'print-skiplist) 

(defun consistent? (l &key (comparator (comparator l)))
  (let ((h (header l))
	(g (gensym)))
    (dotimes (i (num-levels l) t)
      (let ((n (next-node h i))
	    (v g))
	(until (eq n 'dummy)
	  (let* ((items (items n))
		 (item (first items)))
	    (unless (every #'(lambda (i) (eq '= (funcall comparator item i))) (rest items))
	      (return-from consistent? (values nil (list 'not-equal items))))
	    (unless (or (eq v g) (eq '< (funcall comparator v item)))
	      (return-from consistent? (values nil (list 'not-less-than v item i))))
	    (setf v item
		  n (next-node n i))))))))

	
      
      