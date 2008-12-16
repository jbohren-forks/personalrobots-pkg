(defpackage queue
  (:documentation "Package queue

Types
-----
queue

Operations
----------
make-queue
enqueue
dequeue
peek-front
queue-empty
")
  (:export
   queue
   
   make-queue
   queue-empty
   enqueue
   dequeue
   peek-front)
  (:use cl
	utils)
  )

(in-package queue)

(defstruct (queue (:conc-name nil) (:constructor create-queue))
  head
  tail)

(defun make-queue (&optional (s nil))
  "make-queue &optional (SEQUENCE nil)

Makes a queue out of SEQUENCE where (elt SEQUENCE 0) is the head (i.e., the first to be dequeued)."
  
  (let* ((head (etypecase s (list s) (vector (map 'list #'identity s))))
	 (tail (last head)))
    (create-queue :head head :tail tail)))

(defun queue-empty (queue)
  "queue-empty QUEUE.  Return t iff the queue has no elements."
  (null (head queue)))

(defun enqueue (item q)
  "enqueue ITEM QUEUE.  Add ITEM to the back of the QUEUE."
  (let ((new-pair (list item)))
    (if (queue-empty q)
	(setf (head q) (setf (tail q) new-pair))
      (setf (tail q)
	(setf (cdr (tail q))
	  new-pair)))))

(defun dequeue (q)
  "dequeue QUEUE.  Return 1) the first item on the queue if it exists (in this case the item is removed from the queue) 2) t if the queue was nonempty, nil otherwise."
  (if (queue-empty q)
      (values nil nil)
    (prog1
	(car (head q))
      (setf (head q) (cdr (head q))))))

(defun peek-front (q)
  "peek-front QUEUE.  Has the same return values as dequeue, but does not modify the queue."
  (if (queue-empty q)
      (values nil nil)
    (car (head q))))
    

