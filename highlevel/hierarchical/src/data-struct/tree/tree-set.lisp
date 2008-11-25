(defpackage tree-set
  (:documentation "Package tree-set

Types
-----
<tree-set>
")
  (:export
   <tree-set>)
  (:use
   cl
   set
   tree
   utils))

(in-package tree-set)


(defclass <tree-set> (<set>)
  ((root :accessor root))
  (:documentation "A set of lists, that stores them efficiently using a tree.

Initargs
:items - list of lists that represents the initial items in the tree.  nil by default."))

(defmethod initialize-instance :after ((s <tree-set>) &rest args &key (items nil))
  (declare (ignore args))
  (setf (root s)
    (make-instance '<node> :node-label 'no-element))
  (dolist (item items)
    (add item s)))

(defmethod member? (item (s <tree-set>))
  (labels ((helper (l n)
	     (if (null l)
		 (not (eq (tree:node-label n) 'no-element))
	       (dsbind (x . y) l
		 (awhen (find-element (tree:child-edges n) #'(lambda (e) (equal (tree:edge-label e) x)))
		   (helper y (head it)))))))
    (helper item (root s))))

(defmethod add ((s <tree-set>) item &optional (pos nil))
  (assert (null pos) () "Can't add positionally in a tree-set")
  
  (labels ((helper (l n)
	     (if (null l)
		 (setf (tree:node-label n) item)
	       (dsbind (x . y) l
		 (aif (find-element (tree:child-edges n) #'(lambda (e) (equal (tree:edge-label e) x)))
		     (helper y (head it))
		   (helper y (add-new-child n x 'no-element)))))))
    (helper item (root s))))
  

(defmethod iterator ((s <tree-set>))
  (let ((iter (preorder-iterator (root s))))
    #'(lambda ()
	(loop
	  (mvbind (next done?) (funcall iter)
	    (if done?
		(return (iterator-done))
	      (let ((item (node-label next)))
		(unless (eq item 'no-element)
		  (return (iterator-not-done item))))))))))

(defmethod size ((s <tree-set>) &optional (constant-time nil))
  (if constant-time (call-next-method)
    (let ((n 0))
      (do-preorder (x (root s) n)
	(unless (eq (node-label x) 'no-element) (incf n))))))
	    

(defmethod item (n (s <tree-set>))
  (do-elements (x s (assert "not found") i)
    (when (= i n)
      (return x))))
					      
				    
		 
