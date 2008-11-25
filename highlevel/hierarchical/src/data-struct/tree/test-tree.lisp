(defpackage test-tree
  (:use cl
	utils
	set
	tree))

(in-package test-tree)


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; basic tree construction, consistency
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defvars n1 n2 n3 n4 n5 e1 e2 e3  deleted tr s)

(setf n1 (make-instance '<node> :node-label 'foo)
      n2 (add-new-child n1 nil 'bar)
      n3 (add-new-child n1 nil 'baz)
      n4 (add-new-child n2 'qux-edge 'qux)
      n5 (add-new-child n2 'oof-edge 'oof))

(setf e1 (make-instance '<edge> :tail n5 :head n1))
(setf e2 (find 'qux-edge (child-edges n2) :key #'edge-label))
(setf e3 (parent-edge n5))

(do-tests
    "consistency checking"
  (is-inconsistent-tree n1 t) nil
  (is-inconsistent-tree n1 nil) nil
  (is-inconsistent-tree n2 t) 'not-root
  (is-inconsistent-tree n2 nil) nil
  
  (progn
    (push e1 (child-edges n5))
    (is-inconsistent-tree n1 t))
  'cycle
  
  (progn 
    (deletef (child-edges n5) e1)
    (is-inconsistent-tree n1 t))
  nil
  
  (progn
    (setf (tail e2) n1)
    (is-inconsistent-tree n1 t))
  'incorrect-child-edge
  
  (progn
    (setf (tail e2) n2)
    (is-inconsistent-tree n2 nil))
  nil
  
  (progn 
    (setf (parent-edge n5) (parent-edge n3))
    (is-inconsistent-tree n1 t))
  'incorrect-parent-edge)


(do-tests 
    "basic-operations"
  (get-path-from-root n4) (list n1 n2 n4)
  (get-path-from-root n1) (list n1))


(do-tests
    "tree traversal"
  (progn
    (setf (parent-edge n5) e3)
    (map-preorder 'list #'node-label n1))
  '(foo bar qux oof baz)
  
  (map-postorder 'list #'node-label n1)
  '(qux oof bar baz foo))

(do-tests
    "tree modification"
  (progn
    (add-subtree n3 nil (copy-subtree n2) 0)
    (setf (node-label (get-child (get-child n3 0) 1)) 'ooof)
    (map-preorder 'list #'node-label n1))
  '(foo bar qux oof baz bar qux ooof)
  
  (progn
    (setf tr (copy-subtree (get-child n3 0)))
    (add-subtree n1 nil tr 1)
    (map-preorder 'list #'node-label n1))
  '(foo bar qux oof bar qux ooof baz bar qux ooof)
  
  (progn
    (setf deleted (remove-subtree (get-child n1 0)))
    (map-preorder 'list #'node-label n1))
  '(foo bar qux ooof baz bar qux ooof)
  
  (map-postorder 'list #'node-label n1)
  '(qux ooof bar qux ooof bar baz foo)
  
  (is-consistent-tree deleted t)
  t
  
  (map-preorder 'list #'node-label deleted)
  '(bar qux oof)
  
  )
  
  
  

(do-tests "tree sets"
  (progn 
    (setf s (make-instance 'set:<tree-set>))
    (member? nil s))
  nil
  
  (member? '(a b) s)
  nil
  
  (size s)
  0
  
  (progn
    (add s '(a (b)))
    (member? '(a (b)) s))
  t
  
  (member? '(a) s)
  nil
  
  (member? nil s)
  nil
  
  (member? '(a c) s)
  nil
  
  (size s)
  1
  
  (progn
    (add s '(a c d))
    (member? '(a c d) s))
  t
  
  (member? '(a) s)
  nil
  
  (member? '(a (b)) s)
  t
  
  (size s)
  2
  
  (progn
    (add s '(b c d))
    (member? '(b c d) s))
  t
  
  (member? nil s)
  nil
  
  (progn 
    (add s nil)
    (member? nil s))
  t
  
  (size s)
  4)
    
  

    