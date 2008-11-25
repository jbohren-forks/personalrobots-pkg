(defpackage test-graph
  (:use
   set
   utils
   cl
   graph))

(in-package test-graph)

(defvars g g2 g3 g4)

(setf g (make-instance '<adjacency-list-graph>
	  :nodes '(foo baz bar qux)
	  :adjacency-lists
	  #((baz qux) (foo baz) () (bar foo baz))))


(do-tests "Graph accessors"
  (get-edge g 'qux 'baz) t
  (get-edge g 'baz 'qux) nil
  (get-edge g 'baz 'baz) t
  (results-in-error unknown-node (get-edge g 'oof 'baz)) t
  (results-in-error unknown-node (get-edge g 'baz 'oof)) t)

(setf g2 (make-instance '<adjacency-matrix-graph>
	   :nodes '(s v1 v2 t)
	   :m #2A((nil 4 4 nil) (nil nil 1 5) (nil 3 nil 2) (nil nil nil nil))))

(tests "Max flow"
       ((edmonds-karp g2 's t) #2A((0 4 3 0) (0 0 0 5) (0 1 0 2) (0 0 0 0))))

(setf g3 (make-instance '<adjacency-list-graph>
			   :nodes '(a b c d e f g)
			   :labeled-adjacency-lists #(((b . 2)) ((c . 3)) () ((e . 1) (f . 5)) ((g . 3)) ((g . 6)) ())))

(tests "Matching"
       ((max-weight-bipartite-matching g3) (list (get-edge g3 'b 'c) (get-edge g3 'e 'g) (get-edge g3 'd 'f)) #'set-eq))
					   
					   

(setf g4 (make-instance '<adjacency-list-graph>
	   :nodes '(a b c d e f g h)
	   :adjacency-lists #((b c) () (b) (b) (f) () (f) ())))

(tests "Shortest path"
       ((unweighted-shortest-path g4 'a 'd) '(a b d))
       ((unweighted-shortest-path g4 'd 'a) '(d b a))
       ((unweighted-shortest-path g4 'c 'b) '(c b))
       ((unweighted-shortest-path g4 'g 'e) '(g f e))
       ((unweighted-shortest-path g4 'c 'g) nil)
       ((unweighted-shortest-path g4 'a 'h) nil))