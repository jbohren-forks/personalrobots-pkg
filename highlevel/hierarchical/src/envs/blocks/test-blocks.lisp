(defpackage test-blocks
  (:use cl
	blocks
	set
	utils
	lookahead
	prop-logic
	rl-user
	)
  )

(in-package test-blocks)

(setf d (blocks:make-blocks-world-with-ceiling 4 4 '((a 0 1) (baz 2 1) (c 2 2))
					       '(2 3) '((a 1 2) (c 1 1))))

