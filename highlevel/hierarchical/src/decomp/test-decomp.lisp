(defpackage :test-decomp
  (:use :lookahead
	:decomp
	:utils
	:blocks
	:cl
	:decomp
	:mapping
	:env-user
	:set
	:vb-node
	:prop-logic)
  )

(in-package :test-decomp)

(defvars nr nc d h)

(setf nr 4
      nc 4
      d (make-blocks-world-with-ceiling nr nc '((a 0 1) (baz 2 1) (c 2 2))
					'(2 3) '(and (block-pos a 1 2) (block-pos c 1 1)))
      h (make-instance '<blocks-hierarchy> :domain d))






