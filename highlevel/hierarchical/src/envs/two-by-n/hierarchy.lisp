(in-package :two-by-n)

(defclass <two-by-n-hierarchy> (<decomp-hierarchy>)
  ())

(make-hlas (h <two-by-n-hierarchy>) () (0 1)
  
  ;; Top: move through each stage in sequence
  (top () :sequence
       (let ((n (n (planning-domain h))))
	 (list (loop for i below n collecting `(traverse ,i)))))

  (traverse (i) :or '((0) (1))))