(in-package :blocks)

(defun possibly-clear-blocks (opt-set)
  (mapcar #'car (make-and-solve-csp (pss-domain opt-set) '((clear ?b)) '((?b . actual-blocks)) opt-set)))

(defun possibly-clear-surfaces (opt-set)
  (mapcar #'car (make-and-solve-csp (pss-domain opt-set) '((clear ?b)) '((?b . blocks)) opt-set)))

(defun possible-gripper-positions (opt-set)
  (make-and-solve-csp (pss-domain opt-set) '((gripper-pos ?x ?y)) '((?x . columns) (?y . all-rows)) opt-set))

(defun unique-block-position (b opt-set &aux (d (pss-domain opt-set)))
  (let ((pos (if (make-and-solve-csp d '((gripper-holding ?b)) `((?b ,b)) opt-set)
	       (possible-gripper-positions opt-set)
	       (make-and-solve-csp d '((block-pos ?b ?x ?y)) `((?b ,b) (?x . columns) (?y . all-rows)) opt-set))))
    (assert (null (cdr pos)))
    (cdar pos)))

(defun possibly-free (c r opt-set)
  (let ((d (pss-domain opt-set)))
    (and (between2 c 0 (num-cols d))
	 (between2 r 1 (num-rows d))
	 (make-and-solve-csp d '((free ?r ?c)) `((?r ,r) (?c ,c)) opt-set))))

