(in-package :blocks)

(defun possibly-clear-blocks (opt-set)
  (when opt-set
    (mapcar #'car (make-and-solve-csp (pss-domain opt-set) '((clear ?b)) '((?b . actual-blocks)) opt-set))))

(defun possibly-clear-surfaces (opt-set)
  (when opt-set
    (mapcar #'car (make-and-solve-csp (pss-domain opt-set) '((clear ?b)) '((?b . blocks)) opt-set))))

(defun possible-gripper-positions (opt-set)
  (when opt-set
    (make-and-solve-csp (pss-domain opt-set) '((gripper-pos ?x ?y)) '((?x . columns) (?y . all-rows)) opt-set)))

(defun unique-block-position (b opt-set &aux (d (pss-domain opt-set)))
  (if (make-and-solve-csp d '((gripper-holding ?b)) `((?b ,b)) opt-set)
      (let ((p (possible-gripper-positions opt-set)))
	(assert (null (cdr p)))
	(car p))
      (let ((p (make-and-solve-csp d '((block-pos ?b ?x ?y)) `((?b ,b) (?x . columns) (?y . all-rows)) opt-set)))
	(assert (null (cdr p)))
	(cdar p))))


(defun possibly-free (c r opt-set)
  "Return non-nil iff (c,r) is possibly unoccupied"
  (let ((d (pss-domain opt-set)))
    (and (between2 c 0 (num-cols d))
	 (between2 r 1 (num-rows d))
	 (make-and-solve-csp d '((free ?c ?r)) `((?r ,r) (?c ,c)) opt-set))))


(defun possibly-facing (opt-set)
  "Return list of directions gripper is possibly facing in opt-set ('left or 'right)."
  (nconc
   (when (make-and-solve-csp (pss-domain opt-set) '((faceR)) nil opt-set) '(right))
   (when (make-and-solve-csp (pss-domain opt-set) '((faceL)) nil opt-set) '(left))))

(defun unique-block-below (b opt-set)
  "Return the unique block below b in this state-set.  Assert if there isn't exactly one."
  (let ((l (make-and-solve-csp (pss-domain opt-set) '((on ?b ?c)) `((?c . blocks) (?b ,b)) opt-set)))
    (assert (= 1 (length l)) nil "Set of blocks below ~a was ~a in opt-set." b l opt-set)
    (caar l)))
    
  