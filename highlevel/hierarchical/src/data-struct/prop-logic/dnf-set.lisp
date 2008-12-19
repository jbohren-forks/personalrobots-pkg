;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; lookahead/prop/dnf-set.lisp
;; Represents the set of satisfying assignments for a DNF formula
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(in-package prop-logic)

(defclass <dnf-set> (<set>)
  ((formula :type [formula]
	    :reader formula
	    :initarg :formula
	    :writer set-formula)
   (propositions :type [numbered-set]
		 :reader props
		 :initarg :props
		 :writer set-props)
   (num-props :accessor num-props))
  (:documentation "<dnf-set>.  Represents the set of satisfying assignments of a DNF formula. Create using make-dnf-set.

DNF sets support many of the standard set operations.  One thing to note is that  equality is not exactly correct - it returns t iff the two formulae have the same canonical representations.  So it might sometimes fail to detect equivalent formulae (not sure)."))

(defun make-dnf-set (formula props)
  "make-dnf-set FORMULA PROP-SET.  Make a <dnf-set> object representing the set of satisfying assignments to FORMULA over the propositions in PROP-SET (which must include those in formula)."
  (make-instance '<dnf-set> :formula formula :props props)
  )

(defvar *strict* nil)

(defmethod initialize-instance :after ((s <dnf-set>) &rest args &key formula props)
  (declare (ignore args))
  (when *strict*
    (assert (is-dnf-formula formula) nil "~a is not a dnf formula" formula)
    (let ((formula-props (propositions formula)))
      (assert (subset formula-props props) ()
	"Propositions in ~a are ~a, which is not a subset of ~a"
	formula formula-props props)))
  (setf (num-props s) (set:size props)))


(defmethod member? (item (s <dnf-set>))
  (let ((props (props s)))
    (and
     (each item  #'(lambda (x) (member? x props)))
     (holds item (formula s)))))

(defmethod binary-intersection ((s <dnf-set>) (s2 <dnf-set>))
  (let ((p (props s))
	(p2 (props s2)))
    (when *strict*
    (assert (set-eq p p2) ()
      "Sets ~a and ~a have differing proposition sets ~a and ~a" 
      s s2 p p2))
    (make-dnf-set
     (dnf-and (formula s) (formula s2))
     p)))



(defmethod binary-union ((s <dnf-set>) (s2 <dnf-set>))
  (make-dnf-set (dnf-or (formula s) (formula s2)) (props s)))
 
(defmethod is-empty ((s <dnf-set>))
  (not (disjuncts (formula s))))

(defun size-must-exceed (s n)
  (let ((x (- (num-props s) (log n 2) .000001)))
    (some 
     #'(lambda (clause)
	 (not (length-exceeds (conjuncts clause) x)))
     (disjuncts (formula s)))))
	   
	   

; (defmethod binary-union ((s <dnf-set>) (s2 <dnf-set>))
;   (let ((p (props s))
; 	(p2 (props s2)))
;     (when *strict*
;       (assert (set-eq p p2) ()
; 	"Sets ~a and ~a have differing proposition sets ~a and ~a" 
; 	s s2 p p2))
;     (make-dnf-set
;      (dnf-or (formula s) (formula s2))
;      p)))

(defmethod subset ((s <dnf-set>) (s2 <dnf-set>))
  (dnf-implies (formula s) (formula s2)))

(defmethod clone ((s <dnf-set>))
  (make-dnf-set
   (clone (formula s))
   (clone (props s))))

(defmethod print-object ((s <dnf-set>) str)
  (format str "<<DNF Set ")
  (pprint-dnf str (formula s))
  (format str ">>"))


(defmethod set-eq ((s1 <dnf-set>) (s2 <dnf-set>))
  (let ((f (formula s1))
	(f2 (formula s2)))
    (and (set-eq (props s1) (props s2))
	 (dnf-implies f f2) (dnf-implies f2 f))))