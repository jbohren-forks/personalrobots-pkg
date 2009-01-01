(in-package :hla)

(defstruct (simple-valuation (:conc-name sv-) (:constructor create-simple-valuation (s v)))
  "A simple valuation consists of a set and number and represents the function that equals the number on the set and -infty elsewhere."
  s v)

(defun make-simple-valuation (s v)
  (create-simple-valuation s (if (is-empty s) '-infty v)))

(defun initial-valuation (d)
  (make-simple-valuation (init-state-set d) 0))

(defun final-valuation (d)
  (make-simple-valuation (goal d) 0))

(defmethod evaluate-valuation ((v simple-valuation) s) 
  (if (member? s (sv-s v)) (sv-v v) '-infty))

(defmethod equal-valuations ((val1 simple-valuation) (val2 simple-valuation))
  (let ((v1 (sv-v val1))
	(v2 (sv-v val2))
	(s1 (sv-s val1))
	(s2 (sv-s val2)))
    (or (and (eq v1 '-infty) (eq v2 '-infty))
	(and (is-empty s1) (is-empty s2))
	(and (eql v1 v2) (set-eq s1 s2)))))

(defmethod pointwise-subsumes ((v1 simple-valuation) (v2 simple-valuation))
  (or (eql '-infty (sv-v v2))
      (is-empty (sv-s v2))
      (and (subset (sv-s v2) (sv-s v1))
	   (my<= (sv-v v2) (sv-v v1)))))

(defmethod reachable-set ((val simple-valuation))
  (when (my> (sv-v val) '-infty)
    (sv-s val)))

(defmethod max-achievable-value ((val simple-valuation))
  (if (is-empty (sv-s val))
      '-infty
      (sv-v val)))


(defmethod binary-pointwise-min-upper-bound ((v1 simple-valuation) (v2 simple-valuation))
  (make-simple-valuation (binary-intersection (sv-s v1) (sv-s v2)) (mymin (sv-v v1) (sv-v v2))))


;; Not used right now
(defun binary-pointwise-max-upper-bound-approx-simple (v1 v2)
  ;; For simple valuations, union the sets and take the max value, except in the special case where the set for the max val is empty, in which case return the other valuation
  (when (my> (sv-v v2) (sv-v v1))
    (rotatef v1 v2))
  (if (is-empty (sv-s v1))
      v2
      (make-simple-valuation (binary-union (sv-s v1) (sv-s v2)) (sv-v v1))))

(defmethod binary-pointwise-max-upper-bound ((v1 simple-valuation) (v2 simple-valuation))
  (make-max-valuation (list v1 v2)))

(defmethod binary-pointwise-max-lower-bound ((v1 simple-valuation) (v2 simple-valuation))
  (make-max-valuation (list v1 v2)))


(defmethod print-valuation ((v simple-valuation) &optional (str t))
  (pprint-logical-block (str nil)
    (format str "Simple valuation~:@_ Value: ~a~:@_ Set: ~/set:pprint-set/" (sv-v v) (sv-s v))))
