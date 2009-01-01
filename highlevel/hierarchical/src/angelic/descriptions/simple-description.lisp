(in-package :hla)

(defclass <simple-description> ()
  ((succ-state-fn :initarg :succ-state-fn :reader succ-state-fn)
   (predecessor-fn :initarg :predecessor-fn :reader predecessor-fn)
   (reward-fn :initarg :reward-fn :reader reward-fn))
  (:documentation "A simple description has methods for successor-set and hla-sound/complete-reward.  It then progresses/regresses valuations by first finding the successor/predecessor-set, then calling hla-reward.   The successor, predecessor and reward functions can be provided using the initargs :succ-state-fn, :predecessor-fn and :reward-fn (note that there aren't separate functions for sound/complete - override hla-sound/complete-reward if this is a problem, like in ncstrips.lisp).  Alternatively, subclasses may override successor-set, regress and/or hla-reward."))

(defun make-simple-description (succ-state-fn predecessor-fn reward-fn)
  (make-instance '<simple-description> :succ-state-fn (designated-function succ-state-fn) :predecessor-fn (designated-function predecessor-fn) :reward-fn (designated-function reward-fn)))

(defmethod hla-complete-reward ((d <simple-description>) s s2)
  (funcall (reward-fn d) s s2))

(defmethod hla-sound-reward ((d <simple-description>) s s2)
    (funcall (reward-fn d) s s2))

(defmethod successor-set ((d <simple-description>) s)
  (funcall (succ-state-fn d) s))

(defmethod regress (s1 s2 (d <simple-description>))
  (funcall (predecessor-fn d) s1 s2))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Simple valuations
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defmethod progress-sound-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-sound-reward d s s2)))
      (make-simple-valuation s2 (iunless (is-empty s2) (my+ r v))))))

(defmethod progress-complete-valuation ((d <simple-description>) (val simple-valuation))
  (with-struct (sv- s v) val
    (let* ((s2 (successor-set d s))
	   (r (hla-complete-reward d s s2)))
      (make-simple-valuation s2 (my+ r v)))))

(defmethod regress-sound-valuation ((d <simple-description>) (val1 simple-valuation) (val2 simple-valuation))
  (let* ((s2 (sv-s val2))
	 (s1 (regress (sv-s val1) s2 d))
	 (r (hla-sound-reward d s1 s2)))
    ;; Adding this check won't change behavior, and avoids occasional annoying exceptions with adding infty and -infty
    (make-simple-valuation s1 (iunless (is-empty s1) (my+ r (sv-v val2))))))

(defmethod regress-complete-valuation ((d <simple-description>) (val1 simple-valuation) (val2 simple-valuation))
  (let* ((s2 (sv-s val2))
	 (s1 (regress (sv-s val1) s2 d))
	 (r (hla-complete-reward d s1 s2)))
    (make-simple-valuation s1 (my+ r (sv-v val2)))))



;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Max valuations
;; Just progress/regress the maximands then maximize
;; TODO is this always correct?
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


(defmethod progress-sound-valuation ((d <simple-description>) (val <max-valuation>))
  (make-max-valuation (mapcar #'(lambda (v) (progress-sound-valuation d v)) (max-valuation-vals val))))

(defmethod regress-sound-valuation ((d <simple-description>) val1 (val <max-valuation>))
  (make-max-valuation (mapcar #'(lambda (v) (regress-sound-valuation d val1 v)) (max-valuation-vals val))))

(defmethod regress-sound-valuation ((d <simple-description>) (val1 <max-valuation>) val)
  (make-max-valuation (mapcar #'(lambda (v) (regress-sound-valuation d v val)) (max-valuation-vals val1))))

(defmethod progress-complete-valuation ((d <simple-description>) (val <max-valuation>))
  (make-max-valuation (mapcar #'(lambda (v) (progress-complete-valuation d v)) (max-valuation-vals val))))

(defmethod regress-complete-valuation ((d <simple-description>) val1 (val <max-valuation>))
  (make-max-valuation (mapcar #'(lambda (v) (regress-complete-valuation d val1 v)) (max-valuation-vals val))))

(defmethod regress-complete-valuation ((d <simple-description>) (val1 <max-valuation>) val)
  (make-max-valuation (mapcar #'(lambda (v) (regress-complete-valuation d v val)) (max-valuation-vals val1))))



