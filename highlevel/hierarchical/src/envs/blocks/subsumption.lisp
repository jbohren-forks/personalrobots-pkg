(in-package blocks)

(defstruct (blocks-subsumption-checker (:constructor create-blocks-subsumption-checker)
	    (:conc-name bss-))
  mapping
  domain
  num-props)


(defun make-blocks-subsumption-checker (d)
  (create-blocks-subsumption-checker :mapping (make-hash-table :test #'equal) :domain d))

   

(defmethod add-entry ((c blocks-subsumption-checker) val)
  (let ((s (sv-s val))
	(v (sv-v val)))
    (let ((state (sound-set-state s (bss-domain c)))
	  (h (bss-mapping c)))
      (when state
	(let ((l (state->list state)))
	  (mvbind (old-v exists?) (gethash l h)
	    (setf (gethash l h) (mymax v (if exists? old-v '-infty)))))))))

(defmethod is-subsumed ((c blocks-subsumption-checker) val)
  (let ((cs (sv-s val))
	(v (sv-v val)))
    (let ((state (sound-set-state cs (bss-domain c)))
	  (h (bss-mapping c)))
      (when state
	(let ((l (state->list state)))
	  (let ((sv (gethash l h)))
	    (awhen (and sv (my> sv v))
	      (debug-print 3 "~a, ~a is subsumed by value ~a" l v sv))))))))

(defmethod clear-subsumption-checker ((c blocks-subsumption-checker))
  (setf (bss-mapping c) (make-hash-table :test #'equal)))


(defun pprint-bsc (&rest args)
  (bind-pprint-args (str c) args
    (pprint-hash str (bss-mapping c))))

(set-pprint-dispatch 'blocks-subsumption-checker #'pprint-bsc)

