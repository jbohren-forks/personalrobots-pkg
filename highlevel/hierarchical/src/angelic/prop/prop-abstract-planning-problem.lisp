(in-package lookahead)

(defclass <prop-abstract-planning-problem> (<abstract-planning-problem>)
  ((sound-desc-schemas :initarg :sound-desc-schemas :reader sound-desc-schemas :initform nil)
   (complete-desc-schemas :initarg :complete-desc-schemas :reader complete-desc-schemas :initform nil)
   (hset-desc-schemas :initarg :hset-desc-schemas :reader hset-desc-schemas))
  (:documentation "Class <prob-abstract-planning-problem> (<abstract-planning-problem>).  Create using make-instance with initargs
:planning-problem - object of type <prop-domain>
:hierarchy - <prob-hierarchy>
:sound-desc-schemas - a [mapping] from abstract action names to ncstrips schemas
:complete-desc-schemas a [mapping] from abstract action names to ncstrips schemas
:hset-desc-schemas - ditto.  If not provided, defaults to cset-schemas
"))
  
(defmethod initialize-instance :after ((p <prop-abstract-planning-problem>) &rest args)
  (declare (ignore args))
  (set-if-unbound 'hset-desc-schemas p (complete-desc-schemas p)))

(defun lookup-in-ncstrips-schemas (a schemas)
  (instantiate (mapping:evaluate schemas (car a)) (cdr a)))

(defmethod sound-desc ((p <prop-abstract-planning-problem>) a)
  (ecase (action-type a (hierarchy p))
    (primitive (primitive-action-description (planning-problem p) a))
    (high-level (let ((a (if (consp a) a (cons a nil))))
		  (lookup-in-ncstrips-schemas a (sound-desc-schemas p))))))

(defmethod complete-desc ((p <prop-abstract-planning-problem>) a)
  (ecase (action-type a (hierarchy p))
    (primitive (primitive-action-description (planning-problem p) a))
    (high-level (let ((a (if (consp a) a (cons a nil))))
		  (lookup-in-ncstrips-schemas a (complete-desc-schemas p))))))

(defun hset-desc  (p a)
  (let ((a (if (consp a) a (cons a nil))))
    (instantiate (mapping:evaluate (hset-desc-schemas p) (car a)) (cdr a))))

(defmethod progress-hset ((p <prop-abstract-planning-problem>) hset a)
  (successor-set
   (ecase (action-type a (hierarchy p))
     (high-level (hset-desc p a))
     (primitive (primitive-action-description (planning-problem p) a)))
   hset))

(defmethod hset-initial-value ((p <prop-abstract-planning-problem>) s)
  (make-state-set (planning-problem p) s))

  
  
  
  