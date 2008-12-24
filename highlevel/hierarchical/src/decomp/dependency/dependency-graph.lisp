(in-package :dependency-graph)
  
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Types
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Info about a single variable in the dependancy graph
(defstruct (var-desc (:conc-name nil))
  var-type
  (saved-update-state nil)
  update-fn
  (num-ties 0)
  update-hooks
  current-val)

(defclass <dependency-graph> ()
  ((var-descs :reader var-descs :initform (make-hash-table :test #'equal))
   (graph :reader graph :initform (make-instance '<adjacency-list-graph> :node-test #'equal))
   (id :initform (gensym) :initarg :id :reader dep-graph-id)
   (uninitialized-value :reader uninitialized-value :initform (gensym))
   (out-of-date-vars :initform (make-hash-table :test #'equal) :reader out-of-date-vars)))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; API
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun make-dependency-graph ()
  "make-dependency-graph
Make an empty dependency graph"
  (make-instance '<dependency-graph>))

(defun add-variable (g name type &key update-fn simple-update-fn dependees dependants update-hooks (initial-value nil val-given))
  "add-variable DEPENDENCY-GRAPH NAME TYPE &key (UPDATE-FN nil) (DEPENDEES nil) (DEPENDANTS nil) (UPDATE-HOOKS nil) INITIAL-VALUE
Add a new variable to the graph.  

NAME is of a type that can be distinguished using #'equal.  
TYPE is either :internal or :external.  
DEPENDS-ON and DEPENDANTS are lists of names.  

UPDATE-FN is used to implement the idea of a steppable update function.  It receives three arguments: 1) An alist from parent-name to current value 2) An alist from parent name to the diff since the last seen value 3) The old value of the variable 4) The saved state from the last invocation of UPDATE-FN for this variable.  It must return 1) The new value of the variable 2) The diff from the previous value 3) The new saved state 4) A boolean that is t iff the computation of the variable's new value is complete.
SIMPLE-UPDATE-FN can be provided instead of UPDATE-FN, and calls simple-update-fn on its value and uses that as the update function.

UPDATE-HOOKS is a list of functions that will be called whenever the variable is updated (if a sequence of updates happens, they will be called once after all the updates), on the new value and diff.  The functions are expected to not have side effects within the dependency graph.  

If INITIAL-VALUE is supplied, it's used to initialize the variable, which must be external in this case.  Else it's uninitialized.  Also, when an internal variable is added, it and all its descendants become out of date, while if an external variable is added, its descendants become out of date."

  (assert (ecase type
	    (:internal (xor update-fn simple-update-fn))
	    (:external (and (not update-fn) (not simple-update-fn) (null dependees)))))
  (debug-out :decomp 0 t "~&Adding variable ~a of type ~a with dependees ~a and dependants ~a to dependency-graph ~a" name type dependees dependants (dep-graph-id g))
  
  (when simple-update-fn (setf update-fn (make-simple-update-fn simple-update-fn)))

  ;; Add this variable's description
  (let ((descs (var-descs g)))
    (assert (not (hash-table-has-key descs name)) nil
	    "Variable named ~a already exists with description ~a" name (gethash name descs))
    (setf (gethash name descs) (make-var-desc :update-fn update-fn :update-hooks update-hooks :current-val (uninitialized-value g) :var-type type)))
  (when (eq type :internal)
    (setf (up-to-date? name g) nil))

  ;; Update dependencies
  ;; TODO: check that a directed cycle is not introduced
  (add-node (graph g) name)
  (dolist (v dependees)
    (add-dependency g v name))
  (dolist (v dependants)
    (add-dependency g name v))

  (when val-given 
    (assert (eq type :external) nil "Attempted to provide initial value ~a for internal variable ~a" initial-value name)
    (update-external-variable g name (new-val-diff initial-value))))
    


(defun add-dependency (g v1 v2)
  (add-edge (graph g) v1 v2 :label nil) ;; nil label represents the no-op diff
  (propagate-out-of-date v2 g))


(defun update-external-variable (g v d)
  "update-external-variable DEPENDENCY-GRAPH VAR DIFF
VAR must be external.
After this, all descendants of VAR will be out of date."
  (assert (not (internal? v g)) nil "Can't directly set value of internal variable ~a" v)
  (let ((new-val (apply-diff d (current-val (get-var-desc v g)))))
    (change-variable v new-val d g)))

(defun up-to-date? (v g)
  "up-to-date? V G.  Is V either an initialized external variable, or an internal variable that is up-to-date given the values of its ancestors (note that in the second case, it is guaranteed to be initialized)"
  (if (internal? v g)
    (not (hash-table-has-key (out-of-date-vars g) v))
    (initialized? v g)))

(defun do-all-updates (g)
  "do-all-updates G.  Make all internal variables in G up-to-date.  Error will happen if any external variables (with children) are uninitialized."
  (loop
    (unless (do-next-update g t) (return))))

(defun do-next-update (g &optional (fully-update nil))
  "do-next-update GRAPH &optional (FULLY-UPDATE nil)
If there exist out-of-date variables, update one of them and return 1) Its name 2) Its new value.  Else return nil.  If there exist uninitialized external variables, an error may happen (if the error doesn't happen, a correct update will occur)."
  (mvbind (v exists?) (get-out-of-date-var g)
    (when exists?
      (update-ancestor v g fully-update))))

(defun current-value (g v)
  "Current value of variable V.  This might be out of date.  If V is uninitialized, signal an error."
  (assert (initialized? v g) nil "Can't get value of uninitialized variable ~a of ~a" v g)
  (current-val (get-var-desc v g)))

(defun up-to-date-value (g v)
  "Make variable up-to-date and return its value.  An error will happen if V is an uninitialized external variable."
  (make-up-to-date g v)
  (current-value g v))

(defun tie-variables (g1 v1 g2 v2)
  "tie-variables G1 V1 G2 V2
Tie variables across dependency graphs: add a hook such that when you update variable V1 in graph G1 in future, variable V2 in graph G2 receives a new diff as well.  V2 must be an external variable in G2, and not be already tied to something else.

If V1 is initialized, V2 will get its initial value.  Otherwise, V2 will be uninitialized."
  (assert (not (internal? v2 g2)) nil "Can't tie variable ~a in ~a to internal variable ~a in ~a." v1 g1 v2 g2)
  (assert (not (eq g1 g2)) nil "Can't tie variables ~a and ~a in the same graph" v1 v2 g1)
  (let ((d (get-var-desc v2 g2)))
    (assert (zerop (num-ties d)) nil "Variable ~a in ~a already tied." v2 g2)
    (incf (num-ties d)))
  (push #'(lambda (new-val diff) (declare (ignore new-val)) (update-external-variable g2 v2 diff))
	(update-hooks (get-var-desc v1 g1)))
  (when (initialized? v1 g1)
    (update-external-variable g2 v2 (new-val-diff (current-value g1 v1)))))

(defun out-of-date-variables (g)
  "Return list of out-of-date internal variables"
  (hash-keys (out-of-date-vars g)))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; helpers
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun initialized? (v g)
  "Has variable V been given a value?"
  (not (eq (current-val (get-var-desc v g)) (uninitialized-value g))))

(defun update-ancestor (v g update-fully)
  "Assumes V is out of date.  Finds an ancestor that is out-of-date, but whose parents are up-to-date, and updates that ancestor."
  (assert (not (up-to-date? v g)))
  (loop
    (let ((out-of-date-parent (do-parents (p v g)
				(when (and (internal? p g) (not (up-to-date? p g)))
				  (return p)))))
      (if out-of-date-parent
	(setq v out-of-date-parent)
	  ;; else
	(return (values v (update-from-parents v g update-fully)))))))

(defun update-from-parents (v g update-fully)
  (assert (internal? v g) nil "Can't update variable ~a automatically - it must be externally set." v)
  (let ((desc (get-var-desc v g))
	(overall-diff nil)
	(parent-diffs nil)
	(parent-vals nil))
    (do-elements (e (edges-to (graph g) v))
      (let ((parent (source e)))
	(assert (up-to-date? parent g))
	(push (cons parent (edge-label e)) parent-diffs)
	(push (cons parent (current-value g parent)) parent-vals)
	(setf (edge-label e) nil)))
    

    (debug-out :dep-graph 1 t "~&Updating variable ~a of dep graph ~a~& Value: ~a~& Parents: ~a~& Parent-vals: ~a" 
	       v (dep-graph-id g) (current-val desc) (nreverse (to-list (parents (graph g) v))) parent-vals)


    (loop
      (mvbind (new-val diff saved-state done?) (funcall (update-fn desc) parent-vals parent-diffs (current-val desc) (saved-update-state desc))
	(setf (up-to-date? v g) done?
	      (saved-update-state desc) saved-state
	      overall-diff (compose-diffs diff overall-diff))
	(unless (and update-fully (not done?))
	  (change-variable v new-val overall-diff g)
	  (return new-val))))))

(defun change-variable (v new-val diff g)
  (let ((desc (get-var-desc v g)))
    (setf (current-val desc) new-val)
    (debug-out :dep-graph 1 t "~&Changing variable ~a in dep graph ~a to ~a" v (dep-graph-id g) new-val)
    (propagate-diff v g diff)
    (dolist (h (update-hooks desc))
      (funcall h new-val diff))))

(defun propagate-diff (v g diff)
  (do-elements (e (edges-from (graph g) v))
    (setf (edge-label e) (compose-diffs diff (edge-label e)))
    (propagate-out-of-date (dest e) g)))

(defun propagate-out-of-date (v g)
  (when (up-to-date? v g)
    (setf (up-to-date? v g) nil)
    (do-children (c v g)
      (propagate-out-of-date c g))))

(defun variables (g)
  (hash-keys (var-descs g)))

(defun make-up-to-date (g v)
  (unless (up-to-date? v g)
    (assert (internal? v g) nil "Can't make external variable ~a of ~a up-to-date" v g)
    (do-parents (p v g)
      (make-up-to-date g p))
    (update-from-parents v g t)))

(defun get-var-desc (v g)
  (mapping:evaluate (var-descs g) v))

(defun (setf up-to-date?) (new-val var g)
  (assert (internal? var g) nil "Can't setf the up-to-date? property for external variable ~a of ~a" var g)
  (let ((table (out-of-date-vars g)))
    (if (hash-table-has-key table var)
      (when new-val
	(remhash var table)
	new-val)
      (or new-val (progn (setf (gethash var table) t) nil)))))

(defun get-out-of-date-var (g)
  "If there exist out of date internal variables, return one of them and t, else return nil."
  (awhen (hash-keys (out-of-date-vars g))
    (values (first it) t)))

(defun internal? (v g)
  (eq :internal (var-type (get-var-desc v g))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; debug
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun print-dep-graph (&rest args)
  (bind-pprint-args (str g) args
    (pprint-logical-block 
     (str nil :prefix "[" :suffix "]")
     (format str "Dep graph ~a~:@_ Graph: ~a~:@_" (dep-graph-id g) (graph g))
     (do-hash (var val (var-descs g))
       (format str " ~a: ~a~:@_" var val)))))

(set-pprint-dispatch '<dependency-graph> #'print-dep-graph)