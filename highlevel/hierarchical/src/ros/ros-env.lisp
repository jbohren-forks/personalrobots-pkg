(defpackage :ros-env
  (:use :cl :create-env :utils :sb-thread :roslisp)
  (:export
   :setup-ros-connection))

(in-package :ros-env)


(defclass <ros-env> (<fully-observable-env>)
  ;; TODO: it's not actually fully-observable in general.  Just need to go through code and look for places that rely on full observability.
  ((ros-connection-status :initform :not-connected :accessor status))
  (:documentation "Represents an environment that we interact with over ROS.  Basic picture is that we will publish commands and subscribe to a single state topic that comes in at a fairly constant frequency."))



(defmethod do-action ((e <ros-env>) a)
  (assert (connected-to-env-over-ros e))
  (values (send-action-over-ros e a) (get-state e) nil))



(defun setup-ros-connection (e state-topic-name state-topic-type state-message-creator)
  "Advertises on the command topic and sets up a subscription callback that just takes the state message and fills it into the state using the get-new-state function."
  (assert (eq (node-status) :running) nil "Can't set up ros environment until ROS node is running")
  (subscribe state-topic-name state-topic-type #'(lambda (m) (set-state e (funcall state-message-creator m))))
  (setf (status e) ':connected))




;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Internal
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(defun connected-to-env-over-ros (e)
  (and (eq (status e) :connected) (eq (node-status) :running)))

(defun send-action-over-ros (e a)
  (declare (ignore e a))
  (error 'not-yet-implemented))