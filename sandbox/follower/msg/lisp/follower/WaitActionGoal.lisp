; Auto-generated. Do not edit!

(roslisp:load-if-necessary "/u/ethand/ros/ros-pkg/sandbox/follower/msg/lisp/follower/_package.lisp")

(in-package follower)


;//! \htmlinclude WaitActionGoal.msg.html

(defclass <WaitActionGoal> (ros-message)
  ((num_events
    :accessor num_events-val
    :initarg :num_events
    :initform 0)
   (topic_name
    :accessor topic_name-val
    :initarg :topic_name
    :initform ""))
)
(defmethod serialize ((msg <WaitActionGoal>) ostream)
  "Serializes a message object of type '<WaitActionGoal>"
    (write-byte (ldb (byte 8 0) (slot-value msg 'num_events)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'num_events)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'num_events)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'num_events)) ostream)
  (let ((__ros_str_len (length (slot-value msg 'topic_name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'topic_name))
)
(defmethod deserialize ((msg <WaitActionGoal>) istream)
  "Deserializes a message object of type '<WaitActionGoal>"
  (setf (ldb (byte 8 0) (slot-value msg 'num_events)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'num_events)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'num_events)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'num_events)) (read-byte istream))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'topic_name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'topic_name) __ros_str_idx) (code-char (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<WaitActionGoal>)))
  "Returns string type for a message object of type '<WaitActionGoal>"
  "follower/WaitActionGoal")
(defmethod md5sum ((type (eql '<WaitActionGoal>)))
  "Returns md5sum for a message object of type '<WaitActionGoal>"
  #x54f5dc6d242ed96aa3e20c82006143e4)
(defmethod message-definition ((type (eql '<WaitActionGoal>)))
  "Returns full string definition for message of type '<WaitActionGoal>"
  (format nil "int32 num_events~%string topic_name~%~%~%~%"))
(defmethod serialization-length ((msg <WaitActionGoal>))
  (+ 0
     4
     4 (length (slot-value msg 'topic_name))
))
(defmethod ros-message-to-list ((msg <WaitActionGoal>))
  "Converts a ROS message object to a list"
  (list '<WaitActionGoal>
    (cons ':num_events (ros-message-to-list (num_events-val msg)))
    (cons ':topic_name (ros-message-to-list (topic_name-val msg)))
))
