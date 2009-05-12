; Auto-generated. Do not edit!

(roslisp:load-if-necessary "/u/pantofaru/Install/ros/ros-pkg/vision/people/msg/lisp/people/_package.lisp")
(roslisp:load-if-necessary "/u/pantofaru/Install/ros/ros/core/roslib/msg/lisp/roslib/Header.lisp")
(roslisp:load-if-necessary "/u/pantofaru/Install/ros/ros-pkg/common/robot_msgs/msg/lisp/robot_msgs/Point.lisp")

(in-package people)


;//! \htmlinclude PositionMeasurement.msg.html

(defclass <PositionMeasurement> (ros-message)
  ((header
    :accessor header-val
    :initarg :header
    :initform (make-instance 'roslib:<Header>))
   (name
    :accessor name-val
    :initarg :name
    :initform "")
   (object_id
    :accessor object_id-val
    :initarg :object_id
    :initform "")
   (pos
    :accessor pos-val
    :initarg :pos
    :initform (make-instance 'robot_msgs:<Point>))
   (reliability
    :accessor reliability-val
    :initarg :reliability
    :initform 0.0)
   (covariance
    :accessor covariance-val
    :initarg :covariance
    :initform #())
   (initialization
    :accessor initialization-val
    :initarg :initialization
    :initform 0))
)
(defmethod serialize ((msg <PositionMeasurement>) ostream)
  "Serializes a message object of type '<PositionMeasurement>"
  (serialize (slot-value msg 'header) ostream)
  (let ((__ros_str_len (length (slot-value msg 'name))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'name))
  (let ((__ros_str_len (length (slot-value msg 'object_id))))
    (write-byte (ldb (byte 8 0) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_str_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_str_len) ostream))
  (map nil #'(lambda (c) (write-byte (char-code c) ostream)) (slot-value msg 'object_id))
  (serialize (slot-value msg 'pos) ostream)
  (let ((bits (roslisp-utils:encode-double-float-bits (slot-value msg 'reliability))))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))
  (let ((__ros_arr_len (length (slot-value msg 'covariance))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele) (let ((bits (roslisp-utils:encode-double-float-bits ele)))
    (write-byte (ldb (byte 8 0) bits) ostream)
    (write-byte (ldb (byte 8 8) bits) ostream)
    (write-byte (ldb (byte 8 16) bits) ostream)
    (write-byte (ldb (byte 8 24) bits) ostream)
    (write-byte (ldb (byte 8 32) bits) ostream)
    (write-byte (ldb (byte 8 40) bits) ostream)
    (write-byte (ldb (byte 8 48) bits) ostream)
    (write-byte (ldb (byte 8 56) bits) ostream))) (slot-value msg 'covariance))
    (write-byte (ldb (byte 8 0) (slot-value msg 'initialization)) ostream)
)
(defmethod deserialize ((msg <PositionMeasurement>) istream)
  "Deserializes a message object of type '<PositionMeasurement>"
  (deserialize (slot-value msg 'header) istream)
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'name) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'name) __ros_str_idx) (code-char (read-byte istream)))))
  (let ((__ros_str_len 0))
    (setf (ldb (byte 8 0) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_str_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_str_len) (read-byte istream))
    (setf (slot-value msg 'object_id) (make-string __ros_str_len))
    (dotimes (__ros_str_idx __ros_str_len msg)
      (setf (char (slot-value msg 'object_id) __ros_str_idx) (code-char (read-byte istream)))))
  (deserialize (slot-value msg 'pos) istream)
  (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (slot-value msg 'reliability) (roslisp-utils:decode-double-float-bits bits)))
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'covariance) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'covariance)))
      (dotimes (i __ros_arr_len)
        (setf (aref vals i) (let ((bits 0))
    (setf (ldb (byte 8 0) bits) (read-byte istream))
    (setf (ldb (byte 8 8) bits) (read-byte istream))
    (setf (ldb (byte 8 16) bits) (read-byte istream))
    (setf (ldb (byte 8 24) bits) (read-byte istream))
    (setf (ldb (byte 8 32) bits) (read-byte istream))
    (setf (ldb (byte 8 40) bits) (read-byte istream))
    (setf (ldb (byte 8 48) bits) (read-byte istream))
    (setf (ldb (byte 8 56) bits) (read-byte istream))
    (setf (aref vals i) (roslisp-utils:decode-double-float-bits bits)))))))
  (setf (ldb (byte 8 0) (slot-value msg 'initialization)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<PositionMeasurement>)))
  "Returns string type for a message object of type '<PositionMeasurement>"
  "people/PositionMeasurement")
(defmethod md5sum ((type (eql '<PositionMeasurement>)))
  "Returns md5sum for a message object of type '<PositionMeasurement>"
  #x9da2997e69ab83d26b742b4c5d4d0d04)
(defmethod message-definition ((type (eql '<PositionMeasurement>)))
  "Returns full string definition for message of type '<PositionMeasurement>"
  (format nil "Header          header~%string          name~%string          object_id~%robot_msgs/Point  pos~%float64         reliability~%float64[9]      covariance~%byte            initialization~%================================================================================~%MSG: roslib/Header~%#Standard metadata for higher-level flow data types~%#sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: robot_msgs/Point~%float64 x~%float64 y~%float64 z~%~%~%~%"))
(defmethod serialization-length ((msg <PositionMeasurement>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     4 (length (slot-value msg 'name))
     4 (length (slot-value msg 'object_id))
     (serialization-length (slot-value msg 'pos))
     8
     (* 9 8)
     1
))
(defmethod ros-message-to-list ((msg <PositionMeasurement>))
  "Converts a ROS message object to a list"
  (list '<PositionMeasurement>
    (cons ':header (ros-message-to-list (header-val msg)))
    (cons ':name (ros-message-to-list (name-val msg)))
    (cons ':object_id (ros-message-to-list (object_id-val msg)))
    (cons ':pos (ros-message-to-list (pos-val msg)))
    (cons ':reliability (ros-message-to-list (reliability-val msg)))
    (cons ':covariance (ros-message-to-list (covariance-val msg)))
    (cons ':initialization (ros-message-to-list (initialization-val msg)))
))
