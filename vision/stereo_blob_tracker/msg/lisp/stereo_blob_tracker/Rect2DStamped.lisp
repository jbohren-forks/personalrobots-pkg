(roslisp:load-if-necessary "/u/jdchen/workspace/ros-pkg/vision/stereo_blob_tracker/msg/lisp/stereo_blob_tracker/_package.lisp")
(roslisp:load-if-necessary "/u/jdchen/workspace/ros/core/rostools/msg/lisp/rostools/Header.lisp")
(roslisp:load-if-necessary "/u/jdchen/workspace/ros-pkg/vision/stereo_blob_tracker/msg/lisp/stereo_blob_tracker/Rect2D.lisp")

(in-package stereo_blob_tracker)


;//! \htmlinclude Rect2DStamped.msg.html

(defclass <Rect2DStamped> (ros-message)
  ((header
    :accessor header-val
    :initarg :header
    :initform (make-instance 'rostools:<Header>))
   (rect
    :accessor rect-val
    :initarg :rect
    :initform (make-instance 'stereo_blob_tracker:<Rect2D>)))
)
(defmethod serialize ((msg <Rect2DStamped>) ostream)
  "Serializes a message object of type '<Rect2DStamped>"
  (serialize (slot-value msg 'header) ostream)
  (serialize (slot-value msg 'rect) ostream)
)
(defmethod deserialize ((msg <Rect2DStamped>) istream)
  "Deserializes a message object of type '<Rect2DStamped>"
  (deserialize (slot-value msg 'header) istream)
  (deserialize (slot-value msg 'rect) istream)
  msg
)
(defmethod md5sum ((type (eql '<Rect2DStamped>)))
  "Returns md5sum for a message object of type '<Rect2DStamped>"
  #x89499fc21647a323f3afb6d0c46b57fa)
(defmethod serialization-length ((msg <Rect2DStamped>))
  (+ 0
     (serialization-length (slot-value msg 'header))
     (serialization-length (slot-value msg 'rect))
))
(defmethod ros-message-to-list ((msg <Rect2DStamped>))
  "Converts a ROS message object to a list"
  (list '<Rect2DStamped>
    (cons ':header (ros-message-to-list (header-val msg)))
    (cons ':rect (ros-message-to-list (rect-val msg)))
))
