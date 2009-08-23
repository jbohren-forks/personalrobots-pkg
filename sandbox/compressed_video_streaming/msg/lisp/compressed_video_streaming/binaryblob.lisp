; Auto-generated. Do not edit!


(in-package compressed_video_streaming-msg)


;//! \htmlinclude binaryblob.msg.html

(defclass <binaryblob> (ros-message)
  ((blob
    :accessor blob-val
    :initarg :blob
    :initform #()))
)
(defmethod serialize ((msg <binaryblob>) ostream)
  "Serializes a message object of type '<binaryblob>"
  (let ((__ros_arr_len (length (slot-value msg 'blob))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'blob))
)
(defmethod deserialize ((msg <binaryblob>) istream)
  "Deserializes a message object of type '<binaryblob>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'blob) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'blob)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  msg
)
(defmethod ros-datatype ((msg (eql '<binaryblob>)))
  "Returns string type for a message object of type '<binaryblob>"
  "compressed_video_streaming/binaryblob")
(defmethod md5sum ((type (eql '<binaryblob>)))
  "Returns md5sum for a message object of type '<binaryblob>"
  #x686a5a6faa4b2c7d2070ef2a260d09e7)
(defmethod message-definition ((type (eql '<binaryblob>)))
  "Returns full string definition for message of type '<binaryblob>"
  (format nil "uint8[] blob~%~%"))
(defmethod serialization-length ((msg <binaryblob>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'blob) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
))
(defmethod ros-message-to-list ((msg <binaryblob>))
  "Converts a ROS message object to a list"
  (list '<binaryblob>
    (cons ':blob (ros-message-to-list (blob-val msg)))
))
