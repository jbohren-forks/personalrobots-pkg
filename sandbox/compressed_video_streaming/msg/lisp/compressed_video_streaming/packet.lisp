; Auto-generated. Do not edit!


(in-package compressed_video_streaming-msg)


;//! \htmlinclude packet.msg.html

(defclass <packet> (ros-message)
  ((blob
    :accessor blob-val
    :initarg :blob
    :initform #())
   (bytes
    :accessor bytes-val
    :initarg :bytes
    :initform 0)
   (b_o_s
    :accessor b_o_s-val
    :initarg :b_o_s
    :initform 0)
   (e_o_s
    :accessor e_o_s-val
    :initarg :e_o_s
    :initform 0)
   (granulepos
    :accessor granulepos-val
    :initarg :granulepos
    :initform 0)
   (packetno
    :accessor packetno-val
    :initarg :packetno
    :initform 0))
)
(defmethod serialize ((msg <packet>) ostream)
  "Serializes a message object of type '<packet>"
  (let ((__ros_arr_len (length (slot-value msg 'blob))))
    (write-byte (ldb (byte 8 0) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 8) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 16) __ros_arr_len) ostream)
    (write-byte (ldb (byte 8 24) __ros_arr_len) ostream))
  (map nil #'(lambda (ele)   (write-byte (ldb (byte 8 0) ele) ostream))
    (slot-value msg 'blob))
    (write-byte (ldb (byte 8 0) (slot-value msg 'bytes)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'bytes)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'bytes)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'bytes)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'b_o_s)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'b_o_s)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'b_o_s)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'b_o_s)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'e_o_s)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'e_o_s)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'e_o_s)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'e_o_s)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'granulepos)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'granulepos)) ostream)
    (write-byte (ldb (byte 8 0) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 8) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 16) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 24) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 32) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 40) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 48) (slot-value msg 'packetno)) ostream)
  (write-byte (ldb (byte 8 56) (slot-value msg 'packetno)) ostream)
)
(defmethod deserialize ((msg <packet>) istream)
  "Deserializes a message object of type '<packet>"
  (let ((__ros_arr_len 0))
    (setf (ldb (byte 8 0) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 8) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 16) __ros_arr_len) (read-byte istream))
    (setf (ldb (byte 8 24) __ros_arr_len) (read-byte istream))
    (setf (slot-value msg 'blob) (make-array __ros_arr_len))
    (let ((vals (slot-value msg 'blob)))
      (dotimes (i __ros_arr_len)
(setf (ldb (byte 8 0) (aref vals i)) (read-byte istream)))))
  (setf (ldb (byte 8 0) (slot-value msg 'bytes)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'bytes)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'bytes)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'bytes)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'b_o_s)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'b_o_s)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'b_o_s)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'b_o_s)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'e_o_s)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'e_o_s)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'e_o_s)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'e_o_s)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'granulepos)) (read-byte istream))
  (setf (ldb (byte 8 0) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 8) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 16) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 24) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 32) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 40) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 48) (slot-value msg 'packetno)) (read-byte istream))
  (setf (ldb (byte 8 56) (slot-value msg 'packetno)) (read-byte istream))
  msg
)
(defmethod ros-datatype ((msg (eql '<packet>)))
  "Returns string type for a message object of type '<packet>"
  "compressed_video_streaming/packet")
(defmethod md5sum ((type (eql '<packet>)))
  "Returns md5sum for a message object of type '<packet>"
  #xd804434eb295ca184dd6d2e32479185c)
(defmethod message-definition ((type (eql '<packet>)))
  "Returns full string definition for message of type '<packet>"
  (format nil "uint8[] blob~%int32 bytes~%int32 b_o_s~%int32 e_o_s~%~%int64 granulepos  ~%int64 packetno~%~%"))
(defmethod serialization-length ((msg <packet>))
  (+ 0
     4 (reduce #'+ (slot-value msg 'blob) :key #'(lambda (ele) (declare (ignorable ele)) (+ 1)))
     4
     4
     4
     8
     8
))
(defmethod ros-message-to-list ((msg <packet>))
  "Converts a ROS message object to a list"
  (list '<packet>
    (cons ':blob (ros-message-to-list (blob-val msg)))
    (cons ':bytes (ros-message-to-list (bytes-val msg)))
    (cons ':b_o_s (ros-message-to-list (b_o_s-val msg)))
    (cons ':e_o_s (ros-message-to-list (e_o_s-val msg)))
    (cons ':granulepos (ros-message-to-list (granulepos-val msg)))
    (cons ':packetno (ros-message-to-list (packetno-val msg)))
))
