
(in-package :asdf)

(defsystem "compressed_video_streaming-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "binaryblob" :depends-on ("_package"))
    (:file "_package_binaryblob" :depends-on ("_package"))
    (:file "packet" :depends-on ("_package"))
    (:file "_package_packet" :depends-on ("_package"))
    ))
