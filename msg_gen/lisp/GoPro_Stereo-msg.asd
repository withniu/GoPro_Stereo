
(cl:in-package :asdf)

(defsystem "GoPro_Stereo-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "pos2d" :depends-on ("_package_pos2d"))
    (:file "_package_pos2d" :depends-on ("_package"))
    (:file "pos3d" :depends-on ("_package_pos3d"))
    (:file "_package_pos3d" :depends-on ("_package"))
  ))