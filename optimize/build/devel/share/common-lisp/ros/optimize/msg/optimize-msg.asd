
(cl:in-package :asdf)

(defsystem "optimize-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
)
  :components ((:file "_package")
    (:file "Path" :depends-on ("_package_Path"))
    (:file "_package_Path" :depends-on ("_package"))
  ))