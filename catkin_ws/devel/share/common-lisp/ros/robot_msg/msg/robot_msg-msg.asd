
(cl:in-package :asdf)

(defsystem "robot_msg-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "IpPos" :depends-on ("_package_IpPos"))
    (:file "_package_IpPos" :depends-on ("_package"))
  ))