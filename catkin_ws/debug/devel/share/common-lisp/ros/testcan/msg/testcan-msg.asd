
(cl:in-package :asdf)

(defsystem "testcan-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Frame" :depends-on ("_package_Frame"))
    (:file "_package_Frame" :depends-on ("_package"))
    (:file "IpPos" :depends-on ("_package_IpPos"))
    (:file "_package_IpPos" :depends-on ("_package"))
  ))