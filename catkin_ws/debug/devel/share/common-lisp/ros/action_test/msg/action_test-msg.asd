
(cl:in-package :asdf)

(defsystem "action_test-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "action_msg1" :depends-on ("_package_action_msg1"))
    (:file "_package_action_msg1" :depends-on ("_package"))
  ))