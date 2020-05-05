
(cl:in-package :asdf)

(defsystem "action_test-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "action_srv1" :depends-on ("_package_action_srv1"))
    (:file "_package_action_srv1" :depends-on ("_package"))
  ))