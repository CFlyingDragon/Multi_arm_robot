
(cl:in-package :asdf)

(defsystem "armc_visual-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "AddTwoInts" :depends-on ("_package_AddTwoInts"))
    (:file "_package_AddTwoInts" :depends-on ("_package"))
    (:file "VisualVar" :depends-on ("_package_VisualVar"))
    (:file "_package_VisualVar" :depends-on ("_package"))
  ))