
(cl:in-package :asdf)

(defsystem "armc_controller-srv"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "SetArmcConfigure" :depends-on ("_package_SetArmcConfigure"))
    (:file "_package_SetArmcConfigure" :depends-on ("_package"))
  ))