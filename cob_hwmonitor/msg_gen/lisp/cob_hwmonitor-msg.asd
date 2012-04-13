
(cl:in-package :asdf)

(defsystem "cob_hwmonitor-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "hw_msg" :depends-on ("_package_hw_msg"))
    (:file "_package_hw_msg" :depends-on ("_package"))
  ))