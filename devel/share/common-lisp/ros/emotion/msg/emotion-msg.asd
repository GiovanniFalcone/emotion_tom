
(cl:in-package :asdf)

(defsystem "emotion-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "emotion" :depends-on ("_package_emotion"))
    (:file "_package_emotion" :depends-on ("_package"))
  ))