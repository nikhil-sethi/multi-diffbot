
(cl:in-package :asdf)

(defsystem "mirte_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :std_msgs-msg
)
  :components ((:file "_package")
    (:file "Encoder" :depends-on ("_package_Encoder"))
    (:file "_package_Encoder" :depends-on ("_package"))
    (:file "Intensity" :depends-on ("_package_Intensity"))
    (:file "_package_Intensity" :depends-on ("_package"))
    (:file "IntensityDigital" :depends-on ("_package_IntensityDigital"))
    (:file "_package_IntensityDigital" :depends-on ("_package"))
    (:file "Keypad" :depends-on ("_package_Keypad"))
    (:file "_package_Keypad" :depends-on ("_package"))
    (:file "color" :depends-on ("_package_color"))
    (:file "_package_color" :depends-on ("_package"))
  ))