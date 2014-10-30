
(cl:in-package :asdf)

(defsystem "k2_client-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "BodyArray" :depends-on ("_package_BodyArray"))
    (:file "_package_BodyArray" :depends-on ("_package"))
    (:file "JointOrientationAndType" :depends-on ("_package_JointOrientationAndType"))
    (:file "_package_JointOrientationAndType" :depends-on ("_package"))
    (:file "Body" :depends-on ("_package_Body"))
    (:file "_package_Body" :depends-on ("_package"))
    (:file "Activities" :depends-on ("_package_Activities"))
    (:file "_package_Activities" :depends-on ("_package"))
    (:file "Appearance" :depends-on ("_package_Appearance"))
    (:file "_package_Appearance" :depends-on ("_package"))
    (:file "Audio" :depends-on ("_package_Audio"))
    (:file "_package_Audio" :depends-on ("_package"))
    (:file "JointPositionAndState" :depends-on ("_package_JointPositionAndState"))
    (:file "_package_JointPositionAndState" :depends-on ("_package"))
    (:file "Lean" :depends-on ("_package_Lean"))
    (:file "_package_Lean" :depends-on ("_package"))
    (:file "Expressions" :depends-on ("_package_Expressions"))
    (:file "_package_Expressions" :depends-on ("_package"))
  ))