
(cl:in-package :asdf)

(defsystem "bullproof_nav-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :actionlib_msgs-msg
               :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "NavPose2DAction" :depends-on ("_package_NavPose2DAction"))
    (:file "_package_NavPose2DAction" :depends-on ("_package"))
    (:file "NavPose2DActionFeedback" :depends-on ("_package_NavPose2DActionFeedback"))
    (:file "_package_NavPose2DActionFeedback" :depends-on ("_package"))
    (:file "NavPose2DActionGoal" :depends-on ("_package_NavPose2DActionGoal"))
    (:file "_package_NavPose2DActionGoal" :depends-on ("_package"))
    (:file "NavPose2DActionResult" :depends-on ("_package_NavPose2DActionResult"))
    (:file "_package_NavPose2DActionResult" :depends-on ("_package"))
    (:file "NavPose2DFeedback" :depends-on ("_package_NavPose2DFeedback"))
    (:file "_package_NavPose2DFeedback" :depends-on ("_package"))
    (:file "NavPose2DGoal" :depends-on ("_package_NavPose2DGoal"))
    (:file "_package_NavPose2DGoal" :depends-on ("_package"))
    (:file "NavPose2DResult" :depends-on ("_package_NavPose2DResult"))
    (:file "_package_NavPose2DResult" :depends-on ("_package"))
  ))