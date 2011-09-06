
(in-package :asdf)

(defsystem "mst_common-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :roslib-msg
)
  :components ((:file "_package")
    (:file "Raytrace" :depends-on ("_package"))
    (:file "_package_Raytrace" :depends-on ("_package"))
    (:file "Filter" :depends-on ("_package"))
    (:file "_package_Filter" :depends-on ("_package"))
    (:file "IMU" :depends-on ("_package"))
    (:file "_package_IMU" :depends-on ("_package"))
    (:file "VisionFilter" :depends-on ("_package"))
    (:file "_package_VisionFilter" :depends-on ("_package"))
    (:file "Velocity" :depends-on ("_package"))
    (:file "_package_Velocity" :depends-on ("_package"))
    (:file "ImageFilter" :depends-on ("_package"))
    (:file "_package_ImageFilter" :depends-on ("_package"))
    ))
