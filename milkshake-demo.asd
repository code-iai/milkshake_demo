; -*- Mode: Lisp; Syntax: ANSI-Common-Lisp; Base: 10 -*-

(asdf:defsystem milkshake-demo
  :name "milkshake-demo"
  :author "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :maintainer "Mihai Pomarlan <blandc@cs.uni-bremen.com>"
  :licence "BSD"
  :description "Play with blender. Make milkshake."
  :depends-on (:alexandria
               :visualization_msgs-msg
               :ros-load-manifest
               :roslisp-utilities
               :roslisp
               :prac2cram
               :cram-tf
               :actionlib
               :cram-prolog
               :cram-pr2-description
               :trivial-garbage
               :semantic-map-collision-environment
               :cram-executive
               :cram-pr2-pick-place-plans
               :cram-pr2-process-modules
               :pr2-reachability-costmap)
  :components
  ((:module "src"
            :components
            ((:file "package")
             (:file "poses" :depends-on ("package"))
             (:file "markers" :depends-on ("poses"))
             (:file "arms" :depends-on ("package"))
             (:file "kinematic-controllability" ("poses" "arms" "markers"))
             (:file "physical-exposure" :depends-on ("kinematic-controllability"))
             (:file "demo" :depends-on ("physical-exposure"))))))
