;;;
;;; Copyright (c) 2018, Mihai Pomarlan <blandc@cs.uni-bremen.com>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Willow Garage, Inc. nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.
;;;

(in-package :milkshake-demo)

(defparameter *pub-mrk* nil)
(defparameter *RVIZ-ADD-MARKER* 0)
(defparameter *RVIZ-DEL-MARKER* 2)
(defparameter *RVIZ-DEL-ALL-MARKER* 3)

(defparameter *strawberry-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.9 :g 0.1 :b 0.1))
(defparameter *banana-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.9 :g 0.9 :b 0.1))
(defparameter *object-color* (roslisp:make-message "std_msgs/ColorRGBA" :a 1 :r 0.8 :g 0.7 :b 0.3))

(defparameter *object-names* `("banana" "blender-device" "blender-bowl" "bowl" "milk-carton" "mug" "strawberry"))

(defparameter *marker-object-mesh-paths* `(("banana" . "package://milkshake_demo/models/banana/meshes/banana.dae")
                                           ("blender-device" . "package://milkshake_demo/models/blender_device/meshes/blender_device.dae")
                                           ("blender-bowl" . "package://milkshake_demo/models/blender_bowl/meshes/blender_bowl.dae")
                                           ("bowl" . "package://milkshake_demo/models/bowl/meshes/bowl.dae")
                                           ("milk-carton" . "package://milkshake_demo/models/milk_carton/meshes/milk_carton.dae")
                                           ("mug" . "package://milkshake_demo/models/mug/meshes/mug.dae")
                                           ("strawberry" . "package://milkshake_demo/models/strawberry/meshes/strawberry.dae")))

(defparameter *visualization-topic* "/visualization_marker")

(defun ensure-mrk-publisher ()
  (if *pub-mrk*
    *pub-mrk*
    (progn
      (setf *pub-mrk* (roslisp:advertise *visualization-topic* "visualization_msgs/Marker" :latch nil))
      (roslisp:wait-duration 2.0)
      *pub-mrk*)))

(defun destroy-mrk-publisher ()
  (setf *pub-mrk* nil))

(roslisp-utilities:register-ros-cleanup-function destroy-mrk-publisher)

(defun tr->ps (transform)
  (let* ((v (cl-transforms:translation transform))
         (r (cl-transforms:rotation transform))
         (x (cl-transforms:x v))
         (y (cl-transforms:y v))
         (z (cl-transforms:z v))
         (qx (cl-transforms:x r))
         (qy (cl-transforms:y r))
         (qz (cl-transforms:z r))
         (qw (cl-transforms:w r)))
    (roslisp:make-message "geometry_msgs/Pose"
      :position (roslisp:make-message "geometry_msgs/Point" :x x :y y :z z)
      :orientation (roslisp:make-message "geometry_msgs/Quaternion" :x qx :y qy :z qz :w qw))))

(defun make-mrk-msg (base-frame-name &key
                      (pose (cl-transforms:make-identity-transform)) (action *RVIZ-ADD-MARKER*) (id 0) (type 0) (color *object-color*) (frame-locked 0) (colors (coerce nil 'vector))
                      (mesh-resource "") (alpha 1) (namespace "cutplan") (scale (roslisp:make-message "geometry_msgs/Vector3" :x 1 :y 1 :z 1)) (points (coerce nil 'vector)))
  (roslisp:with-fields (a r g b) color
    (let* ((color (roslisp:make-message "std_msgs/ColorRGBA" :a (* a alpha) :r r :g g :b b)))
      (roslisp:make-message "visualization_msgs/Marker"
                            :header (roslisp:make-message "std_msgs/Header" :frame_id base-frame-name :stamp 0)
                            :ns namespace
                            :id id
                            :frame_locked frame-locked
                            :action action
                            :type type
                            :pose (tr->ps pose)
                            :scale scale
                            :color color
                            :colors colors
                            :points points
                            :mesh_resource mesh-resource))))

(defun make-mesh-marker-msg (base-frame-name mesh-resource &key
                             (color *object-color*) (frame-locked 0) (id 0) (action *RVIZ-ADD-MARKER*) (pose (cl-transforms:make-identity-transform)) (alpha 1) (namespace "cutplan"))
  (make-mrk-msg base-frame-name :frame-locked frame-locked :color color :id id :action action :pose pose :type 10 :alpha alpha :namespace namespace :mesh-resource mesh-resource))

(defun publish-object-marker (object-name)
  (let* ((transforms (cpl-impl:value (cdr (assoc object-name *marker-object-fluents* :test #'equal))))
         (transforms (if (listp transforms)
                       transforms
                       (list transforms)))
         (base-frame-name (cl-tf:frame-id (car transforms)))
         (indices (alexandria:iota (length transforms)))
         (mesh-resource (cdr (assoc object-name *marker-object-mesh-paths* :test #'equal))))
    (if (equal object-name "strawberry")
      (mapcar (lambda (transform index)
                (roslisp:publish (ensure-mrk-publisher)
                                 (make-mesh-marker-msg base-frame-name mesh-resource :frame-locked 1 :pose transform :namespace object-name :id index)))
              transforms
              indices)
      (roslisp:publish (ensure-mrk-publisher)
                       (make-mesh-marker-msg base-frame-name mesh-resource :frame-locked 1 :pose (car transforms) :namespace object-name)))))

(defun initial-marker-placement ()
  (reset-marker-object-fluents)
  (mapcar #'publish-object-marker
          *object-names*))

(defun place-banana-in-blender-bowl ()
  (let* ((mesh-resource (cdr (assoc "banana" *marker-object-mesh-paths* :test #'equal)))
         (blender-bowl-pose (cpl-impl:value (cdr (assoc "blender-bowl" *marker-object-fluents* :test #'equal))))
         (blender-bowl-pose (if (listp blender-bowl-pose) (car blender-bowl-pose) blender-bowl-pose))
         (base-frame-name (cl-tf:frame-id blender-bowl-pose))
         (banana-pose (cl-tf:transform* blender-bowl-pose banana-in-blender-bowl))
         (banana-pose (cl-tf:make-transform-stamped base-frame-name "banana" 0 (cl-tf:translation banana-pose) (cl-tf:rotation banana-pose))))
    (setf (cpl-impl:value *pose-banana*) banana-pose)
    (roslisp:publish (ensure-mrk-publisher)
                     (make-mesh-marker-msg base-frame-name mesh-resource :frame-locked 1 :pose banana-pose :namespace "banana"))))

(defun place-strawberry-in-blender-bowl ()
  (let* ((mesh-resource (cdr (assoc "strawberry" *marker-object-mesh-paths* :test #'equal)))
         (blender-bowl-pose (cpl-impl:value (cdr (assoc "blender-bowl" *marker-object-fluents* :test #'equal))))
         (blender-bowl-pose (if (listp blender-bowl-pose) (car blender-bowl-pose) blender-bowl-pose))
         (base-frame-name (cl-tf:frame-id blender-bowl-pose))
         (strawberry-poses (mapcar (lambda (strawberry-in-blender-bowl)
                                     (cl-tf:transform* blender-bowl-pose strawberry-in-blender-bowl))
                                   strawberries-in-blender-bowl))
         (strawberry-poses (mapcar (lambda (strawberry-pose)
                                     (cl-tf:make-transform-stamped base-frame-name "strawberry" 0 (cl-tf:translation strawberry-pose) (cl-tf:rotation strawberry-pose)))
                                   strawberry-poses))
         (indices (alexandria:iota (length strawberry-poses))))
    (setf (cpl-impl:value *pose-strawberry*) strawberry-poses)
    (mapcar (lambda (strawberry-pose index)
              (roslisp:publish (ensure-mrk-publisher)
                               (make-mesh-marker-msg base-frame-name mesh-resource :frame-locked 1 :pose strawberry-pose :namespace "strawberry" :id index)))
            strawberry-poses indices)))

(defun place-banana-in-hiding ()
  (let* ((mesh-resource (cdr (assoc "banana" *marker-object-mesh-paths* :test #'equal)))
         (banana-pose hiding-pose))
    (setf (cpl-impl:value *pose-banana*) banana-pose)
    (roslisp:publish (ensure-mrk-publisher)
                     (make-mesh-marker-msg "map" mesh-resource :frame-locked 1 :pose banana-pose :namespace "banana"))))

(defun place-strawberry-in-hiding ()
  (let* ((mesh-resource (cdr (assoc "strawberry" *marker-object-mesh-paths* :test #'equal)))
         (strawberry-poses (mapcar (lambda (strawberry-in-blender-bowl)
                                     (declare (ignore strawberry-in-blender-bowl))
                                     hiding-pose)
                                   strawberries-in-blender-bowl))
         (indices (alexandria:iota (length strawberry-poses))))
    (setf (cpl-impl:value *pose-strawberry*) strawberry-poses)
    (mapcar (lambda (strawberry-pose index)
              (roslisp:publish (ensure-mrk-publisher)
                               (make-mesh-marker-msg "map" mesh-resource :frame-locked 1 :pose strawberry-pose :namespace "strawberry" :id index)))
            strawberry-poses indices)))

