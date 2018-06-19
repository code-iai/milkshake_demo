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

(defparameter halfpi (* 0.5 3.14159))
(defparameter deg2rad (/ 3.14159 180.0))
(defparameter null-quat (cl-tf:euler->quaternion))
(defparameter top-grasp (cl-tf:euler->quaternion :ay halfpi))
(defparameter fwd-grasp null-quat)

(defparameter grasp-banana (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) top-grasp))
(defparameter grasp-blender-bowl (cl-tf:make-transform (cl-tf:make-3d-vector -0.124 0 0.025) fwd-grasp))
(defparameter grasp-bowl (cl-tf:make-transform (cl-tf:make-3d-vector -0.118 0 0.042) fwd-grasp))
(defparameter grasp-milk-carton (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) fwd-grasp))
(defparameter grasp-mug (cl-tf:make-transform (cl-tf:make-3d-vector -0.063 0 0.009) fwd-grasp))

(defparameter press-blender-device (cl-tf:make-transform (cl-tf:make-3d-vector -0.12 0 0.015) fwd-grasp))

(defparameter banana-in-blender-bowl (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0) (cl-tf:euler->quaternion :ay (* 233 deg2rad))))
(defparameter strawberries-in-bowl (list (cl-tf:make-transform (cl-tf:make-3d-vector  0     0    -0.027) (cl-tf:euler->quaternion :ax 0.1 :ay -0.1))
                                         (cl-tf:make-transform (cl-tf:make-3d-vector -0.02  0.05 -0.027) (cl-tf:euler->quaternion :ax 0.2 :ay -0.1))
                                         (cl-tf:make-transform (cl-tf:make-3d-vector -0.04 -0.06 -0.027) (cl-tf:euler->quaternion :ax 0.05 :ay -0.15))
                                         (cl-tf:make-transform (cl-tf:make-3d-vector  0.05 -0.03 -0.027) (cl-tf:euler->quaternion :ax 0.0 :ay -0.3))
                                         (cl-tf:make-transform (cl-tf:make-3d-vector  0.04  0.04 -0.027) (cl-tf:euler->quaternion :ax 0.1 :ay -0.25))))
(defparameter strawberries-in-blender-bowl (list (cl-tf:make-transform (cl-tf:make-3d-vector  0.08  0    -0.027) (cl-tf:euler->quaternion :ax 0.1 :ay -0.1))
                                                 (cl-tf:make-transform (cl-tf:make-3d-vector  0.02  0.06 -0.027) (cl-tf:euler->quaternion :ax 0.2 :ay -0.1))
                                                 (cl-tf:make-transform (cl-tf:make-3d-vector -0.035  0.03 -0.027) (cl-tf:euler->quaternion :ax 0.05 :ay -0.15))
                                                 (cl-tf:make-transform (cl-tf:make-3d-vector -0.035 -0.03 -0.027) (cl-tf:euler->quaternion :ax 0.0 :ay -0.3))
                                                 (cl-tf:make-transform (cl-tf:make-3d-vector  0.02 -0.06 -0.027) (cl-tf:euler->quaternion :ax 0.1 :ay -0.25))))

(defparameter hiding-pose (cl-tf:make-transform-stamped "map" "map" 0 (cl-tf:make-3d-vector 0 0 0) (cl-tf:euler->quaternion)))

(defparameter floor-level-banana -0.03)
(defparameter floor-level-blender-device -0.063)
(defparameter floor-level-bowl -0.04)
(defparameter floor-level-milk-carton -0.129)
(defparameter floor-level-mug -0.042)

(defparameter floor-level-map 0.9)

(defparameter init-banana (cl-tf:make-transform-stamped "map" "banana" 0 (cl-tf:make-3d-vector 0.75 0.25 (- floor-level-map floor-level-banana)) null-quat))
(defparameter init-blender-device (cl-tf:make-transform-stamped "map" "blender-device" 0 (cl-tf:make-3d-vector 0.9 0 (- floor-level-map floor-level-blender-device)) null-quat))
(defparameter init-bowl (cl-tf:make-transform-stamped "map" "bowl" 0 (cl-tf:make-3d-vector 0.75 0.5 (- floor-level-map floor-level-bowl)) null-quat))
(defparameter init-milk-carton (cl-tf:make-transform-stamped "map" "milk-carton" 0 (cl-tf:make-3d-vector 0.75 -0.25 (- floor-level-map floor-level-milk-carton)) null-quat))
(defparameter init-mug (cl-tf:make-transform-stamped "map" "mug" 0 (cl-tf:make-3d-vector 0.75 -0.5 (- floor-level-map floor-level-mug)) null-quat))

(defparameter blender-device-to-bowl (cl-tf:make-transform (cl-tf:make-3d-vector -0.017 0 0.129) null-quat))
(defparameter banana-to-pouring (cl-tf:make-transform (cl-tf:make-3d-vector -0.11 0 0) null-quat))
(defparameter blender-bowl-to-pouring (cl-tf:make-transform (cl-tf:make-3d-vector 0.127 0 0.074) null-quat))
(defparameter milk-carton-to-pouring (cl-tf:make-transform (cl-tf:make-3d-vector 0.036 0 0.126) null-quat))
(defparameter bowl-to-pouring (cl-tf:make-transform (cl-tf:make-3d-vector 0.118 0 0.043) null-quat))

(defparameter blender-bowl-entrance-left (cl-tf:make-transform (cl-tf:make-3d-vector -0.05 0.05 0.16) (cl-tf:euler->quaternion :az -1)))
(defparameter blender-bowl-entrance-right (cl-tf:make-transform (cl-tf:make-3d-vector -0.05 -0.05 0.16) (cl-tf:euler->quaternion :az 1)))
(defparameter mug-entrance-left (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0.12) (cl-tf:euler->quaternion :az (- 0.2))))
(defparameter mug-entrance-right (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0.12) (cl-tf:euler->quaternion :az 0.2)))

(defparameter *grasp-poses* `(("banana" . ,grasp-banana)
                              ("blender-bowl" . ,grasp-blender-bowl)
                              ("bowl" . ,grasp-bowl)
                              ("milk-carton" . ,grasp-milk-carton)
                              ("mug" . ,grasp-mug)))
(defparameter *init-poses* `(("banana" . ,init-banana)
                              ("blender-device" . ,init-blender-device)
                              ("bowl" . ,init-bowl)
                              ("milk-carton" . ,init-milk-carton)
                              ("mug" . ,init-mug)))

(defparameter *pose-banana* (cpl-impl:make-fluent
                              :name :pose-banana
                              :value init-banana))

(defparameter *pose-blender-device* (cpl-impl:make-fluent
                                      :name :pose-blender-device
                                      :value init-blender-device))

(defparameter *pose-blender-bowl* (cpl-impl:make-fluent
                                    :name :pose-blender-bowl
                                    :value (cl-tf:transform* init-blender-device blender-device-to-bowl)))

(defparameter *pose-bowl* (cpl-impl:make-fluent
                            :name :pose-bowl
                            :value init-bowl))

(defparameter *pose-milk-carton* (cpl-impl:make-fluent
                                   :name :pose-milk-carton
                                   :value init-milk-carton))

(defparameter *pose-mug* (cpl-impl:make-fluent
                            :name :pose-mug
                            :value init-bowl))

(defparameter *pose-strawberry* (cpl-impl:make-fluent
                                  :name :pose-strawberry
                                  :value (mapcar (lambda (strawberry-in-bowl)
                                                   (cl-tf:transform* init-bowl strawberry-in-bowl))
                                                 strawberries-in-bowl)))

(defparameter *marker-object-fluents* `(("banana" . ,*pose-banana*)
                                        ("blender-device" . ,*pose-blender-device*)
                                        ("blender-bowl" . ,*pose-blender-bowl*)
                                        ("bowl" . ,*pose-bowl*)
                                        ("milk-carton" . ,*pose-milk-carton*)
                                        ("mug" . ,*pose-mug*)
                                        ("strawberry" . ,*pose-strawberry*)))

(defun reset-marker-object-fluents ()
  (setf (cpl-impl:value *pose-banana*) init-banana)
  (setf (cpl-impl:value *pose-blender-device*) init-blender-device)
  (let* ((init-blender-bowl (cl-tf:transform* init-blender-device blender-device-to-bowl))
         (init-blender-bowl (cl-tf:make-transform-stamped "map" "blender-bowl" 0 (cl-tf:translation init-blender-bowl) (cl-tf:rotation init-blender-bowl))))
    (setf (cpl-impl:value *pose-blender-bowl*) init-blender-bowl))
  (setf (cpl-impl:value *pose-bowl*) init-bowl)
  (setf (cpl-impl:value *pose-milk-carton*) init-milk-carton)
  (setf (cpl-impl:value *pose-mug*) init-mug)
  (setf (cpl-impl:value *pose-strawberry*)
        (mapcar (lambda (strawberry-in-bowl)
                  (let* ((init-strawberry (cl-tf:transform* init-bowl strawberry-in-bowl))
                         (init-strawberry (cl-tf:make-transform-stamped "map" "strawberry" 0 (cl-tf:translation init-strawberry) (cl-tf:rotation init-strawberry))))
                    init-strawberry))
                strawberries-in-bowl)))

