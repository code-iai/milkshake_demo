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

(defclass affecting-schema ()
  ((actee :initarg :actee :initform nil :reader actee)))

(defclass instrumental-schema (affecting-schema)
  ((instrument :initarg :instrument :initform nil :accessor instrument)))

(defclass kinematic-controllability (instrumental-schema)
  ((intention :initarg :intention :initform nil :accessor intention)))

(defun get-different-actees (actees-a actees-b)
  (let* ((actees-a (if (listp actees-a)
                     actees-a
                     (list actees-a)))
         (actees-b (if (listp actees-b)
                     actees-b
                     (list actees-b))))
    (if (equal actees-b '(T))
      nil
      (set-difference actees-a actees-b :test #'equal))))

(defun remove-instrumental-schema (schemas actees instrument)
  (let* ((schemas (mapcar (lambda (schema)
                            (if (or (equal instrument T) (equal (instrument schema) instrument))
                              (let* ((actee-survivors (get-different-actees (actee schema) actees)))
                                (when actee-survivors
                                  (setf (actee schema) actee-survivors)
                                  schema))
                              schema))
                          schemas)))
    (remove-if #'null schemas)))

(defparameter *kinematic-controllabilities* (cpl-impl:make-fluent :name :kinematic-controllabilities :value nil))

(defun check-kinematic-controllabilities (controllabilities actees instrument intention)
  (let* ((actees (if (listp actees) actees (list actees)))
         (controllabilities (mapcar (lambda (controllability)
                                      (let* ((c-actees (actee controllability))
                                             (c-actees (if (listp actee) actee (list actee))))
                                        (when (and (or (equal instrument t) (equal instrument (instrument controllability)))
                                                   (or (equal intention t) (equal intention (intention controllability)))
                                                   (or (equal actees (list t)) (intersection actees c-actees :test #'equal)))
                                          controllability)))
                                    controllabilities))
         (controllabilities (remove-if #'null controllabilities))
         (rem-actees (reduce (lambda (actees controllability)
                               (let* ((actee (actee controllability))
                                      (actee (if (listp actee) actee (list actee))))
                                 (append actees actee)))
                             controllabilities
                             :initial-value nil)))
    (or (equal actees (list t)) (equal (set-difference actees rem-actees :test #'equal) nil))))

(defun init-kinematic-controllabilities ()
  (setf (cpl-impl:value *kinematic-controllabilities*)
        (list (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "banana"
                             :actee "banana")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "blender-device"
                             :actee "blender-device")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "blender-bowl"
                             :actee "blender-bowl")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "bowl"
                             :actee "bowl")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "mug"
                             :actee "mug")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "milk-carton"
                             :actee "milk-carton")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "bowl"
                             :actee "strawberry")
              (make-instance 'kinematic-controllability
                             :intention nil
                             :instrument "milk-carton"
                             :actee "milk"))))

(defun terminate-kinematic-controllability (actees instrument intention)
  (let* ((already-established (check-kinematic-controllability (cpl-impl:value *kinematic-controllabilities*) actees instrument instrument)))
    (when already-established
      (setf (cpl-impl:value *kinematic-controllabilities*)
            (remove-instrumental-schema (cpl-impl:value *kinematic-controllabilities*) actees instrument)))))

(defun find-holder (controllabilities actee)
  (let* ((actee (if (listp actee) actee (list actee)))
         (controllabilities (mapcar (lambda (controllability)
                                      (let* ((c-actee (actee controllability))
                                             (c-actee (if (listp c-actee) c-actee (list c-actee))))
                                        (when (not (set-difference actee c-actee :test #'equal))
                                          controllability)))
                                    controllabilities))
         (controllabilities (remove-if #'null controllabilities)))
    (instrument (car controllabilities))))

(defun find-holds (controllabilities instrument)
  (let* ((actees (reduce (lambda (actees controllability)
                           (let* ((c-actees (actee controllability))
                                  (c-actees (if (listp c-actee) c-actee (list c-actee))))
                             (if (equal instrument (instrument controllability))
                               (append actees c-actees)
                               actees)))
                         controllabilities
                         :initial-value nil)))
    actees))

(defun grab-and-lift-container (container contained)
  (let* ((initial-pose (cpl-impl:value (cdr (assoc container *marker-object-fluents* :test #'equal))))
         (initial-pose (if (listp initial-pose) (car initial-pose) initial-pose))
         (arm (if (< 0 (cl-tf:y (cl-tf:translation initial-pose)))
                :left
                :right))
         (tool-frame (if (equal arm :left)
                       "l_gripper_tool_frame"
                       "r_gripper_tool_frame"))
         (grab-pose (cdr (assoc container *grasp-poses* :test #'equal)))
         (grabbed-pose (cl-tf:transform-inv grab-pose))
         (grab-pose (cl-tf:transform* initial-pose grab-pose))
         (grab-pose (cl-tf:make-transform-stamped "map" tool-frame 0
                                                  (cl-tf:translation grab-pose)
                                                  (cl-tf:rotation grab-pose)))
         (grab-pose-inv (cl-tf:transform-inv grab-pose))
         (lifted-pose (cl-tf:make-transform (cl-tf:make-3d-vector (cl-tf:x (cl-tf:translation grab-pose))
                                                                  (cl-tf:y (cl-tf:translation grab-pose))
                                                                  (+ (cl-tf:z (cl-tf:translation grab-pose)) 0.2))
                                            (cl-tf:rotation grab-pose)))
         (lifted-pose (cl-tf:make-transform-stamped "map" tool-frame 0
                                                    (cl-tf:translation lifted-pose)
                                                    (cl-tf:rotation lifted-pose)))
         (contained (if (listp contained) contained (list contained)))
         (contained (mapcar (lambda (object)
                              (unless (or (equal object "milk") (equal object "milkshake"))
                                object))
                            contained))
         (contained (remove-if #'null contained))
         (updated-markers (cons container contained)))
    (move-arm-poses arm (list grab-pose))
    (setf (cpl-impl:value (cdr (assoc container *marker-object-fluents* :test #'equal)))
          (cl-tf:make-transform-stamped tool-frame container 0
                                        (cl-tf:translation grabbed-pose)
                                        (cl-tf:rotation grabbed-pose)))
    (mapcar (lambda (object)
              (let* ((poses (cpl-impl:value (cdr (assoc object *marker-object-fluents* :test #'equal))))
                     (poses (if (listp poses) poses (list poses)))
                     (poses (mapcar (lambda (pose)
                                      (let* ((pose (cl-tf:transform* grab-pose-inv pose)))
                                        (cl-tf:make-transform-stamped tool-frame object 0
                                                                      (cl-tf:translation pose)
                                                                      (cl-tf:rotation pose))))
                                    poses))
                     (poses (if (equal (length poses) 1)
                              (car poses)
                              poses)))
                (setf (cpl-impl:value (cdr (assoc object *marker-object-fluents* :test #'equal)))
                      poses)))
            contained)
    (mapcar #'publish-object-marker updated-markers)
    (move-arm-poses arm (list lifted-pose))
    (list arm initial-pose)))

(defun pour-into-container (arm old-container contained new-container)
  (let* ((contained (if (listp contained) contained (list contained)))
         (tool-frame (if (equal arm :left)
                       "l_gripper_tool_frame"
                       "r_gripper_tool_frame"))
         (new-container-pose (cpl-impl:value (cdr (assoc new-container *marker-object-fluents* :test #'equal))))
         (new-container-pose (if (listp new-container-pose) (car new-container-pose) new-container-pose))
         (new-container-entry-pose (if (equal new-container "blender-bowl")
                                     blender-bowl-entrance
                                     mug-entrance))
         (old-container-pouring-inv (if (equal "blender-bowl" old-container)
                                      blender-bowl-to-pouring
                                      (if (equal "milk-carton-to-pouring" old-container)
                                        milk-carton-to-pouring
                                        bowl-to-pouring)))
         (old-container-pouring-inv (cl-tf:transform-inv old-container-pouring-inv))
         (old-container-grab-inv (cl-tf:transform-inv (cdr (assoc container *grasp-poses* :test #'equal))))
         (pouring-poses (mapcar (lambda (angle)
                                  (cl-tf:make-transform (cl-tf:make-3d-vector 0 0 0)
                                                        (cl-tf:euler->quaternion :ay angle)))
                                (list 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1)))
         (pouring-poses (mapcar (lambda (pouring-pose)
                                  (let* ((pouring-pose (cl-tf:transform* new-container-pose new-container-entry-pose pouring-pose old-container-pouring-inv old-container-grab-inv)))
                                    (cl-tf:make-transform-stamped "map" tool-frame 0
                                                                  (cl-tf:translation pouring-pose)
                                                                  (cl-tf:rotation pouring-pose))))
                                pouring-poses)))
    (move-arm-poses arm pouring-poses)
    (mapcar (lambda (object)
              (if (equal object "banana")
                (place-banana-in-blender-bowl)
                (if (equal object "strawberry")
                  (place-strawberry-in-blender-bowl))))
            contained)
    (when (or (equal old-container "bowl") (equal old-container "blender-bowl"))
      (move-arm-poses arm (reverse pouring-poses))
      T)))

(defun place-container (arm container pose)
  (let* ((fluent (cdr (assoc container *marker-object-fluents* :test #'equal)))
         (tool-frame (if (equal arm :left)
                       "l_gripper_tool_frame"
                       "r_gripper_tool_frame"))
         (grasp-pose (cdr (assoc container *grasp-poses* :test #'equal)))
         (target-pose (cl-tf:transform* pose grasp-pose))
         (target-pose (cl-tf:make-transform-stamped "map" tool-frame 0
                                                    (cl-tf:translation target-pose)
                                                    (cl-tf:rotation target-pose)))
         (post-pose (cl-tf:transform* target-pose (cl-tf:make-transform (cl-tf:make-3d-vector -0.08 0 0) null-quat)))
         (post-pose (cl-tf:make-transform-stamped "map" tool-frame 0
                                                  (cl-tf:translation post-pose)
                                                  (cl-tf:rotation post-pose))))
    (move-arm-poses arm (list target-pose))
    (setf (cpl-impl:value fluent)
          pose)
    (publish-object-marker container)
    (move-arm-poses arm (list post-pose))))

(defun assert-kinematic-controllability-internal (actee instrument intention)
  (setf (cpl-impl:value *kinematic-controllabilities*)
        (cons (make-instance 'kinematic-controllability
                             :intention intention
                             :instrument instrument
                             :actee actee)
              (cpl-impl:value *kinematic-controllabilities*))))

(defun assert-kinematic-controllability (actees instrument intention)
  (let* ((actees (if (listp actees) actees (list actees))))
    (mapcar (lambda (actee)
              (assert-kinematic-controllability actee instrument intention))
            actees)))

(defun establish-kinematic-controllability-internal (actee instrument intention)
  (let* ((already-established (check-kinematic-controllability (cpl-impl:value *kinematic-controllabilities*) actee instrument intention)))
    (unless already-established
      (let* ((cr-holder (find-holder (cpl-impl:value *kinematic-controllabilities*) actee))
             (cr-held (find-holds (cpl-impl:value *kinematic-controllabilities*) cr-holder))
             (grabbed (grab-and-lift-container cr-holder cr-held))
             (arm (first grabbed))
             (cr-holder-ini-pose (second grabbed))
             (cr-holder-needs-placeback (pour-into-container arm cr-holder cr-held instrument)))
        (when cr-holder-needs-placeback
          (place-container arm cr-holder cr-holder-ini-pose))
        (terminate-kinematic-controllability cr-held T T)
        (assert-kinematic-controllability-internal cr-held instrument intention)))))

(defun establish-kinematic-controllability (actees instrument intention)
  (let* ((actees (if (listp actees) actees (list actees))))
    (mapcar (lambda (actee)
              (establish-kinematic-controllability actee instrument intention))
            actees)))


