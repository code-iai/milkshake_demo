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

(defun press-blender-button (arm)
  (let* ((blender-device-pose (cpl-impl:value *pose-blender-device*))
         (press-pose (cl-tf:transform* blender-device-pose press-blender-device))
         (pre-press-offset (cl-tf:make-transform (cl-tf:make-3d-vector -0.08 0 0) (cl-tf:euler->quaternion)))
         (pre-press-pose (cl-tf:transform* blender-device-pose press-blender-device pre-press-offset))
         (pre-press-pose (cl-tf:make-pose-stamped (cl-tf:frame-id blender-device-pose) 0
                                                  (cl-tf:translation pre-press-pose)
                                                  (cl-tf:rotation pre-press-pose)))
         (press-pose (cl-tf:make-pose-stamped (cl-tf:frame-id blender-device-pose) 0
                                              (cl-tf:translation press-pose)
                                              (cl-tf:rotation press-pose))))
    (move-arm-poses arm (list pre-press-pose press-pose pre-press-pose))))

(defclass physical-exposure (instrumental-schema)
  ((physical-condition :initarg :physical-condition :initform nil :accessor physical-condition)))

(defparameter *physical-exposures* (cpl-impl:make-fluent :name :physical-exposures :value nil))

(defun check-physical-exposure (exposures actees instrument physical-condition)
  (let* ((actees (if (listp actees) actees (list actees)))
         (exposures (mapcar (lambda (exposure)
                              (let* ((c-actees (actee exposure))
                                     (c-actees (if (listp c-actees) c-actees (list c-actees))))
                                (when (and (or (equal instrument t) (equal instrument (instrument exposure))
                                           (or (equal physical-condition t) (equal physical-condition (physical-condition exposure)))
                                           (or (equal actees (list t)) (intersection actees c-actees :test #'equal)))
                                  exposure))))
                            exposures))
         (exposures (remove-if #'null exposures))
         (rem-actees (reduce (lambda (actees exposure)
                               (let* ((actee (actee exposure))
                                      (actee (if (listp actee) actee (list actee))))
                                 (append actees actee)))
                             exposures
                             :initial-value nil)))
    (or (equal actees (list t)) (equal (set-difference actees rem-actees :test #'equal) nil))))


(defun init-physical-exposures ()
  (setf (cpl-impl:value *physical-exposures*)
        nil))

(defun start-simulate (actees physical-condition)
  (mapcar (lambda (actee)
            (cond
              ((and (equal physical-condition "blend") (equal actee "banana"))
                (terminate-kinematic-controllability "banana" "blender-bowl" T)
                (place-banana-in-hiding))
              ((and (equal physical-condition "blend") (equal actee "strawberry"))
                (terminate-kinematic-controllability "strawberry" "blender-bowl" T)
                (place-strawberry-in-hiding))
              (T nil)))
          actees))

(defun terminate-physical-exposure (instrument physical-condition)
  (let* ((already-established (check-physical-exposure (cpl-impl:value *physical-exposures*) T instrument physical-condition)))
    (when already-established
      (press-blender-button :left)
      (setf (cpl-impl:value *physical-exposures*)
            (remove-instrumental-schema (cpl-impl:value *physical-exposures*) T instrument)))))

(defun establish-physical-exposure (actees instrument physical-condition)
  (let* ((already-established (check-physical-exposure (cpl-impl:value *physical-exposures*) actees instrument physical-condition)))
    (unless already-established
      (establish-kinematic-controllability actees instrument physical-condition)
      (press-blender-button :left)
      (start-simulate actees physical-condition)
      ;;;(establish-kinematic-controllability "milkshake" instrument nil)
      (assert-kinematic-controllability "milkshake" instrument nil)
      (setf (cpl-impl:value *physical-exposures*)
            (cons (make-instance 'physical-exposure
                                 :physical-condition physical-condition
                                 :instrument instrument
                                 :actee actees)
                  (cpl-impl:value *physical-exposures*))))))

