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
    (set-difference actees-a actees-b)))

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
  nil)

(defun init-kinematic-controllabilities
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

(defun establish-kinematic-controllability (actees instrument intention)
  (let* ((already-established (check-kinematic-controllability (cpl-impl:value *kinematic-controllabilities*) actees instrument instrument)))
    (unless already-established
      ;;;;;
      (terminate-kinematic-controllability actees T T)
      (setf (cpl-impl:value *kinematic-controllabilities*)
            (cons (make-instance 'kinematic-controllability
                                 :intention intention
                                 :instrument instrument
                                 :actee actees)
                  (cpl-impl:value *kinematic-controllabilities*))))))

