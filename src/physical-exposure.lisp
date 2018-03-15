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

(defclass physical-exposure ()
  ((physical-condition :initarg :physical-condition :initform nil :reader physical-condition)
   (instrument :initarg :instrument :initform nil :reader instrument)
   (actee :initarg :actee :initform nil :reader actee)))

(defparameter *physical-exposures* (cpl-impl:make-fluent :name :physical-exposures :value nil))

