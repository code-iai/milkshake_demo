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

;; Basic move functions

(defun ensure-pose-stamped (pose)
  (if (typep pose 'cl-tf:pose-stamped)
    pose
    (if (typep pose 'cl-tf:transform-stamped)
      (cl-tf:make-pose-stamped (cl-tf:frame-id pose) 0
                               (cl-tf:translation pose)
                               (cl-tf:rotation pose)))))

(defun move-arm-pose (arm pose)
  (let* (pose (ensure-pose-stamped pose))
    (cpl-impl:with-failure-handling
      ((cram-common-failures:actionlib-action-timed-out (e)
         (declare (ignore e))
         (return)))
      (if (eql arm :left)
        (pr2-pp-plans::move-arms-in-sequence (list pose) nil)
        (pr2-pp-plans::move-arms-in-sequence nil (list pose))))))

(defun move-arm-poses (arm poses)
  (let* ((poses (if (listp poses) poses (list poses))))
    (mapcar (lambda (pose)
              (move-arm-pose arm pose))
            poses)))

(defun move-arms-up ()
  (pr2-pp-plans::park-arms))

