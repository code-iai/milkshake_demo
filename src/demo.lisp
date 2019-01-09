;;;
;;; Copyright (c) 2018, Mihai Pomarlan <blandc@cs.uni-bremen.de>
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
;;;     * Neither the name of the Intelligent Autonomous Systems Group/
;;;       Technische Universitaet Muenchen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
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

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; action_core: Cutting
;; action_roles: ['action_verb', 'amount', 'obj_to_be_cut', 'unit', 'utensil']
;; cram_plan: "(cut (an object (type {obj_to_be_cut}){obj_to_be_cut_props})(into (amount (a quantity (type {unit})(number {amount}))))(with (an object (type {utensil}){utensil_props})))"

;; action_core: Pouring
;; action_roles: ['stuff','goal','action_verb','unit','amount']
;; required_action_roles: ['stuff','goal','action_verb']
;; cram_plan: "(pour-from-container
;;               (from (an object  (type container.n.01) (contains (some stuff (type {stuff}){stuff_props})))) (a quantity (type {unit})(number {amount})) (to (an object  (type {goal}){goal_props})))"

(defun cleanup-action-roles (action-roles)
  (let* ((action-roles (mapcar (lambda (action-role)
                                 (roslisp:with-fields ((role-name role_name) (role-value role_value) (role-values role_values)) action-role
                                   (let* ((point-pos (position #\. role-value))
                                          (role-value (if point-pos
                                                          (subseq role-value 0 point-pos)
                                                          role-value)))
                                     (list role-name role-value role-values))))
                               action-roles)))
    action-roles))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(cpl-impl:def-cram-function perform-mixing (milkshake-type)
  )
(defun perform-mixing-get-args (&rest action-roles)
  )

(cpl-impl:def-cram-function perform-cooking (milkshake-type)
    ;;(cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm pr2-pms::pr2-base-pm)
  (cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm)
      (initial-marker-placement)
      (move-arms-up)
      (establish-physical-exposure (list milkshake-type "milk") "blender-bowl" "blend")
      (establish-kinematic-controllability "milkshake" "mug" nil)
      (initial-marker-placement)
      (init-kinematic-controllabilities)
      (init-physical-exposures)
      ;;)
   ))

(defun perform-cooking-get-args (&rest action-roles)
  (cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm)
    (let* ((action-roles (cleanup-action-roles action-roles))
           (item-type (car (cdr (assoc "type" action-roles :test #'equal))))
           (milkshake-type (car (cdr (assoc "flavor" action-roles :test #'equal))))
           (should-run-plan (when (and (equal item-type "milkshake")
                                       (position milkshake-type '("strawberry" "banana") :test #'equal))
                                  T))
           (args (when should-run-plan
                   (list milkshake-type)))
           (msg (if should-run-plan
                  (format nil "cooking of a ~a ~a~%" milkshake-type item-type)
                  (format nil "unrecognized recipe to cook or don't have ingredients for ~a ~a~%" milkshake-type item-type)))
           (plan-string (if should-run-plan
                          (format nil "(perform-cooking ~a ~a)~%" item-type milkshake-type)
                          (format nil ""))))
      (values (if should-run-plan 0 -1)
              args
              msg
              plan-string))))

(defun cancel-function ()
  (cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm)
    (move-arms-up)
    )
  (initial-marker-placement)
  (init-kinematic-controllabilities)
  (init-physical-exposures))

(defparameter *pracsimserver-plan-matchings*
              (list (cons "Mixing" (list #'perform-mixing #'perform-mixing-get-args))
                    (cons "Cooking" (list #'perform-cooking #'perform-cooking-get-args))))

(defun start-scenario ()
  (roslisp:ros-info (milkshake-demo) "Starting up ...")
  (remhash 'cram-moveit::init-moveit-bridge roslisp-utilities::*ros-init-functions*)
  (remhash 'semantic-map-collision-environment::init-semantic-map-collision-environment roslisp-utilities::*ros-init-functions*)
  (roslisp-utilities:startup-ros)
  (setf cram-tf:*transformer* (make-instance 'cl-tf2:buffer-client))
  (cpl-impl:sleep* 1)
  (roslisp:ros-info (milkshake-demo) "Node startup complete")
  (prac2cram:prac2cram-server *pracsimserver-plan-matchings* #'cancel-function)
  (roslisp:ros-info (milkshake-demo) "Started a prac2cram server")
  (initial-marker-placement)
  (init-kinematic-controllabilities)
  (init-physical-exposures)
  (cpl-impl:top-level
    (cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm)
      (move-arms-up)))
  (let* ((a 1) (b 1) (s 1)
         ;;(thr (sb-thread:make-thread (lambda ()
         ;;                              (cpl-impl:top-level
         ;;                                (perform-mixing "strawberry")))))
         )
    (loop
      (let ((c (rem (+ a b) 97)))
        ;;(initial-marker-placement)
        ;;(init-kinematic-controllabilities)
        ;;(init-physical-exposures)
        ;;(cpl-impl:top-level
        ;;  (cram-process-modules:with-process-modules-running (pr2-pms::pr2-arms-pm pr2-pms::pr2-grippers-pm pr2-pms::pr2-ptu-pm pr2-pms::pr2-base-pm)
        ;;    ;;(press-blender-button :left)
        ;;    ;;(let* ((grabbed (grab-and-lift-container "banana" "banana"))
        ;;    ;;       (arm (first grabbed))
        ;;    ;;       (init-pose (second grabbed)))
        ;;    ;;  (pour-into-container arm "banana" "banana" "blender-bowl")
        ;;    ;;  ;;(place-container arm "bowl" init-pose)
        ;;    ;;  (roslisp:wait-duration 10)
        ;;    ;;  )
        ;;    (establish-physical-exposure (list "banana" "milk") "blender-bowl" "blend")
        ;;    (establish-kinematic-controllability "milkshake" "mug" nil)
        ;;    (initial-marker-placement)
        ;;    (init-kinematic-controllabilities)
        ;;    (init-physical-exposures)
        ;;    ))
        (roslisp:wait-duration 2.5)
        (mapcar #'publish-object-marker *object-names*)
        (format t "Tick-tock ~a: ~a.~%" s c)
        (setf s (+ s 1))
        (setf s (if (<= 100 s) 0 s))
        (setf a b)
        (setf b c)))))

