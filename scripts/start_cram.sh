#!/usr/bin/env sh
"true";exec /usr/bin/env /usr/bin/sbcl --noinform --dynamic-space-size 4096 --control-stack-size 8 --end-runtime-options --noprint --no-userinit --disable-debugger --script "$0" "$@" 


(REQUIRE :ASDF)

(labels ((get-roslisp-path ()
           ;; calls rospack to find path to roslisp
           (let ((rospack-process
                   (run-program "rospack" '("find" "roslisp")
                                :search t
                                :output :stream)))
             (when rospack-process
               (unwind-protect
                    (with-open-stream (o (process-output rospack-process))
                      (concatenate 'string (car (loop
                                                  for line := (read-line o nil nil)
                                                  while line
                                                  collect line)) "/load-manifest/"))
                 (process-close rospack-process)))))
         (load-ros-lookup ()
           ;; make sure roslisp is in asdf central registry
           (PUSH (get-roslisp-path) ASDF:*CENTRAL-REGISTRY*)
           ;; load ros-load-manifest, defining e.g. "ros-load:load-system"
           (ASDF:OPERATE 'ASDF:LOAD-OP :ROS-LOAD-MANIFEST :VERBOSE NIL)))
  (load-ros-lookup))

(PUSH :ROSLISP-STANDALONE-EXECUTABLE *FEATURES*)

(ros-load:load-system "roslisp" "roslisp")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;          Here, the actual executive code begins.          ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;; Make sure the milkshake-demo package is loaded
(roslisp:ros-info (milkshake-demo) "Loading milkshake demo.")
(ros-load:load-system "milkshake_demo" "milkshake-demo")
(roslisp:ros-info (milkshake-demo) "Milkshake demo loaded.")

;; Start the scenario
(milkshake-demo::start-scenario)
