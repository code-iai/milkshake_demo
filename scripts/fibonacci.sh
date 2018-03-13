#!/usr/bin/env sh
"true";exec /usr/bin/env /usr/bin/sbcl --noinform --end-runtime-options --noprint --no-userinit --disable-debugger --script "$0" "$@"


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


(defparameter a 1)
(defparameter b 1)

(loop
  (let* ((c (rem (+ a b) 97)))
    (format t "~a~%" c)
    (setf a b)
    (setf b c)
    (roslisp:wait-duration 1)))

