

(load "package://robot_assembler/test/test.l")
(load "package://robot_assembler/euslisp/assembled-robot-interface.l")

(assembled-robot-init)

(objects (list *robot*))

(send *robot* :angle-vector #f(0 0 0 0 0 0 0 0 0 0 0 30 0 0 0 0 -30 0 0 0))
(send *ri* :angle-vector (send *robot* :angle-vector))

(defun rev()
  ;; (send *robot* :angle-vector #f(0 0 0 0 0 0 0 0 0 0 0 60 0 0 0 0 -60 0 0 0))

  (send *robot* :angle-vector #f(0 0 -170 0 0 0 0 170 0 0 0 60 -170 0 0 0 -60 170 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun ik(limb fv)
  (let* ((num-list (list 5 10 15 20))
         (p (position limb (list :fl :fr :bl :br)))
         (num (nth p num-list))
         )
  
  (setq c (make-cascoords :pos (v+ fv (send (nth num (send *robot* :links)) :worldpos))
                          :rot (send (nth num (send *robot* :links)) :worldrot)))
  
  (send *robot* :inverse-kinematics c
        :link-list (send *robot* :link-list (send (nth num (send *robot* :links)) :parent))
        :move-target (nth num (send *robot* :links))
        ;; :rotation-axis t
        )
  ))

(defun walk()
  (send *robot* :angle-vector #f(0 -30 0 0 0 0 30 0 0 0 0 90 0 0 0 0 -90 0 0 0))
  (send *robot* :newcoords (make-coords))

  (ik :bl #f(100 0 0))

  
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)

  )

