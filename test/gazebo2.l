

(load "package://robot_assembler/test/test2.l")
(load "package://robot_assembler/euslisp/assembled-robot-interface.l")

(assembled-robot-init)

(objects (list *robot*))

(send *robot* :init-pose)
(send *ri* :angle-vector (send *robot* :angle-vector))


(defun ik(limb fv)
  (let* ((num-list (list 6 12 18 24))
         (p (position limb (list :br :fr :bl :fl)))
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

  (send *robot* :newcoords (make-coords))

  (do-until-key

   (send *robot* :init-pose)

   (ik :fr #f(-10 0 0))
   (ik :fl #f(-10 0 0))
   (ik :br #f(-10 0 0))
   (ik :bl #f(-10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)

   (dolist (limb (list :fr :bl :fl :br))
     (ik limb #f(-10 20 0))
     (send *ri* :angle-vector (send *robot* :angle-vector) 100)
     (send *ri* :wait-interpolation)
     (ik limb #f(10 0 0))
     (send *ri* :angle-vector (send *robot* :angle-vector) 100)
     (send *ri* :wait-interpolation)
     )
   ))

(defun walk2()
  (send *robot* :newcoords (make-coords))
  (do-until-key
   (send *robot* :init-pose)
   (ik :fr #f(-10 0 0))
   (ik :fl #f(-10 0 0))
   (ik :br #f(-10 0 0))
   (ik :bl #f(-10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   
   (ik :fr #f(-10 20 0))
   (ik :bl #f(-10 20 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (ik :fr #f(10 0 0))
   (ik :bl #f(10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)

   (ik :fl #f(-10 20 0))
   (ik :br #f(-10 20 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (ik :fl #f(10 0 0))
   (ik :br #f(10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   )
  )

