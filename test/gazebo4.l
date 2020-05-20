

(load "package://robot_assembler/test/test4.l")
(load "package://robot_assembler/euslisp/assembled-robot-interface.l")

(assembled-robot-init)

(objects (list *robot*))

(setq *plain-pose* (send *robot* :init-pose))
(setf (elt *plain-pose* 10) 90)
(setf (elt *plain-pose* 16) -90)
(setf (elt *plain-pose* 22) -90)
(setf (elt *plain-pose* 28) 90)
(send *robot* :angle-vector *plain-pose*)

(send *ri* :angle-vector (send *robot* :angle-vector))



(defun ik(limb fv)
  (let* ((num-list (list 15 21 27 33))
         (p (position limb (list :bl :br :fl :fr)))
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

(defun head-set()
  (send *robot* :init-pose)
  (neck-rot);;
  (setq c (make-cascoords :pos (v+ #f(-350 0 160) (send (nth 9 (send *robot* :links)) :worldpos))
                          :rot (rpy-matrix 0 -2.6 1.57)))
  (send *robot* :inverse-kinematics c
        :link-list (send *robot* :link-list (send (nth 9 (send *robot* :links)) :parent))
        :move-target (nth 9 (send *robot* :links))
        )
  )

(defun head-up()
  (send *robot* :init-pose)
  (neck-rot);;
  (setq c (make-cascoords :pos (v+ #f(0 0 150) (send (nth 9 (send *robot* :links)) :worldpos))
                          :rot (rpy-matrix 0 -1.0 1.57)))
  (send *robot* :inverse-kinematics c
        :link-list (send *robot* :link-list (send (nth 9 (send *robot* :links)) :parent))
        :move-target (nth 9 (send *robot* :links))
        )
  )


(defun head-open()
  (let* ((f (send *robot* :angle-vector))
         )
    (setf (elt f 8) 50)
    (send *robot* :angle-vector f)
    )
  )

(defun head-close(&optional (theta -30))
  (let* ((f (send *robot* :angle-vector))
         )
    (setf (elt f 8) theta)
    (send *robot* :angle-vector f)
    )
  )


(defun neck-rot(&optional (theta 180))
  (let* ((f (send *robot* :angle-vector))
         )
    (setf (elt f 0) theta)
    (send *robot* :angle-vector f)
    )
  )

(defun neck-back()
  (let* ((f (send *robot* :angle-vector))
         )
    (setf (elt f 1) 0)
    (setf (elt f 3) 0)
    (setf (elt f 5) -90)
    (send *robot* :angle-vector f)
    )
  )



(defun put-on-back()
  (head-set)
  (head-open)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)

  (head-close)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (head-up)
  (head-close)
  (send *ri* :angle-vector (send *robot* :angle-vector) 30000)
  (send *ri* :wait-interpolation)

  (neck-rot 0)
  (neck-back)
  (head-close -20)
  (send *ri* :angle-vector (send *robot* :angle-vector) 30000)
  (send *ri* :wait-interpolation)

  (head-open)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)
  )
  

(defun re()
  (send *robot* :init-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  )

;;*walk-pose*
(send *robot* :init-pose)
(ik :fr #f(10 0 20))
(ik :fl #f(10 0 20))
(ik :br #f(10 0 -20))
(ik :bl #f(10 0 -20))
(setq *walk-pose* (send *robot* :angle-vector))
(setf (elt *walk-pose* 1) -88)
(setf (elt *walk-pose* 3) 0);;90
(setf (elt *walk-pose* 5) 0);;90
(send *robot* :angle-vector *walk-pose*)


(defun walk()
  (send *robot* :newcoords (make-coords))
  (do-until-key
   (send *robot* :angle-vector *walk-pose*)
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (dolist (limb (list :fr :bl :fl :br))
     (ik limb #f(10 0 20))
     (send *ri* :angle-vector (send *robot* :angle-vector) 100)
     (send *ri* :wait-interpolation)
     (ik limb #f(-10 0 0))
     (send *ri* :angle-vector (send *robot* :angle-vector) 100)
     (send *ri* :wait-interpolation)
     )
   ))