

(load "package://robot_assembler/test/4leg_1neck_robot.l")
(load "package://robot_assembler/euslisp/assembled-robot-interface.l")
(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-msgs "std_msgs")


(assembled-robot-init)

(objects (list *robot*))

(setq *plain-pose* (send *robot* :init-pose))
(setf (elt *plain-pose* 10) -90)
(setf (elt *plain-pose* 16) -90)
(setf (elt *plain-pose* 22) 90)
(setf (elt *plain-pose* 28) 90)
(send *robot* :angle-vector *plain-pose*)

(send *ri* :angle-vector (send *robot* :angle-vector))



(defun ik(limb fv)
  (let* ((num-list (list 15 21 27 33))
         (p (position limb (list :br :fl :fr :bl)))
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

(defun ikc()
  (send *robot* :inverse-kinematics
        (send (send *robot* :rarm :end-coords :copy-worldcoords) :translate (float-vector 0 0 20) :world)
        :move-target (send *robot* :rarm :end-coords)
        :link-list (send *robot* :link-list (send (send *robot* :rarm :end-coords) :parent)))
  )

(defun head-ik()
  (send *robot* :inverse-kinematics
        (send (send *robot* :head :end-coords :copy-worldcoords) :translate (float-vector 0 0 20) :world)
        :move-target (send *robot* :head :end-coords)
        :link-list (send *robot* :link-list (send (send *robot* :head :end-coords) :parent)))
  )



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


(defun head-open(&optional (theta 50))
  (let* ((f (send *robot* :angle-vector))
         )
    (setf (elt f 8) theta)
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


(defun neck-rot(&optional (theta 178))
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

(defun look-forward()
  (send *robot* :init-pose)
  (neck-rot);;
  (setq c (make-cascoords :pos (v+ #f(-350 0 160) (send (nth 9 (send *robot* :links)) :worldpos))
                          :rot (rpy-matrix 0 -2.0 1.57)))
  (send *robot* :inverse-kinematics c
        :link-list (send *robot* :link-list (send (nth 9 (send *robot* :links)) :parent))
        :move-target (nth 9 (send *robot* :links))
        )
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  )

(defun look-forward2()
  (send *robot* :init-pose)
  (send *robot* :angle-vector #f(0.0 0.0 0.0 60.0 0.0 90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  )

(defun look-forward3()
  (send *robot* :init-pose)
  (send *robot* :angle-vector #f(0.0 0.0 0.0 50.0 0.0 60.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  )


;; (defun put-on-back()
;;   (head-set)
;;   (head-open)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 3000)

;;   (head-close)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
;;   (send *ri* :wait-interpolation)

;;   (head-up)
;;   (head-close)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 30000)
;;   (send *ri* :wait-interpolation)

;;   (neck-rot 0)
;;   (neck-back)
;;   (head-close -20)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 30000)
;;   (send *ri* :wait-interpolation)

;;   (head-open)
;;   (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
;;   (send *ri* :wait-interpolation)
;;   )
  

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

(defun walk2();;only leg
  (send *robot* :newcoords (make-coords))
  (do-until-key
   (send *robot* :larm :angle-vector #f(0 0 0 0 0 0))
   (send *robot* :rarm :angle-vector #f(0 0 0 0 0 0))
   (send *robot* :lleg :angle-vector #f(0 0 0 0 0 0))
   (send *robot* :rleg :angle-vector #f(0 0 0 0 0 0))
   (ik :fr #f(10 0 20))
   (ik :fl #f(10 0 20))
   (ik :br #f(10 0 -20))
   (ik :bl #f(10 0 -20))
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


(defun obj-head-ik()
  ;; if dx > dy,dz rot
  (let* ((pos (send *obj-bbox* :pose :position))
         (x (+ (* 1000 (send pos :x)) 30))
         (y (* 1000 (send pos :y)))
         (z (- (* 1000 (send pos :z)) 50))
         )
    (send *robot* :inverse-kinematics
          (make-coords :pos (float-vector x y z)
                       :rot (rpy-matrix 0 -1.57 0))
          :move-target (send *robot* :head :end-coords)
          :link-list (send *robot* :link-list (send (send *robot* :head :end-coords) :parent))
          :rotation-axis t
          :debug-view :no-message)
    )

  (head-open 70)
  )

(defun table-demo()
  (look-forward3)
  (obj-bbox-spin-once)
  (head-open 70)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (obj-head-ik)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)

  (head-close 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  
  
  )





;;;;
(defvar *obj-bbox* nil)
(defun obj_bbox_callback (msg)
  (let* (ob pos x y z dim dx dy dz
         )
    (if (> (length (send msg :boxes)) 0)
        (progn
          (setq *obj-bbox* (car (last (send msg :boxes))))
          (setq pos (send *obj-bbox* :pose :position))
          (setq x (send pos :x))
          (setq y (send pos :y))
          (setq z (send pos :z))
          (setq dim (send *obj-bbox* :dimensions))
          (setq dx (* 1000 (send dim :x)))
          (setq dy (* 1000 (send dim :y)))
          (setq dz (* 1000 (send dim :z)))
          (format t "obj-bbox x: ~A y: ~A z: ~A~%" x y z)
          (setq ob (make-cube dx dy dz))
          (send ob :translate (scale 1000 (float-vector x y z)))
          (objects (list *robot* ob))
          ))
    ))

(defun obj-bbox-spin-once nil
  (ros::rate 100)
  (setq *obj-bbox* nil)
  (while (not *obj-bbox*)
    (ros::spin-once)
    (ros::sleep)))

(ros::subscribe "/rgbcolorfilter/boxes"
                jsk_recognition_msgs::BoundingBoxArray
                #'obj_bbox_callback)


(defun go-to-obj()
  (look-forward2)
  (obj-bbox-spin-once)
  (while (> (- (send *obj-bbox* :pose :position :z)
               (* 0.5 (send *obj-bbox* :dimensions :z))
               )
            0.38
            )
       (send *robot* :larm :angle-vector #f(0 0 0 0 0 0))
       (send *robot* :rarm :angle-vector #f(0 0 0 0 0 0))
       (send *robot* :lleg :angle-vector #f(0 0 0 0 0 0))
       (send *robot* :rleg :angle-vector #f(0 0 0 0 0 0))
       (ik :fr #f(10 0 20))
       (ik :fl #f(10 0 20))
       (ik :br #f(10 0 -20))
       (ik :bl #f(10 0 -20))
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
       (obj-bbox-spin-once)
       )
  )

(defun pick-and-place()

  (send *robot* :angle-vector #f(0.0 90.0 0.0 40.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  
  (head-close)
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (send *robot* :angle-vector #f(0.0 0.0 0.0 40.0 0.0 40.0 0.0 0.0 -30.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)

  (neck-rot)
  (send *ri* :angle-vector (send *robot* :angle-vector) 10000)
  (send *ri* :wait-interpolation)

  (send *robot* :angle-vector #f(178.0 0.0 0.0 10.0 0.0 90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  (send *robot* :angle-vector #f(178.0 0.0 0.0 40.0 0.0 90.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  
  
  )

(defun demo()
  (go-to-obj)
  (pick-and-place)
  )
