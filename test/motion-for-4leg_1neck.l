

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



(defun head-open-close (&optional (theta 50))
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

(defun look-forward4()
  (send *robot* :angle-vector #f(-90.0 0.0 -80.0 50.0 0.0 60.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  )

(defun re()
  (send *robot* :init-pose)
  (send *ri* :angle-vector (send *robot* :angle-vector) 1000)
  (send *ri* :wait-interpolation)
  )

(defun walk();;一足ずつ移動
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

(defun walk2();;対角の足を同時に動かす
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
   (ik :fr #f(10 0 20))
   (ik :bl #f(10 0 20))
   (ik :fl #f(0 0 10))
   (ik :br #f(0 0 10))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (ik :fr #f(-10 0 0))
   (ik :bl #f(-10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (ik :fl #f(10 0 20))
   (ik :br #f(10 0 20))
   (ik :fr #f(0 0 10))
   (ik :bl #f(0 0 10))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   (ik :fl #f(-10 0 0))
   (ik :br #f(-10 0 0))
   (send *ri* :angle-vector (send *robot* :angle-vector) 100)
   (send *ri* :wait-interpolation)
   ))

(defun one-walk();;一足ずつ
  (send *robot* :newcoords (make-coords))
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
  )

(defun one-walk2();;対角の足を同時に動かす
  (send *robot* :newcoords (make-coords))
  ;;
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
  (ik :fr #f(10 0 20))
  (ik :bl #f(10 0 20))
  (ik :fl #f(0 0 10))
  (ik :br #f(0 0 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fr #f(-10 0 0))
  (ik :bl #f(-10 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fl #f(10 0 20))
  (ik :br #f(10 0 20))
  (ik :fr #f(0 0 10))
  (ik :bl #f(0 0 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fl #f(-10 0 0))
  (ik :br #f(-10 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  )

(defun one-walk3();;対角の足を同時に動かす
  (send *robot* :newcoords (make-coords))
  ;;
  (send *robot* :larm :angle-vector #f(0 0 0 0 0 0))
  (send *robot* :rarm :angle-vector #f(0 0 0 0 0 0))
  (send *robot* :lleg :angle-vector #f(0 0 0 0 0 0))
  (send *robot* :rleg :angle-vector #f(0 0 0 0 0 0))
  (ik :fr #f(10 0 10))
  (ik :fl #f(10 0 10))
  (ik :br #f(10 0 -20))
  (ik :bl #f(10 0 -20))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fr #f(10 0 35))
  (ik :bl #f(10 0 35))
  (ik :fl #f(0 0 10))
  (ik :br #f(0 0 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fr #f(-10 0 0))
  (ik :bl #f(-10 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fl #f(10 0 35))
  (ik :br #f(10 0 35))
  (ik :fr #f(0 0 10))
  (ik :bl #f(0 0 10))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  (ik :fl #f(-10 0 0))
  (ik :br #f(-10 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  )


(defun obj-head-ik()
  ;; if dx > dy,dz rot
  (let* ((pos (send *obj-bbox* :pose :position))
         (x (+ (* 1000 (send pos :x)) -20));;30
         (y (* 1000 (send pos :y)))
         (z (- (* 1000 (send pos :z)) 30));;50
         )
    (send *robot* :inverse-kinematics
          (make-coords :pos (float-vector x y z)
                       :rot (rpy-matrix 0 +1.57 0))
          :move-target (send *robot* :head :end-coords)
          :link-list (send *robot* :link-list (send (send *robot* :head :end-coords) :parent))
          :rotation-axis :z
          :debug-view :no-message
          :warnp nil :dump-command nil
          )
    )
  )

(defun table-demo()
  ;;look side
  (send *robot* :angle-vector #f(-40.0 0.0 -80.0 80.0 0.0 80.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0))
  (ik :fr #f(10 0 20))
  (ik :fl #f(10 0 20))
  (ik :br #f(10 0 -20))
  (ik :bl #f(10 0 -20))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)

  (setq *obj-head-ik-av* nil)
  (while (not *obj-head-ik-av*)
    (one-walk3)
    (obj-bbox-spin-once)
    (setq *obj-head-ik-av* (obj-head-ik))
    )
  ;;机との干渉回避
  (send *robot* :head :angle-vector #f(-47.8584 -9.03955 -42.3663 58.8687 10.6443 70.937 16.5931 44.7053 -18.0727))
  (head-open-close 70)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;;grasp pose
  (send *robot* :angle-vector *obj-head-ik-av*)
  (head-open-close 70)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;grasp
  (head-open-close 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  ;;背中の上まで運ぶ
  (send *robot* :head :angle-vector #f(-178.0 3.74413 -64.0953 76.9229 8.60041 46.3214 29.1646 3.52934 -17.8613))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;release
  (head-open-close 90)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;物体を倒さないように戻す
  (send *robot* :head :angle-vector #f(-178.0 10.6614 -11.3082 30.9044 13.7562 27.5701 16.3527 78.5257 86.6596))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;
  (send *robot* :head :angle-vector #f(-178 0 0 0 0 0 0 0 90))
  (send *ri* :angle-vector (send *robot* :angle-vector) 4000)
  (send *ri* :wait-interpolation)

  ;;二本目
  ;; (send *robot* :head :angle-vector #f(-40.0 0.0 -80.0 80.0 0.0 80.0 0.0 0.0 0.0))
  (send *robot* :head :angle-vector #f(0.0 0.0 -80.0 80.0 0.0 80.0 0.0 0.0 0.0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)

  (setq *obj-head-ik-av* nil)
  (while (not *obj-head-ik-av*)
    (one-walk3)
    (obj-bbox-spin-once)
    (setq *obj-head-ik-av* (obj-head-ik))
    )
    ;;机との干渉回避
  (send *robot* :head :angle-vector #f(-47.8584 -9.03955 -42.3663 58.8687 10.6443 70.937 16.5931 44.7053 -18.0727))
  (head-open-close 70)
  (send *ri* :angle-vector (send *robot* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  ;;grasp pose
  (send *robot* :angle-vector *obj-head-ik-av*)
  (head-open-close 70)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;grasp
  (head-open-close 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 100)
  (send *ri* :wait-interpolation)
  ;;前に持っていく
  (send *robot* :head :angle-vector #f(0 70 0 30 0 30 0 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;
  (dotimes (i 5)
    (one-walk3)
    )
  ;;release
  (head-open-close 80)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)

  ;;
  ;;背中の上
  (send *robot* :head :angle-vector #f(-128.775 -14.8628 -74.8032 99.002 10.566 82.1315 29.0982 1.63397 94.9764))
  (send *ri* :angle-vector (send *robot* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  ;;
  (send *robot* :head :angle-vector #f(-178.0 3.74413 -64.0953 76.9229 8.60041 46.3214 29.1646 3.52934 90))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;grasp
  (head-open-close 0)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;
  (send *robot* :head :angle-vector #f(-178.0 0 0 0 0 0 0 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;前に持っていく
  (send *robot* :head :angle-vector #f(0 70 0 30 0 30 0 0 0))
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
  (send *ri* :wait-interpolation)
  ;;release
  (head-open-close 80)
  (send *ri* :angle-vector (send *robot* :angle-vector) 3000)
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
