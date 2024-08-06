define laser ranger
(
  sensor
  (
    range_max 10
    fov 180
    samples 1920
  )
  # generic model properties
  color "black"
  size [ 0.06 0.15 0.03 ]
)

define diff_drive_robot position
(
  pose [ 0.0 0.0 0.0 0.0 ]

  odom_error [0.0 0.0 0.0 0.0]

  size [ 0.68 0.57 1.5 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  color "gray"

  # kinematics
  drive "diff"
  bitmap "robot1.png"

  # spawn sensors
  laser(pose [ 0.31 0.0 -1.4 0.0 ])
  laser(pose [ -0.31 0.0 -1.4 180 ])
)
