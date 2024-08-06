define front_laser ranger
(
  sensor
  (
    range [0.05 10]
    fov 180.0
    samples 1080
  )
  # generic model properties
  color "yellow"
 size [ 0.05 0.05 0.05 ]
)

define back_laser ranger
(
  sensor
  (
    range [0.05 10]
    fov 270.0
    samples 1080
  )
  # generic model properties
  color "yellow"
  size [ 0.05 0.05 0.05 ]
)

define mid_laser ranger
(
  sensor
  (
    range [0.05 10]
    fov 180.0
    samples 1440
  )
  # generic model properties
  color "yellow"
  size [ 0.05 0.05 0.05 ]
)

define block model
(
  size [0.15 0.15 0.100]
  gui_nose 0
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.4 0.4 0.12 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "omni"
  color "gray"
  block(pose [0.0000 0.0 0.000 0.0 ] color "red")
  block(pose [0.1000 0.0 0.000 0.0 ] color "red")
  #block(pose [0.6000 0.350 0.000 0.0 ] color "red")
  #block(pose [-0.6000 -0.350 0.000 0.0 ] color "red")
  #front_laser(pose [ 0.6 0.35 0.0  45.0])
  #back_laser(pose [ -0.6 -0.35 0.0  -135.0])
  mid_laser(pose [ 0.0 0.0 0.0 0.0])
)









