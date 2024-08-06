

define stone model
(
  size [0.4 0.4 0.4]
  gui_nose 0
)
define pillar model
(
  size [0.05 0.05 0.4]
  gui_nose 0
)

define stone position
(
  pose [ 0.0 0.0 0.0 0.0 ]
  odom_error [0.03 0.03 999999 999999 999999 0.02]
  block(pose [0.4000 0.000 0.000 0.000 ] color "red")
)
