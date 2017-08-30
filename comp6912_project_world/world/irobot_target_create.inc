
define target position
(
  size [0.43 0.36 0.15]

	#Shape of robot
	block
	(
	  points 4
	  point[0] [1 1]
	  point[1] [0 1]
	  point[2] [0 0]
	  point[3] [1 0]	
	  z [0 1]
	)
	#fit shape to this size in meters
		size [0.3 0.3 0.3]

  color "red50"
)

define irobot_target_create target( color "red90" )
