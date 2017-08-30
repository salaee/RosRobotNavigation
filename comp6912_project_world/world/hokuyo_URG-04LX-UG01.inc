
define hokuyolaser ranger
(
    # laser-specific properties
    sensor( 			
        range [ 0.20  6.0 ]
        fov 240.0
        samples 100 # Actually, should be 683 but we downsample for speed
    )

    # generic model properties

    # Draw a block to represent the physical sensor
    block( 
        points 4 
        point[0] [ -0.05 -0.05 ]
        point[1] [ 0.05 -0.05 ]
        point[2] [ 0.05 0.05 ]
        point[3] [ -0.05 0.05 ]
        z [-0.05 0.02]
    )

    color "blue"
    size [ 0.05 0.05 0.07 ]
)

define hokuyolaser_target ranger
(
    # laser-specific properties
    sensor( 			
        range [ 0.20  2.0 ]
        fov 240.0
        samples 100 # Actually, should be 683 but we downsample for speed
    )

    # generic model properties

    # Draw a block to represent the physical sensor
    block( 
        points 4 
        point[0] [ -0.05 -0.05 ]
        point[1] [ 0.05 -0.05 ]
        point[2] [ 0.05 0.05 ]
        point[3] [ -0.05 0.05 ]
        z [-0.05 0.02]
    )

    color "blue"
    size [ 0.05 0.05 0.07 ]
)
