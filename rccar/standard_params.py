standard_params = dict(
    counts_per_rotation=6,
    wheel_radius=0.097,
    front_back_dist=0.234,
    max_speed=1,  # 0.88

    # physical limit of the servo in radians
    left_angle_limit=0.81096,
    right_angle_limit=-0.53719,

    # physical limit of the servo in servo counts
    left_servo_limit=35,
    right_servo_limit=-25,

    # the servo value at which the robot can't drive forward because it's turned too much
    left_turning_limit=25,
    right_turning_limit=-15
)