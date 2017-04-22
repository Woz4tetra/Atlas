import math

g = 9.80665

constants = dict(
    initial_v_north=0,
    initial_v_east=0,
    initial_v_down=0,

    initial_attitude_unc=math.radians(1),
    initial_velocity_unc=1,
    initial_position_unc=10,
    initial_accel_bias_unc=(g * 1E-6) ** 2,
    initial_gyro_bias_unc=math.radians(1 / 3600),

    gyro_noise_PSD=100 * math.radians(0.2 / 60) ** 2,
    accel_noise_PSD=1000 * (g * 1E-3) ** 2,
    accel_bias_PSD=1.0E-6,
    gyro_bias_PSD=2.0E-12,
    pos_meas_SD=4.5,
    vel_meas_SD=0.5
    # find a way to take the data and find the correct values for these
)
