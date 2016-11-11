import math

g = 9.80665

constants = dict(
    roll_error=math.radians(-0.05),
    pitch_error=math.radians(0.04),
    yaw_error=math.radians(1),
    initial_v_north=0,
    initial_v_east=0,
    initial_v_down=0,

    initial_attitude_unc=math.radians(1),
    initial_velocity_unc=0.1,
    initial_position_unc=10,
    initial_accel_bias_unc=(g * 1E-3) ** 2,
    initial_gyro_bias_unc=math.radians(1 / 3600),
    initial_clock_offset_unc=10000,
    initial_clock_drift_unc=100,

    gyro_noise_PSD=math.radians(0.02 / 60) ** 2,
    accel_noise_PSD=200 * (g * 1E-6) ** 2,
    accel_bias_PSD=1.0E-7,
    gyro_bias_PSD=2.0E-12,
    pos_meas_SD=2.5,
    vel_meas_SD=0.1
)
