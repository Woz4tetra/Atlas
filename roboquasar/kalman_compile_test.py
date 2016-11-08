from autobuggy.filters.kalman_filter import GrovesKalmanFilter
import math

g = 9.80665

initial_roll_ecef = 0
initial_pitch_ecef = 0
initial_yaw_ecef = math.pi / 2
roll_error = math.radians(-0.05)
pitch_error = math.radians(0.04)
yaw_error = math.radians(1)
initial_lat = 40.44057846069336
initial_long = -79.94245147705078
initial_alt = 302.79998779296875 # geoid: -33.0
initial_v_north = 0
initial_v_east = 0
initial_v_down = 0

initial_attitude_unc = math.radians(1)
initial_velocity_unc = 0.1
initial_position_unc = 10
initial_accel_bias_unc = (g * 1E-3) ** 2
initial_gyro_bias_unc = math.radians(1 / 3600)
initial_clock_offset_unc = 10000
initial_clock_drift_unc = 100

gyro_noise_PSD = math.radians(0.02 / 60) ** 2
accel_noise_PSD = 200 * (g * 1E-6) ** 2
accel_bias_PSD = 1.0E-7
gyro_bias_PSD = 2.0E-12
pos_meas_SD = 2.5
vel_meas_SD = 0.1

kf = GrovesKalmanFilter(
    initial_roll_ecef, initial_pitch_ecef, initial_yaw_ecef,
    roll_error, pitch_error, yaw_error,
    initial_lat, initial_long, initial_alt,
    initial_v_north, initial_v_east, initial_v_down,

    initial_attitude_unc, initial_velocity_unc,
    initial_position_unc, initial_accel_bias_unc,
    initial_gyro_bias_unc, initial_clock_offset_unc,
    initial_clock_drift_unc,

    gyro_noise_PSD, accel_noise_PSD, accel_bias_PSD, gyro_bias_PSD,
    pos_meas_SD, vel_meas_SD
)
kf.imu_updated(0.02098393440246582,
               0.019999999552965164, 0.07999999821186066, -0.2800000011920929,
               -0.04886922240257263, 0.13962635397911072, -0.027925269678235054)

kf.gps_updated(0.03800225257873535,
               40.44057846069336, -79.94245147705078, 302.79998779296875)

print(kf.get_position())
