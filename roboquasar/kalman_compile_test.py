from autobuggy.filters.kalman_filter import GrovesKalmanFilter
from roboquasar_constants import constants

kf = GrovesKalmanFilter(**constants)
kf.imu_updated(0.02098393440246582,
               0.019999999552965164, 0.07999999821186066, -0.2800000011920929,
               -0.04886922240257263, 0.13962635397911072, -0.027925269678235054)

kf.gps_updated(0.03800225257873535,
               40.44057846069336, -79.94245147705078, 302.79998779296875)

print(kf.get_position())
