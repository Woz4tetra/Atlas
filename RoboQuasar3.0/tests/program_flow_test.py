import math
import sys

sys.path.insert(0, "../")

from analyzers.converter import HeadingConverter, PositionConverter
from analyzers.kalman_filter import HeadingFilter, PositionFilter


def init_objects(initial_lat, initial_long, initial_enc):
    heading_converter = HeadingConverter(initial_long, initial_lat)
    position_converter = PositionConverter(
        initial_enc, initial_long, initial_lat)
    position_filter = PositionFilter()
    heading_filter = HeadingFilter()

    return heading_converter, position_converter, heading_filter, position_filter


def almost_equal(main_value, *values, epsilon=0.0001):
    for value in values:
        if abs(main_value - value) > epsilon:
            return False
    return True


def test_initial():
    """
    Test initial conditions of pipeline. Everything points 45 degrees
    """

    # initialize converters and filters to 0, 0 (in degrees)
    heading_converter, position_converter, heading_filter, position_filter = \
        init_objects(0, 0, 0)

    # next gps reading is 1m north and 1m east
    deg_to_m = 111226.343
    gps_long = gps_lat = 1 / deg_to_m

    # initial angle of buggy is 45 degrees. Acceleration relative to that
    # direction. Converters SHOULD reorient this. Initial heading isn't defined
    # anywhere because it's discovered through the heading filter
    ax = math.sqrt(2)
    ay = 0

    # convert gps and bind coordinates to heading values. In the initial time
    # step, bind doesn't have a previous value, so it is 0, 0
    gps_heading, bind_heading = heading_converter.convert(
        gps_long, gps_lat, 0, 0)
    assert gps_heading == math.pi / 4
    assert bind_heading == 0

    # filter gps heading, bind heading (which is ignored, indicated by the
    # "False" flag), and imu delta heading
    kalman_heading = heading_filter.update(
        gps_heading, True, bind_heading, False, math.pi / 4, True)

    # The filter should return 0.0 because the masked binding heading is
    # ignored. The filter can't do anything if one of its observation is
    # invalid. So it waits until it receives an observation with all valid
    # observations.
    assert kalman_heading == 0.0

    # say that the bound point is 45 degrees from the start
    bind_heading = math.pi / 4

    # All observations are now valid, the filter can return a heading now
    kalman_heading = heading_filter.update(
        gps_heading, True, bind_heading, True, math.pi / 4, True)
    print(kalman_heading)

    # it's basically 45 degrees...
    assert almost_equal(math.pi / 4, kalman_heading, epsilon=0.1)

    # for sake of testing, assume the filter return a perfect 45 degree value
    kalman_heading = math.pi / 4

    # Make sure everything's in the right units and coordinate frame.
    # Convert gps degrees to meters and reset to the buggy's origin point.
    # Shift accelerometer to be in the correct reference frame.
    # Convert encoder counts to meters.
    gps_x, gps_y, shifted_ax, shifted_ay, enc_dist = \
        position_converter.convert(
            gps_long, gps_lat, ax, ay,
            math.sqrt(2) / (0.271 * math.pi), kalman_heading
        )

    assert (almost_equal(1, shifted_ax, shifted_ay) and
            almost_equal(math.cos(gps_lat / 2), gps_x) and
            almost_equal(1, gps_y) and
            almost_equal(math.sqrt(2), enc_dist))

    x, y = position_filter.update(
        gps_x, gps_y, True, 0.001,
        shifted_ax, shifted_ay, True, 0.001,
        enc_dist, True, 0.001, 0.001,
        kalman_heading)
    print(x, y)

    # doesn't equal 1, 1 because it's weighting the starting point of 0, 0.
    assert almost_equal(x, y)


def test_time_step_n_perfect_data():
    """
    testing moving from (0, 0) -> (1, 1) -> (-1, 1) -> (-2, 2)
    with an initial heading of 45 degrees
    assuming perfect data and time step of 1 second
    """

    # initialize converters and filters to 0, 0 (in degrees)
    heading_converter, position_converter, heading_filter, position_filter = \
        init_objects(0, 0, 0)

    deg_to_m = 111226.343
    dt = 1

    def time_step(gps_long_m, gps_lat_m, ax, ay, encoder_counts, imu_heading,
                  bind_x, bind_y):
        gps_long = gps_long_m / deg_to_m
        gps_lat = gps_lat_m / deg_to_m

        gps_heading, bind_heading = heading_converter.convert(
            gps_long, gps_lat, bind_x, bind_y)

        kalman_heading = heading_filter.update(
            gps_heading, True, bind_heading, True, imu_heading, True)

        print("heading:", kalman_heading)

        gps_x, gps_y, shifted_ax, shifted_ay, enc_dist = \
            position_converter.convert(
                gps_long, gps_lat, ax, ay,
                encoder_counts, kalman_heading
            )

        x, y = position_filter.update(
            gps_x, gps_y, True, dt,
            shifted_ax, shifted_ay, True, dt,
            enc_dist, True, dt, dt,
            kalman_heading)

        print("x: %f, y: %f" % (x, y))

    # time step 0
    time_step(1, 1, 2 * math.sqrt(2), 0, math.sqrt(2) / (0.271 * math.pi), 0,
              1, 1)

    # time step 1
    time_step(-1, 1, -6, 0, 2 / (0.271 * math.pi), 135 * math.pi / 180,
              -1, 1)

    # time step 2
    time_step(-2, 2, 2 * math.sqrt(2), 0, math.sqrt(2) / (0.271 * math.pi), 0,
              -2, 2)

    # time step 3...13
    for _ in range(10):
        time_step(-2, 2, 0, 0, 0, 0, -2, 2)


def test_all():
    test_initial()
    test_time_step_n_perfect_data()


if __name__ == '__main__':
    test_all()
