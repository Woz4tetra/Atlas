import math

from analyzers.converter import HeadingConverter, PositionConverter
from analyzers.kalman_filter import HeadingFilter, PositionFilter


def init_objects(initial_lat, initial_long, initial_enc):
    heading_converter = HeadingConverter(initial_lat, initial_long)
    position_converter = PositionConverter(
        initial_enc, initial_lat, initial_long)
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
    heading_converter, position_converter, heading_filter, position_filter = \
        init_objects(0, 0, 0)
    gps_long = gps_lat = 1 / 111226.343

    # initial angle of buggy is 45 degrees. Acceleration relative to that
    # direction. Converters SHOULD reorient this.
    ax = 1
    ay = 0

    gps_heading, bind_heading = heading_converter.convert(gps_long, gps_lat, 0, 0)
    assert gps_heading == math.pi / 4
    assert bind_heading == 0

    kalman_heading = heading_filter.update(
        gps_heading, True, bind_heading, False, math.pi / 4, True)
    print(kalman_heading)
    # assert kalman_heading == math.pi / 4

    kalman_heading = math.pi / 4
    gps_x, gps_y, shifted_ax, shifted_ay, enc_dist = \
        position_converter.convert(gps_long, gps_lat, ax, ay, 1, kalman_heading)

    assert (almost_equal(0.7071067812, shifted_ax, shifted_ay) and
            almost_equal(math.cos(gps_lat / 2), gps_x) and
            almost_equal(1, gps_y) and
            almost_equal(0.271 * math.pi, enc_dist))

    x, y = position_filter.update(
        gps_x, gps_y, True, 0.001,
        shifted_ax, shifted_ay, True, 0.001,
        enc_dist, True, 0.001, 0.001,
        kalman_heading)
    assert almost_equal(x, y)


def test_all():
    test_initial()


if __name__ == '__main__':
    test_all()
