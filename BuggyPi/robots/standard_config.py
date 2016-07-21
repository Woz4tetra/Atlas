import math
import time


def axis_inactive(axis, robot):
    if robot.manual_mode:
        if axis == "left x":
            robot.servo.set(0)
            robot.filter.update_servo(0)
        elif axis == "left y":
            robot.blue_led.set(0)
            robot.motors.set(0)
            robot.filter.update_motors(0)


def axis_active(axis, value, robot):
    if robot.manual_mode:
        if axis == "left x":
            robot.servo.set(robot.angle_to_servo(-value))
            robot.filter.update_servo(robot.servo.get())
        elif axis == "left y":
            robot.blue_led.set(int(abs(value) * 255))
            robot.motors.set(int(-value * 100))
            robot.filter.update_motors(robot.motors.get())


def button_dn(button, robot):
    if button == 'B':
        robot.manual_mode = not robot.manual_mode
        print("Switching to",
              "manual mode!" if robot.manual_mode else "autonomous!")
        if not robot.manual_mode:
            robot.pid.reset()
    elif button == 'A':
        robot.communicator.record('checkpoint', num=robot.checkpoint_num,
                                  long=robot.gps.get("long"),
                                  lat=robot.gps.get("lat"))
        print(
            "--------\nCheckpoint reached! %s\n--------" % str(
                robot.checkpoint_num))
        robot.checkpoint_num += 1
    elif button == 'X':
        robot.goal_x, robot.goal_y = robot.waypoints.map[robot.waypoint_num]
        robot.waypoint_num += 1
        print("Setting goal to (%s, %s)" % (robot.goal_x, robot.goal_y))
    elif button == 'Y':
        robot.goal_point += 1
        robot.goal_x, robot.goal_y = robot.checkpoints[robot.goal_point]
        print("setting goal:", robot.goal_point, robot.goal_x, robot.goal_y)

def get_gps_bearing(long, lat, prev_long, prev_lat):
    long = math.radians(long)
    lat = math.radians(lat)
    prev_long = math.radians(prev_long)
    prev_lat = math.radians(prev_lat)

    y = math.sin(long - prev_long) * math.cos(lat)
    x = (math.cos(prev_lat) * math.sin(lat) - math.sin(
        prev_lat) * math.cos(lat) *
         math.cos(long - prev_long))

    return math.atan2(y, x)


def update_camera(robot):
    if robot.enable_camera:
        key = robot.capture.key_pressed()

        if key == 'q' or key == "esc":
            print("quitting...")
            return False
        elif key == ' ':
            if robot.capture.paused:
                print("%0.4fs: ...Video unpaused" % (
                    time.time() - robot.time_start))
            else:
                print("%0.4fs: Video paused..." % (
                    time.time() - robot.time_start))
            robot.capture.paused = not robot.capture.paused
        elif key == 's':
            robot.capture.save_frame()
        elif key == 'v':
            if not robot.capture.recording:
                robot.capture.start_recording()
            else:
                robot.capture.stop_recording()

        if not robot.capture.paused:
            robot.capture.show_frame()

    return True  # True == don't exit program


def close(robot):
    pass


def yaw_updated(robot):
    robot.filter.update_imu(time.time() - robot.time_start,
                            robot.yaw.get('yaw'))
    if robot.log_data:
        robot.record("state", robot.get_state())


def gps_updated(robot):
    if robot.gps.get("fix"):
        robot.filter.update_gps(time.time() - robot.time_start,
                                robot.gps.get("long"), robot.gps.get("lat"))
        if robot.log_data:
            robot.record("state", robot.get_state())


def encoder_updated(robot):
    robot.filter.update_encoder(time.time() - robot.time_start,
                                robot.encoder.get("counts"))
    if robot.log_data:
        robot.record("state", robot.get_state())


properties = dict(
    # ----- general properties -----
    close_fn=close,
    turn_display_off=False,
    manual_mode=True,

    # ----- initial state -----
    initial_long=("checkpoint", 0),
    initial_lat=("checkpoint", 0),
    initial_heading=0.0,

    # ----- physical properties -----

    counts_per_rotation=6,
    wheel_radius=0.097,
    front_back_dist=0.234,
    max_speed=0.88,
    left_angle_limit=0.81096,
    right_angle_limit=-0.53719,
    left_servo_limit=35,
    right_servo_limit=-25,

    # ----- camera -----
    enable_camera=True,
    enable_draw=True,
    cam_width=480,
    cam_height=320,
    update_camera_fn=update_camera,

    # ----- logger -----
    log_file=None,

    log_data=False,
    log_dir=None,
    record_state=True,

    # ----- map and checkpoints -----
    map_file=0,
    map_dir=None,

    checkpoints_file=None,
    checkpoints_dir=None,

    # ----- joystick-----
    use_joystick=True,

    button_down_fn=button_dn,
    button_up_fn=None,
    axis_active_fn=axis_active,
    axis_inactive_fn=axis_inactive,
    joy_hat_fn=None,
)

sensors = dict(
    encoder=dict(sensor_id=0, properties='counts',
                 update_fn=encoder_updated),
    gps=dict(sensor_id=1, properties=['long', 'lat', 'fix'],
             update_fn=gps_updated),
    yaw=dict(sensor_id=2, properties='yaw',
             update_fn=yaw_updated),
)

commands = dict(
    leds=dict(command_array={
        "red": 0,
        "yellow": 1,
        "green": 2,
    }, range=(0, 2),
        mapping={
            "off": 0,
            "on": 1,
            "toggle": 2
        }),
    blue_led=dict(command_id=3, range=(0, 255)),
    servo=dict(command_id=4, range=(
    properties["left_servo_limit"], properties["right_servo_limit"]),
               # (-90, 90),
               mapping={
                   "left": properties['left_servo_limit'],
                   "right": properties['right_servo_limit'],
                   "forward": 0
               }),
    motors=dict(command_id=5, range=(-100, 100),
                mapping={
                    "forward": 100,
                    "backward": -100,
                    "stop": 0
                })
)


def update_properties(**dictionary):
    properties.update(dictionary)
    return properties


def update_sensors(**dictionary):
    sensors.update(dictionary)
    return sensors


def update_commands(**dictionary):
    commands.update(dictionary)
    return commands
