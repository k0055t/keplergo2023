from pybricks.pupdevices import Motor, ColorSensor, ColorLightMatrix
from pybricks.parameters import Port, Direction, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from pybricks.hubs import PrimeHub
from pybricks.parameters import Stop
from urandom import randint
from umath import *



# test se spousti vsemi tremi tlacitky najednou

# drive do lajny musí mít ruznej treshold pro bílou a černou

# Motor hold disabled on drive!

BAT_MUL = 0.99

MAX_SPEED = 500 #454 is the calculated maximum, but the scale is not linear so its closer to 500    
MIN_SPEED = 30
DEFAULT_TURN_RATE = 128
DEFAULT_TURN_ACCELERATION = 580
AXLE_TRACK = 144.8

hub = PrimeHub()
print("Battery: " + str(hub.battery.voltage()))
watch = StopWatch()
watch.reset()

left_motor = Motor(Port.E, Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.C)
use_motor_left = Motor(Port.A)
use_motor_right = Motor(Port.B)
color_left = ColorSensor(Port.F)
color_right = ColorSensor(Port.D)

drive_base = DriveBase(left_motor, right_motor, wheel_diameter=56, axle_track=144.8)
drive_base.settings(MAX_SPEED, 1000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
hub.imu.reset_heading(0)

color_left.lights.off()
color_right.lights.off()

def clamp(value, minimum, maximum):
    '''### Restricts a given value within a specified range (minimum and maximum).'''
    return max(min(value, maximum), minimum)

def fake_bezier(x, minimum = 0.1) -> float:
    '''### Returns the value of a curve at point x. The curve mimics a bezier based s-curve. Return values range from 0-1'''
    x = clamp(x, 0, 1)
    return (3*(x**2) - 2*(x**3))*(1-minimum)+minimum

def parse(value : str | bool, keyword : str) -> bool:
    '''### Returns True if the value is the same as the keyword.'''
    out = value
    if type(value) == str:
        out = value.lower().replace(" ", "") == keyword
    return out

def find_index(value: str, options: list[str], only_first_letter = True) -> int | None:
    '''### Given a list of strings as options and a value, the function returns the index of the value in the options list. \n
    If it is not present the function returns None.'''
    for i in range(0, len(options)):
        if only_first_letter and value[0] == options[i][0]:
            return i
        elif value == options[i]:
            return i
    return None

def follow(value : float, mode = 'distance', sensor = 'left', side = 'right', TRESHOLD = 65, PROPORTIONAL_CONSTANT = 2.34, INTEGRAL_CONSTANT = 0.00001, DERIVATIVE_CONSTANT = 1000, speed = 200) -> None: #P=2.34 S=225 D=100000
    '''### Follows a line for a given amout. \n
    - mode: Sets the mode in which the function will operate. Options: 'distance', 'time' \n
    - sensor: Sets the sensor used for following. Options: 'left', 'right' \n
    - side: Sets the side on which the robot will drive. Options 'left', 'right' \n
    - speed: The speed at which the robot will follow the line. \n
    \n
    *TRESHOLD: The treshold for the black value._* \n
    *PROPORTIONAL_CONSTANT: The proportional constant of the PID loop.* \n
    *INTEGRAL_CONSTANT: The integral constant of the PID loop.* \n
    *DERIVATIVE_CONSTANT: The derivative constant of the PID loop.*'''

    """""T = 65, P= 2.34, I= 0.00001, D= 1000
    funguje z pomezí černý a bílý i pokud nezačíná rovnoběžně s čárou"""

    #parser
    mode = parse(mode.lower(), "time")
    sensor = parse(sensor.lower(), "right")
    if sensor:
        sensor = color_right.reflection
    else:
        sensor = color_left.reflection
    side = parse(side.lower(), "right")


    if mode == True:
        start_time = watch.time()
        last_error = sensor() - TRESHOLD
        additive = 0
        while watch.time() - start_time < value:
            data = sensor()
            error = (data-TRESHOLD)
            additive += error
            proportional = error * -PROPORTIONAL_CONSTANT
            derivative = (error - last_error) * -DERIVATIVE_CONSTANT
            integral = additive * -INTEGRAL_CONSTANT
            last_error = error
            drive_base.drive(speed,-(proportional+integral+derivative))
    else:
        drive_base.stop()
        start_distance = drive_base.distance()
        last_error = sensor() - TRESHOLD
        additive = 0
        derivative = 0
        while abs(drive_base.distance() - start_distance) < abs(value):
            data = sensor()
            error = (data-TRESHOLD)
            additive += error
            proportional = error * -PROPORTIONAL_CONSTANT
            derivative = (error - last_error) * -DERIVATIVE_CONSTANT + round(0.85*derivative)
            additive = additive + derivative
            integral = additive * -INTEGRAL_CONSTANT
            drive_base.drive(speed, proportional+integral+derivative)
            last_error = error
    drive_base.stop()

#funguje 100%
def line_square(treshold = 25, drive_speed = 100, turn_rate = 0, turn_speed = 100, after_turn = 10):
    '''under construction. subject to change.'''
    left = color_left.reflection()
    right = color_right.reflection()
    while left > treshold and right > treshold:
        drive_base.drive(drive_speed,turn_rate)
        left = color_left.reflection()
        right = color_right.reflection()
    if left <= treshold:
        while right > treshold:
            right_motor.run(turn_speed)
            right = color_right.reflection()
        left = color_left.reflection()
        while left > treshold:
            left_motor.run(-turn_speed)
            left = color_left.reflection()
        #turn(after_turn*-1)
    elif right <= treshold:
        while left > treshold:
            left_motor.run(turn_speed)
            left = color_left.reflection()
        right = color_left.reflection()
        while right > treshold:
            right_motor.run(-turn_speed)
            right = color_left.reflection()
        turn(after_turn)

def drive(distance : int, mode = 'distance', max_speed = 500, reflection = color_left.reflection, treshold = 30, acceleration = 75, deceleration = 50, turn = 0):
    '''### Drives with both motors at the same time for a given amount. \n
    - distance: The maximum possible distance the robot will travel. \n
    - mode: The mode the function uses. Options: 'distance', 'white_line', 'black_line', 'stall'. \n
    - max_speed: The maximum speed the robot is going to reach.\n
    - reflection: The reflection function (not value!) of the desired color sensor. \n
    - acceleration: The distance in which the robot will be accelerating. Does not work with 'distance' mode. Use drive_base.settings() for that. \n
    - turn: How much the robot will turn whilst driving. Does not work with 'distance' mode.'''
    mode = find_index(mode.lower(), ['distance', 'white_line', 'black_line', 'stall'])
    direction = distance/abs(distance)
    acceleration_mul = 1
    deceleration_mul = 1
    if mode == 0:
        orig_settings = drive_base.settings()
        if not acceleration == 75:
            drive_base.settings(straight_speed=max_speed, straight_acceleration=acceleration)
        else:
            drive_base.settings(straight_speed=max_speed)
        drive_base.straight(distance)
        drive_base.settings(straight_speed=orig_settings[0], straight_acceleration=orig_settings[1])
    elif mode == 3:
        start_distance = drive_base.distance()
        while abs(drive_base.distance() - start_distance) < abs(distance) and not drive_base.stalled():
            if not acceleration == 0:
                acceleration_mul = fake_bezier(abs(drive_base.distance() - start_distance)/acceleration, minimum=MIN_SPEED/max_speed)
            if not deceleration == 0:
                deceleration_mul = fake_bezier((abs(distance)-abs(drive_base.distance() - start_distance))/deceleration, minimum=MIN_SPEED/max_speed)
            drive_base.drive(direction * max_speed * min(acceleration_mul, deceleration_mul), turn*deceleration_mul*acceleration_mul)
    elif mode == 1 or mode == 2:
        start_distance = drive_base.distance()
        if mode == 1:
            while abs(drive_base.distance() - start_distance) < abs(distance) and reflection() < treshold:
                if not acceleration == 0:
                    acceleration_mul = fake_bezier(abs(drive_base.distance() - start_distance)/acceleration, minimum=MIN_SPEED/max_speed)
                if not deceleration == 0:
                    deceleration_mul = fake_bezier((abs(distance)-abs(drive_base.distance() - start_distance))/deceleration, minimum=MIN_SPEED/max_speed)
                drive_base.drive(direction * max_speed * min(acceleration_mul, deceleration_mul), turn*deceleration_mul*acceleration_mul)
        else:
            while abs(drive_base.distance() - start_distance) < abs(distance) and reflection() > treshold:
                if not acceleration == 0:
                    acceleration_mul = fake_bezier(abs(drive_base.distance() - start_distance)/acceleration, minimum=MIN_SPEED/max_speed)
                if not deceleration == 0:
                    deceleration_mul = fake_bezier((abs(distance)-abs(drive_base.distance() - start_distance))/deceleration, minimum=MIN_SPEED/max_speed)
                drive_base.drive(direction * max_speed * min(acceleration_mul, deceleration_mul), turn*deceleration_mul*acceleration_mul)
    drive_base.stop()
    #left_motor.hold()
    #right_motor.hold()

def turn(angle: int, motor = 'both', radius= AXLE_TRACK*0.5, acceleration=DEFAULT_TURN_ACCELERATION):
    '''### Turns the robot by a given angle around a specified point.
    - angle: The angle to rotate by.
    - motor: Which motor(s) to use. Options: 'both', 'left', 'right'
    - radius: The radius of the turn. Does not work with 'both' motors. Use radius along with the motor which needs to spin more.'''
    mode = find_index(motor.lower(), ['both', 'left', 'right'])
    direction = angle/abs(angle)
    if not acceleration == DEFAULT_TURN_ACCELERATION:
        orig_acceleration = drive_base.settings()[3]
        drive_base.settings(turn_acceleration=acceleration)
    if mode == 0:
        drive_base.turn(angle)
    elif mode == 1:
        drive_base.curve(direction*radius, direction*angle)
    elif mode == 2:
        drive_base.curve(-direction*radius, -direction*angle)
    if not acceleration == DEFAULT_TURN_ACCELERATION:
        drive_base.settings(turn_acceleration=orig_acceleration)

def set_motors(left, right):
    use_motor_left.control.target_tolerances(100,1)
    use_motor_right.control.target_tolerances(100,1)
    use_motor_left.reset_angle()
    use_motor_right.reset_angle()
    use_motor_left.track_target(left)
    use_motor_right.track_target(right)
    wait(1000)


def maneuver1():
    drive(-400, mode='stall', turn=2) #445
    drive(-50, max_speed=50)
    turn(-2, motor="right")
    use_motor_left.run_until_stalled(-700, duty_limit=75)
    drive(250, max_speed=200, deceleration=0)
    use_motor_left.stop()
    turn(-25, motor='left', radius=400)
    use_motor_left.run_angle(150, 150)
    drive_base.settings(turn_rate=60)
    #drive_base.use_gyro(True)
    turn(142*(1/BAT_MUL), motor='left')
    #drive_base.use_gyro(False)
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    drive(255, max_speed=300)
    turn(57, motor='right')
    use_motor_left.run(-30)
    drive(510)
    use_motor_left.stop()
    use_motor_right.run_angle(400, 125)
    wait(200)
    drive_base.settings(turn_acceleration=DEFAULT_TURN_ACCELERATION/2)
    turn(29, motor='left', radius=600)
    drive_base.settings(turn_acceleration=DEFAULT_TURN_ACCELERATION)
    drive_base.settings(turn_rate=100)
    turn(-82*(1/BAT_MUL), motor='left', radius=36.2)
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    drive(175*BAT_MUL)
    use_motor_left.run(75)
    turn(90, motor='left', radius=AXLE_TRACK*0.9)
    drive(300)
    use_motor_left.stop()
    use_motor_right.run_angle(-1000, 120)

def maneuver2():
    use_motor_left.run_angle(-500, 180)
    drive_base.settings(turn_rate=40)
    turn(-53.5, motor='left')
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    drive(-460, max_speed=250)
    drive_base.stop()
    right_motor.run_angle(-500, 180)
    drive(20)
    drive_base.settings(straight_acceleration=5000)
    drive(-40)
    drive_base.settings(straight_acceleration=1000)
    use_motor_right.run_angle(-1000, 3200)
    turn(-3, motor="right")
    drive(300, acceleration=200, max_speed=250)
    drive_base.settings(turn_rate=100)
    turn(-148, motor='right')
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    drive(480, max_speed=300)
    turn(65, motor='left')
    drive(155)
    drive_base.stop()
    left_motor.run_angle(500, 100)
    use_motor_left.run_angle(-500, 210)
    use_motor_right.run_angle(-1000, 300)
    drive(-150)
    use_motor_left.run_angle(500, 195)
    turn(120)
    drive(700)

def maneuver3():
    wait(250)
    drive(20)
    drive_base.settings(straight_acceleration=500)
    drive(-40)
    drive_base.settings(straight_acceleration=1000)
    #drive_base.settings(MAX_SPEED, 2000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
    drive(850)
    #drive_base.settings(MAX_SPEED, 1000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
    drive(-100)
    turn(90, motor='right')
    turn(35, motor='right', radius=900)
    drive(-40)
    turn(-35)
    drive(90)
    turn(30)
    drive(90)
    turn(-34, motor='right')
    wait(250)
    turn(10)
    drive(-200)
    turn(-65, motor="left")
    drive(-355)
    turn(55, motor='right')
    drive_base.settings(MAX_SPEED, 2000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
    drive(250)
    drive(20, max_speed=100)
    drive_base.stop()
    left_motor.run_angle(300, 180)
    right_motor.run_angle(300, 180)
    use_motor_right.run_angle(1000, 1700)
    drive_base.settings(MAX_SPEED, 1000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
    drive(-80)
    drive_base.settings(turn_rate=64)
    turn(89)
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    drive(-270)
    use_motor_left.run_until_stalled(500, duty_limit=87)
    wait(500)
    use_motor_left.run_until_stalled(-500, duty_limit=90)
    drive(250)
    turn(130, motor='left', radius=200)
    turn(-90, motor='right')
    drive(-175)
    drive(100)
    use_motor_left.run_until_stalled(1000, duty_limit=85)
    drive(-100)
    use_motor_left.run(-100)
    wait(250)
    drive(20)
    drive(10)
    drive(-10)
    drive_base.settings(MAX_SPEED, 1000, DEFAULT_TURN_RATE, DEFAULT_TURN_ACCELERATION)
    wait(250)
    use_motor_left.hold()
    wait(100)
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE/4)
    turn(45, motor='left', radius=AXLE_TRACK/4)
    drive_base.settings(turn_rate=DEFAULT_TURN_RATE)
    use_motor_left.run(-100)
    wait(100)
    use_motor_left.hold()
    drive(90)
    turn(92)
    drive(-400)

def maneuver4():
    drive_base.stop()
    drive_base.use_gyro(True)
    turn(-56, motor="left", radius=320)
    drive_base.stop()
    drive_base.use_gyro(False)
    use_motor_right.run_until_stalled(-500, duty_limit=90)
    drive(-770)
    turn(60, motor="left", radius=270)
    drive(250)
    use_motor_left.run_angle(-200,100)
    use_motor_left.run_angle(-100,75)
    use_motor_left.run_angle(400,300)
    use_motor_right.run_angle(100,50)
    drive(120)
    turn(45, motor="left")
    drive(70)
    drive(-80)
    use_motor_right.run_angle(-100,70)
    drive_base.settings(straight_acceleration=250)
    drive(100)
    drive_base.settings(straight_acceleration=1000)
    use_motor_right.run_angle(100,80)
    turn(-45, motor="left")
    drive(-250)
    turn(65, motor="right")
    drive(-420)
    use_motor_right.run_until_stalled(150, duty_limit=90)

def test():
    use_motor_left.run_until_stalled(-700, duty_limit=75)