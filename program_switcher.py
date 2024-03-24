from pybricks.hubs import PrimeHub
from pybricks.tools import wait, StopWatch
from pybricks.parameters import Button, Color
from main import *

hub = PrimeHub()
hub.system.set_stop_button(Button.BLUETOOTH)

#If smaller than 0 test is automatically launched
current_program_index = 0 #program numbers start from 0 (0,1,2,3), -1 being test
automatically_start = False #Needs to be true in order for the autolaunch to work

button_hold = False
program_colors = [Color.RED, Color.ORANGE, Color.YELLOW, Color.GREEN, Color.CYAN, Color.BLUE, Color.VIOLET]

while watch.time() < 1000:
    voltage = hub.battery.voltage()
    for y in range(0, 5):
        for x in range(0,5):
            hub.display.pixel(y, x, (voltage-(7900+100*(4-y))))

if automatically_start:
    if current_program_index >= 0:
        program = 'maneuver' + str(current_program_index+1) + '()'
    else: 
        program = 'test()'
    eval(program)
    current_program_index += 1

#program switcher
while True:
    hub.display.char(str(current_program_index+1))
    if Button.LEFT in hub.buttons.pressed() and not button_hold:
        button_hold = True
        current_program_index = (current_program_index-1)%4
    if Button.RIGHT in hub.buttons.pressed() and not button_hold:
        button_hold = True
        current_program_index = (current_program_index+1)%4
    hub.light.on(program_colors[current_program_index])
    if Button.CENTER in hub.buttons.pressed() and not button_hold:
        button_hold = True
        program = 'maneuver' + str(current_program_index+1) + '()'
        current_program_index = (current_program_index+1)%4
        eval(program)
        use_motor_left.stop()
        use_motor_right.stop()
        left_motor.stop()
        right_motor.stop()
    if Button.RIGHT in hub.buttons.pressed() and Button.LEFT in hub.buttons.pressed() and Button.CENTER in hub.buttons.pressed():
        test()
    if not Button.RIGHT in hub.buttons.pressed() and not Button.LEFT in hub.buttons.pressed() and not Button.CENTER in hub.buttons.pressed():
        button_hold = False