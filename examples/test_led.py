from context import andrew_robot

import time
from andrew_robot.led import LedController

# The port will likely need to be changed to try this example
led_com = 'COM7'
led_controller = LedController(led_com)
# LED needs time to init
time.sleep(.1)

print("Blinking LED on COM7 three times...")
for _ in range(3):
    led_controller.turn_on(1)
    time.sleep(1)
    led_controller.turn_off(1)
    time.sleep(1)
print("Finished!")