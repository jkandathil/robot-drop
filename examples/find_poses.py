from context import andrew_robot

import time
from andrew_robot import AndrewRobot

def main():
    # The ports and config file path will likely need to be changed to try this example
    robot = AndrewRobot('D:\\Resources\\andrew.xml', 'COM7', 250000, 'COM8')
    robot.max_speed = 50
    # LED needs time to init
    time.sleep(.1)

    robot.led_arm(0)
    robot.led_body(0)

    robot.disable_torque()
    while True:
        print(robot.get_servo_positions())
        input()
        pass

if __name__ == '__main__':
    main()