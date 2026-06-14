import sys
import os

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from andrew_robot.config import AndrewConfig

def main():
    try:
        config = AndrewConfig('D:\\Resources\\andrew.xml')
        print("arm_deflection:", config.arm_deflection)
        print("tip_geometry_offset:", config.tip_geometry_offset)
    except Exception as e:
        print(f"Error parsing config: {e}")

if __name__ == '__main__':
    main()
