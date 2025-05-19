import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/r/SpotMicroROS2/install/my_robot_tutorials'
