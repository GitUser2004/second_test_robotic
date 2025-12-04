import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dylan/repos/second_test_robotic/install/dqn_robot_nav'
