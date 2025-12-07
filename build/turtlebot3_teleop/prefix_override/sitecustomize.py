import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/fabin/PycharmProjects/turtlebot3_research_project/install/turtlebot3_teleop'
