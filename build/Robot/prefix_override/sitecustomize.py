import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/seba/Desktop/Turtlebot/Turtlebot-estacionador/install/Robot'
