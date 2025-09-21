import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dhanushya/ws_mobile/install/wall_follower_robot'
