import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ben/KV6022_limo_ros2/src/Week2_lab/install/Week2_lab'
