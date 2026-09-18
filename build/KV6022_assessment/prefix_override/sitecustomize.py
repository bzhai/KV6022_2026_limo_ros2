import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ben/KV6022_limo_ros2/install/KV6022_assessment'
