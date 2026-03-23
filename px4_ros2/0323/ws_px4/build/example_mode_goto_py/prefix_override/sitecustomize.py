import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leedonghyuck/ws_px4/install/example_mode_goto_py'
