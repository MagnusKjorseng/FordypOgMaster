import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/magnus/Documents/FordypOgMaster/Simulator/V4/install/controller'
