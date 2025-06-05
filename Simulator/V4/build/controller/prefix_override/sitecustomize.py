import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/Bigboi/Documents/FordypOgMaster/Simulator/V4/install/controller'
