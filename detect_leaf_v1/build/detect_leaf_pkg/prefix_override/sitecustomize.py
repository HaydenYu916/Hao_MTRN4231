import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hao/Desktop/4231SuppliedCode/detect_leaf/install/detect_leaf_pkg'
