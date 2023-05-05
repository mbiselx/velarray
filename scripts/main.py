#!/usr/bin/env python
import sys
import rospkg

# messing around with paths to get imports to work
try:
    rospack = rospkg.RosPack()
    pckg_path = rospack.get_path('velarray')
    sys.path.append(pckg_path)
    import src.velarray_threading as vth
    import src.velarray_multiprocessing as vmp
except Exception as e:
    raise e


threading = True
threading = False

if threading:
    vth.main()
else:
    vmp.main()
