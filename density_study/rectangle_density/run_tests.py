import os, sys



mesh_factors = [2.0, 1.0, 0.5, 0.25, 0.125, 0.062]
mesh_factors = [2.0, 1.0, 0.95, 0.9,0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45,0.4, 0.35, 0.3,0.25, 0.2]
mesh_factors = [2.0, 1.0]
import numpy as np
mesh_factors = np.linspace(1.5,10,18)
for i in mesh_factors:
    size = "{:<05}".format(i)
    cmd_string = "(time ~/Documents/sofa/build/master/bin/runSofa block_test.py --start --argv "+size+") 2> output"+size+".txt"
    print(cmd_string)
    os.system(cmd_string)


