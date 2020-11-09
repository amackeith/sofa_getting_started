import os, sys



import numpy as np
time_steps = [0.0001, 0.001, 0.01, 0.1, 1.0, 2.0]

for i in time_steps:
    size = "{:<05}".format(i)
    cmd_string = "(time ~/Documents/sofa/build/master/bin/runSofa block_test.py --start --argv "+size+") 2> output"+size+".txt"
    print(cmd_string)
    os.system(cmd_string)


