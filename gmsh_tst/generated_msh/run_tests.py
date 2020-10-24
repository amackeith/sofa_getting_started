import os, sys



original_mesh_factors = [2.0, 1.0, 0.5, 0.25, 0.125, 0.625]
mesh_factors = [0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3]
for i in mesh_factors:
    size = "{:<05}".format(i)
    cmd_string = "(time ~/Documents/sofa/build/master/bin/runSofa tst_actuator.py --start --argv "+size+") 2> output"+size+".txt"
    print(cmd_string)
    #os.system(cmd_string)


