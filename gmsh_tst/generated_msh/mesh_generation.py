import os, sys



mesh_factors = [2.0, 1.0, 0.5, 0.25, 0.125, 0.625]

for i in mesh_factors:
    size = "{:<05}".format(i)
    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -3 -o disk_"+size+".msh disk.brep"
    print(cmd_string)
    os.system(cmd_string)

    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -o disk_inside"+size+".msh disk_inside.brep"
    print(cmd_string)
    os.system(cmd_string)

