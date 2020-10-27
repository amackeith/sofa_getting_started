import os, sys



original_mesh_factors = [2.0, 1.0, 0.5, 0.25, 0.125, 0.625]
mesh_factors = [0.95, 0.85, 0.75, 0.65, 0.55, 0.45, 0.35]
mesh_factors = [0.3,0.25, 0.2, 0.15]

for i in mesh_factors:
    size = "{:<05}".format(i)
    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -3 -format msh -o disk_"+size+".msh disk.brep"
    print(cmd_string)
    os.system(cmd_string)

    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -format stl -o disk_inside"+size+".stl disk_inside.brep"
    print(cmd_string)
    os.system(cmd_string)

