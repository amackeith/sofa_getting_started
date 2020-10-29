import os, sys



mesh_factors = [2.0, 1.0, 0.5, 0.25, 0.125, 0.062]
mesh_factors = [0.95, 0.85, 0.75, 0.65, 0.55, 0.45, 0.35]
#mesh_factors = [0.3,0.25, 0.2, 0.15]
mesh_factors =  [0.95, 0.9,0.85, 0.8, 0.75, 0.7, 0.65, 0.6, 0.55, 0.5, 0.45,0.4, 0.35, 0.3,0.25, 0.2, 0.15]

for i in mesh_factors:
    size = "{:<05}".format(i)
    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -3 -format msh22 -o block_"+size+".msh rectangle.brep"
    print(cmd_string)
    os.system(cmd_string)
    '''
    cmd_string = "gmsh -algo meshadapt -clscale "+size+" -1 -2 -format stl -o block_outside"+size+".stl rectangle.brep"
    print(cmd_string)
    os.system(cmd_string)
    '''

