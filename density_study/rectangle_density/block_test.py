import Sofa
import sys
import os
from stlib.physics.constraints import FixedBox
from stlib.scene import Scene
from splib.animation import animate
from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity
import point_finder
import numpy as np

path = os.path.dirname(os.path.abspath(__file__)) + '/'
meshpath = path
# look at point 90
def createSceneReal(rootNode, dt,     fixed_const_lst, moving_const_lst):
    length_scale = sys.argv[1]
    block_msh = 'block_'+length_scale+'.msh'


    
    
    
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 1000.0], dt=dt)
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.dt = dt

    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')

    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    YoungModulus = 1800
    InitialValue = 0.01
    Translation = None
    Block = ElasticMaterialObject(name="block",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+block_msh,
                                  surfaceMeshFileName=None,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation=None)
    
    fixed_const_str = ""
    for i in fixed_const_lst:
        fixed_const_str = fixed_const_str + " " + str(i)

    moving_const_str = ""
    for i in moving_const_lst:
        moving_const_str = moving_const_str + " " + str(i)

    print 'moving', moving_const_str
    print 'fixed ', fixed_const_str
    
    Block.createObject("FixedConstraint", indices=fixed_const_str)

    inds = "7 63 62 60 5 254 240 206 236 47 68 69 253"

    Block.createObject('PartialLinearMovementConstraint', indices=moving_const_str, keyTimes='0 '+str(dt)+" 10", template='Vec3d',
                          movements='0. 0. 0. 0. -1 0. 0. -1 0.')
    #cavity = PneumaticCavity(name='Cavity', attachedAsAChildOf=Block,
    #                         surfaceMeshFileName=meshpath + block_inside_stl, valueType='pressureGrowth',
    #                         initialValue=InitialValue, translation=Translation)

    
    BlockVisu = Block.createChild('visu')
    BlockVisu.createObject('TriangleSetTopologyContainer', name='container')
    BlockVisu.createObject('TriangleSetTopologyModifier')
    BlockVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    BlockVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    BlockVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    BlockVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    BlockVisu.createObject('IdentityMapping')
    
    return Block



def createScene(rootNode):
    dt = 0.001
    length_scale = sys.argv[1]
    block_msh = 'block_' + length_scale + '.msh'
    # find point closest to zero (with positive z valu)
    vector, mid_ind = point_finder.nth_smallest_point(block_msh, 0)
    fixed_const_lst = point_finder.edge_nodes(block_msh, 1)
    moving_const_lst = point_finder.edge_nodes(block_msh, -1)
    
    middle_nodes_lst = point_finder.nodes_near_10(block_msh, float(length_scale))

    i = 0
    while vector[-1] < 1.0:
        i += 1
        vector, mid_ind = point_finder.nth_smallest_point(block_msh, i)
    
    
    list_of_vectors_static = []
    list_of_vectors_middle = []
    list_of_vectors_mid_piont = []
    mid_point = point_finder.center_point(block_msh)
    num_nodes = point_finder.number_of_nodes(block_msh)
    info_arr = np.array([float(length_scale), 0, dt, num_nodes])
    print "info arra ", info_arr
    print "number of nodes", num_nodes, " num end nodes ", len(fixed_const_lst), " num mid nodes ", len(middle_nodes_lst)
    import timeit
    start = timeit.default_timer()
    def animation(target, factor):
        x = 1
        forces = np.array(target.block.dofs.force)[fixed_const_lst]
        vels = np.array(target.block.dofs.velocity)[fixed_const_lst]
        mid_forces = np.array(target.block.dofs.force)[middle_nodes_lst]
        mid_vels = np.array(target.block.dofs.velocity)[middle_nodes_lst]

        mid_f = np.array(target.block.dofs.force)[mid_point]
        mid_v = np.array(target.block.dofs.velocity)[mid_point]

        list_of_vectors_static.append(np.array([forces, vels]))
        list_of_vectors_middle.append(np.array([mid_forces, mid_vels]))
        list_of_vectors_mid_piont.append(np.array([mid_f, mid_v]))




    
    def ExitFunc(target, factor):
        runtime = timeit.default_timer() - start
        info_arr[1] = runtime
        print "runtime", runtime, "number of nodes ", num_nodes
        np.save('output'+str(length_scale)+'.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_middle.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_middle.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_mid_point.npy', np.array(list_of_vectors_mid_piont))

        np.save('output'+str(length_scale)+'_info.npy', info_arr)

        sys.exit(0)



    createSceneReal(rootNode, dt, fixed_const_lst, moving_const_lst)
    animate(animation, {"target": rootNode}, duration=2.0, mode="once", onDone=ExitFunc)

    return rootNode
    

    
