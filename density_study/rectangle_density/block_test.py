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

# scene helper function, defining materials and constraints
def createSceneReal(rootNode, dt,     fixed_const_lst, moving_const_lst):
    # get the files for the given lengthscale

    length_scale = sys.argv[1]
    block_msh = 'block_'+length_scale+'.msh'

    # this sets gravity and dt in the simulator

    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 1000.0], dt=dt)
    # marks a required plugin and some visual style stuff

    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    # animation loop used for legrangian constraints
    rootNode.createObject('FreeMotionAnimationLoop')\
    # linear solver (with parameters from example file)
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    # set color of background, just visual style
    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    # YM of the material. All in kg / m / s
    YoungModulus = 1800
    Translation = None
    # elestic material from prefab
    Block = ElasticMaterialObject(name="block",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+block_msh,
                                  surfaceMeshFileName=None,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation=None)
    
    fixed_const_str = ""
    # these points will be fixed in place
    for i in fixed_const_lst:
        fixed_const_str = fixed_const_str + " " + str(i)

    moving_const_str = ""
    # these points will have a translation imposed on them
    for i in moving_const_lst:
        moving_const_str = moving_const_str + " " + str(i)

    # impose the constraints
    Block.createObject("FixedConstraint", indices=fixed_const_str)
    # the keyTimes are the times (in seconds) that the final translations are reached.
    # movements (here 9 floats) is interpreted as 3, displacement vectors.
    # the constraint takes this input and creates a displacement that occures between the keytimes linearly.
    # after the final keyTime the constraint is released, this is why I added a third keyTime that is after the simulation
    # is over
    Block.createObject('PartialLinearMovementConstraint', indices=moving_const_str, keyTimes='0 '+str(dt)+" 10", template='Vec3d',
                          movements='0. 0. 0.            0. -1 0.               0. -1 0.')

    # Block visualization
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


# "Main" function that runSofa uses to build the scene.
def createScene(rootNode):
    dt = 0.001 # set the time step for the simulator
    # set length scale
    length_scale = sys.argv[1]
    block_msh = 'block_' + length_scale + '.msh'

    # find vertexes on that are at either end (fixed end and moving end)
    # the +/- 1 just sets which extreme it will collect from
    fixed_const_lst = point_finder.edge_nodes(block_msh, 1)

    moving_const_lst = point_finder.edge_nodes(block_msh, -1) #this end has a translation constraint on it.
    # the middle nodes are the ones that will have data collected about them.
    # this accumulates all the nodes within 2*lengths scale of the center of the block (10 units)
    middle_nodes_lst = point_finder.nodes_near_10(block_msh, float(length_scale))

    # three lists to accumulate forces and velocities. THe static one is the fixed end, so
    # only the foces will be non-zero.
    list_of_vectors_static = []
    list_of_vectors_middle = []
    list_of_vectors_mid_piont = []
    mid_point = point_finder.center_point(block_msh) # point closest to the center of the block
    # for some data that isn't averaged.

    # size of mesh
    num_nodes = point_finder.number_of_nodes(block_msh)

    # information about the output.
    info_arr = np.array([float(length_scale), 0, dt, num_nodes])

    print "info array ", info_arr
    print "number of nodes", num_nodes, " num end nodes ", len(fixed_const_lst), " num mid nodes ", len(middle_nodes_lst)
    import timeit
    # simulation timer
    start = timeit.default_timer()

    #animation function called at each step
    def animation(target, factor):

        # collect forces, and velocitires about the vertexes described above.
        forces = np.array(target.block.dofs.force)[fixed_const_lst]
        vels = np.array(target.block.dofs.velocity)[fixed_const_lst]
        mid_forces = np.array(target.block.dofs.force)[middle_nodes_lst]
        mid_vels = np.array(target.block.dofs.velocity)[middle_nodes_lst]

        mid_f = np.array(target.block.dofs.force)[mid_point]
        mid_v = np.array(target.block.dofs.velocity)[mid_point]

        list_of_vectors_static.append(np.array([forces, vels]))
        list_of_vectors_middle.append(np.array([mid_forces, mid_vels]))
        list_of_vectors_mid_piont.append(np.array([mid_f, mid_v]))

    # the exit func is called when duration of time has elapsed, it marks the simulation time
    # saves it, and saves the collected data to an array. And exits the simulation with sys exit
    def ExitFunc(target, factor):
        # save the various data.
        runtime = timeit.default_timer() - start
        info_arr[1] = runtime
        print "runtime", runtime, "number of nodes ", num_nodes

        # the format of each saved array is# [N, 2, 3] N=number of time steps
        # first vector is force, second is velocity.
        np.save('output'+str(length_scale)+'.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_middle.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_middle.npy', np.array(list_of_vectors_static))
        np.save('output' + str(length_scale) + '_mid_point.npy', np.array(list_of_vectors_mid_piont))

        np.save('output'+str(length_scale)+'_info.npy', info_arr)

        sys.exit(0)


    # this is the Sofa animation function we pass it our animation function
    # and along with the exit function.
    createSceneReal(rootNode, dt, fixed_const_lst, moving_const_lst)
    animate(animation, {"target": rootNode}, duration=2.0, mode="once", onDone=ExitFunc)

    return rootNode
    

    
