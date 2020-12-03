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
def createSceneReal(rootNode, dt):
    # get the files for the given lengthscale
    length_scale = sys.argv[1]
    disk_msh = 'disk_'+length_scale+'.msh' #this defines the solid volume

    #this defines the surface to which pressure forces are applied to.
    disk_inside_stl = 'disk_inside'+length_scale+'.stl'



    # this sets gravity and dt in the simulator
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 9.8], dt=dt)
    # marks a required plugin and some visual style stuff
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')

    # animation loop used for legrangian constraints
    rootNode.createObject('FreeMotionAnimationLoop')
    # linear solver (with parameters from example file)
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    # YM initial pressure value of the material and cavity. All in kg / m / s
    YoungModulus = 1800
    InitialValue = 0.01
    Translation = None
    # Disk object from prefab function
    disk_object = ElasticMaterialObject(name="disk",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+disk_msh,
                                  surfaceMeshFileName=meshpath+disk_inside_stl,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation=None)


    fixed_const_str = ""
    # returns all nodes below zero Z, within a given radius, in our case this is the bottom side of the disk
    # and the points returned are fixed so the disk doesn't move from the origin from gravity or accumulating
    # momentum error.
    fixed_const_lst = point_finder.all_nodes_within_and_below_zero(disk_msh, 2.0)
    for i in fixed_const_lst:
        fixed_const_str = fixed_const_str + " " + str(i)

    # the fixed constraint is applied to these points, (Thier velocity is projected to zero at each time step).
    disk_object.createObject("FixedConstraint", indices=fixed_const_str)

    # this defines the penumatic cavity, volume growth is also an option.
    cavity = PneumaticCavity(name='Cavity', attachedAsAChildOf=disk_object,
                             surfaceMeshFileName=meshpath + disk_inside_stl, valueType='pressureGrowth',
                             initialValue=InitialValue, translation=Translation)

    # this defines the visualization of the model.
    disk_objectVisu = disk_object.createChild('visu')
    disk_objectVisu.createObject('TriangleSetTopologyContainer', name='container')
    disk_objectVisu.createObject('TriangleSetTopologyModifier')
    disk_objectVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    disk_objectVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    disk_objectVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    disk_objectVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    disk_objectVisu.createObject('IdentityMapping')

    return disk_object


# "Main" function that runSofa uses to build the scene.
def createScene(rootNode):
    #sets simulation time setp
    dt = 0.001
    # length scale passed in thru --argv
    length_scale = sys.argv[1]
    disk_msh = 'disk_' + length_scale + '.msh'

    # find index of vertex closest to the center of the disk (with positive z value)
    # to be tracked and used for mesh study
    vector, mid_ind = point_finder.nth_smallest_point(disk_msh, 0)
    i = 0
    # makes sure the z component is above 1.0 (this means it will be on the
    # upper side of the disk as it inflates, which is the free side, the bottom
    # will be fixed)
    while vector[-1] < 1.0:
        i += 1
        vector, mid_ind = point_finder.nth_smallest_point(disk_msh, i)

    # this list of vectors will accumulate all the data we collect
    # this line is a header, but the rest of it is [T, 2, 3] array,
    # where T is the number of timesteps, and the 2x3 arrays are the force and velocity
    # of the point selected by point_finder.nth_smallest_point
    list_of_vectors = [np.array([np.array([float(length_scale), 0, dt]), np.array([0.,0.,0.])])]
    import timeit #set a timer for the simulator
    start = timeit.default_timer()

    # this animation function gets run at every time step. It is used to collect data
    # target is the root node, as defined below, and factor corresponds to the time in the simulation
    def animation(target, factor):
        # these collect the force, velocity from the selected vertex at each time step
        f = target.disk.dofs.force[mid_ind]
        v = target.disk.dofs.velocity[mid_ind]
        list_of_vectors.append(np.array([np.array(f), np.array(v)]))

    # the exit func is called when duration of time has elapsed, it marks the simulation time
    # saves it, and saves the collected data to an array. And exits the simulation with sys exit
    def ExitFunc(target, factor):
        runtime = timeit.default_timer() - start
        list_of_vectors[0][1] = runtime
        print "runtime", runtime
        np.save('output'+str(length_scale)+'.npy', np.array(list_of_vectors))
        sys.exit(0)



    createSceneReal(rootNode, dt)
    # this is the Sofa animation function we pass it our animation function
    # and along with the exit function.
    animate(animation, {"target": rootNode}, duration=2, mode="once", onDone=ExitFunc)

    return rootNode



