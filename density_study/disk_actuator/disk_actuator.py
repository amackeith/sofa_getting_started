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
def createSceneReal(rootNode, dt):
    length_scale = sys.argv[1]
    disk_msh = 'disk_'+length_scale+'.msh'
    disk_inside_stl = 'disk_inside'+length_scale+'.stl'
    
    
    '''
    # find point closest to zero (with positive z valu) 
    vector, mid_ind = point_finder.nth_smallest_point(disk_msh, 0)
    i = 0
    while vector[-1] < 1.0:
        i += 1
        vector, mid_ind = point_finder.nth_smallest_point(disk_msh, i)

    print(vector, mid_ind)'''
    

    
    
    
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 9.8], dt=dt)
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.dt = 0.001

    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', name='gencs', maxIterations='500', printLog='0', tolerance='0.0000001')
    #disksolver = rootNode.add('SparseLDLSolver', name="solver")
    rootNode.createObject('BackgroundSetting', color='0 0.168627 0.211765')
    YoungModulus = 1800
    InitialValue = 0.01
    Translation = None
    Bunny = ElasticMaterialObject(name="disk",
                                  attachedTo=rootNode,
                                  volumeMeshFileName=meshpath+disk_msh,
                                  surfaceMeshFileName=meshpath+disk_inside_stl,
                                  youngModulus=YoungModulus,
                                  withConstrain=True,
                                  totalMass=1.0,
                                  translation=None)
    
    fixed_const_str = ""
    fixed_const_lst = point_finder.all_nodes_within_and_below_zero(disk_msh, 2.0)
    for i in fixed_const_lst:
        fixed_const_str = fixed_const_str + " " + str(i)
    
    print "hell", fixed_const_lst, fixed_const_str
    
    Bunny.createObject("FixedConstraint", indices=fixed_const_str)

    cavity = PneumaticCavity(name='Cavity', attachedAsAChildOf=Bunny,
                             surfaceMeshFileName=meshpath + disk_inside_stl, valueType='pressureGrowth',
                             initialValue=InitialValue, translation=Translation)

    
    BunnyVisu = Bunny.createChild('visu')
    BunnyVisu.createObject('TriangleSetTopologyContainer', name='container')
    BunnyVisu.createObject('TriangleSetTopologyModifier')
    BunnyVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    BunnyVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    BunnyVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    BunnyVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.6', translation=Translation)
    BunnyVisu.createObject('IdentityMapping')
    
    return Bunny



def createScene(rootNode):
    dt = 0.001
    length_scale = sys.argv[1]
    disk_msh = 'disk_' + length_scale + '.msh'
    # find point closest to zero (with positive z valu)
    vector, mid_ind = point_finder.nth_smallest_point(disk_msh, 0)
    i = 0
    while vector[-1] < 1.0:
        i += 1
        vector, mid_ind = point_finder.nth_smallest_point(disk_msh, i)
    
    print(vector, mid_ind)
    
    list_of_vectors = [np.array([np.array([float(length_scale), 0, dt]), np.array([0.,0.,0.])])]
    import timeit
    start = timeit.default_timer()
    def animation(target, factor):
        x = 1
        #print '\n\n\n\n'
        f = target.disk.dofs.force[mid_ind]
        print(f)
        v = target.disk.dofs.velocity[mid_ind]
        m = target.disk.getData('vertexMass')
        print(type(m))
        list_of_vectors.append(np.array([np.array(f), np.array(v)]))
    
    
    def ExitFunc(target, factor):
        runtime = timeit.default_timer() - start
        list_of_vectors[0][1] = runtime
        print "runtime", runtime
        np.save('output'+str(length_scale)+'.npy', np.array(list_of_vectors))
        sys.exit(0)



    createSceneReal(rootNode, dt)
    animate(animation, {"target": rootNode}, duration=2, mode="once", onDone=ExitFunc)

    return rootNode
    

    
