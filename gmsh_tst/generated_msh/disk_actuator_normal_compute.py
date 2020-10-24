import Sofa
import sys
import os
from stlib.physics.constraints import FixedBox
from stlib.scene import Scene
from splib.animation import animate
from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity

path = os.path.dirname(os.path.abspath(__file__)) + '/'
meshpath = path

def createSceneReal(rootNode):
    disk_msh = 'disk_'+length_scale+'.msh'
    disk_inside_stl = 'disk_inside'+length_scale+'.stl'
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 0.0], dt=0.001)
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')

    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    rootNode.dt = 0.001

    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', maxIterations='500', printLog='0', tolerance='0.0000001')

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
                                  totalMass=0.5,
                                  translation=None
                                  )

    cavity = PneumaticCavity(name='Cavity', attachedAsAChildOf=Bunny,
                             surfaceMeshFileName=meshpath + 'disk_inside.stl', valueType='pressureGrowth',
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

    def animation(target, factor):
        x = factor+1
        

    def ExitFunc(target, factor):
        sys.exit(0)



    createSceneReal(rootNode)
    animate(animation, {"target": None}, duration=10, mode="once", onDone=ExitFunc)

    return rootNode
    

    
