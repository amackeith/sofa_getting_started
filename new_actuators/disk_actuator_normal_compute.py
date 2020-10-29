import Sofa

import os
from stlib.scene import Scene

from stlib.physics.deformable import ElasticMaterialObject
from softrobots.actuators import PneumaticCavity

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/pipe_climber/'
meshpath = path

def createScene(rootNode):
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
                                  volumeMeshFileName=meshpath+'disk.msh',
                                  surfaceMeshFileName=meshpath+'disk_inside.stl',
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


    

def ieecreateScene(rootNode):
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 0.0], dt=0.0001)
    rootNode.createObject('RequiredPlugin', pluginName='SoftRobots')
    rootNode.createObject('VisualStyle',
                          displayFlags='showVisualModels hideBehaviorModels showCollisionModels hideBoundingCollisionModels hideForceFields showInteractionForceFields hideWireframe')
    
    rootNode.createObject('FreeMotionAnimationLoop')
    rootNode.createObject('GenericConstraintSolver', maxIterations='100', tolerance='0.0000001')
    
    # bunny
    bunny = rootNode.createChild('bunny')
    bunny.createObject('EulerImplicit', name='odesolver')
    bunny.createObject('ShewchukPCGLinearSolver', iterations='15', name='linearsolver', tolerance='1e-5',
                       preconditioners='preconditioner', use_precond='true', update_step='1')
    
    bunny.createObject('MeshGmshLoader', name='loader', filename=path + 'disk.msh')
    bunny.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.createObject('TetrahedronSetTopologyModifier')
    bunny.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
    bunny.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
    
    bunny.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false',
                       showIndicesScale='4e-5', rx='0', dz='0')
    bunny.createObject('UniformMass', totalMass='0.5')
    bunny.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',
                       youngModulus='180000')
    
    #bunny.createObject('BoxROI', name='boxROI', box='-3 -3 -3  3 3 3', drawBoxes='true',
    #                   position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    #bunny.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
    
    bunny.createObject('SparseLDLSolver', name='preconditioner')
    bunny.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')
    # bunny.createObject('UncoupledConstraintCorrection')
    
    # bunny/cavity
    cavity = bunny.createChild('cavity')
    cavity.createObject('MeshGmshLoader', name='loader', filename=path + 'disk_inside.msh')
    cavity.createObject('Mesh', src='@loader', name='topo')
    cavity.createObject('MechanicalObject', name='cavity')
    cavity.createObject('SurfacePressureConstraint', triangles='@topo.triangles', value='4000', valueType="1")
    cavity.createObject('BarycentricMapping', name='mapping', mapForces='false', mapMasses='false')
    
    # bunny/bunnyVisu
    bunnyVisu = bunny.createChild('visu')
    bunnyVisu.createObject('TriangleSetTopologyContainer', name='container')
    bunnyVisu.createObject('TriangleSetTopologyModifier')
    bunnyVisu.createObject('TriangleSetTopologyAlgorithms', template='Vec3d')
    bunnyVisu.createObject('TriangleSetGeometryAlgorithms', template='Vec3d')
    bunnyVisu.createObject('Tetra2TriangleTopologicalMapping', name='Mapping', input="@../container",
                           output="@container")
    
    bunnyVisu.createObject('OglModel', template='ExtVec3f', color='0.3 0.2 0.2 0.8')
    bunnyVisu.createObject('IdentityMapping')


    return rootNode
