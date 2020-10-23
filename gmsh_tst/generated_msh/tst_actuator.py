import Sofa
import sys
import os
from stlib.physics.constraints import FixedBox
from stlib.scene import Scene
from splib.animation import animate



Translation = [0, 0, 0]

BoxROICoordinates=[-5 + Translation[0], -100 + Translation[1], -5 + Translation[2],  5 + Translation[0], -100+4.5 + Translation[1], 5 + Translation[2]]
path = os.path.dirname(os.path.abspath(__file__)) + '/'


def createSceneReal(rootNode):
    #print sys.argv
    #sys.exit()
    length_scale = sys.argv[1]
    #length_scale = ""
    disk_msh = 'disk_'+length_scale+'.msh'
    disk_inside_msh = 'disk_inside'+length_scale+'.msh'
    rootNode = Scene(rootNode, gravity=[0.0, -0.0, 0.0], dt=0.001)
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
    
    bunny.createObject('MeshGmshLoader', name='loader', rotation="90 0 0", filename=path + disk_msh)
    bunny.createObject('TetrahedronSetTopologyContainer', src='@loader', name='container')
    bunny.createObject('TetrahedronSetTopologyModifier')
    bunny.createObject('TetrahedronSetTopologyAlgorithms', template='Vec3d')
    bunny.createObject('TetrahedronSetGeometryAlgorithms', template='Vec3d')
    
    bunny.createObject('MechanicalObject', name='tetras', template='Vec3d', showIndices='false',
                       showIndicesScale='4e-5', rx='0', dz='0')
    bunny.createObject('UniformMass', totalMass='0.5')
    bunny.createObject('TetrahedronFEMForceField', template='Vec3d', name='FEM', method='large', poissonRatio='0.3',
                       youngModulus='180')
    
    #bunny.createObject('BoxROI', name='boxROI', box='-3 -3 -3  3 3 3', drawBoxes='true',
    #                   position="@tetras.rest_position", tetrahedra="@container.tetrahedra")
    #bunny.createObject('RestShapeSpringsForceField', points='@boxROI.indices', stiffness='1e12')
    
    bunny.createObject('SparseLDLSolver', name='preconditioner')
    bunny.createObject('LinearSolverConstraintCorrection', solverName='preconditioner')
    # bunny.createObject('UncoupledConstraintCorrection')
    
    # bunny/cavity
    cavity = bunny.createChild('cavity')
    cavity.createObject('MeshGmshLoader', name='loader', rotation="90 0 0", filename=path + disk_inside_msh)
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

    #FixedBox(bunny, doVisualization=True, atPositions=BoxROICoordinates)
    #bunny.FixedBox.BoxROI.show = True
    return rootNode


def createScene(rootNode):

    def animation(target, factor):
        #target.angleIn = math.cos(factor * 2 * math.pi)

        x = factor+1
        #print(factor)
        #if factor == 0:
        #    keyboard.press_and_release("v")
        #sys.exit(0)

    def ExitFunc(target, factor):
        import sys
        sys.exit(0)
        import matplotlib.pyplot as plt
        plt.plot([1,2,3,4])
        plt.show()



    #Scene(rootNode)

    #rootNode.dt = 0.003
    #rootNode.gravity = [0., -9810., 0.]
    #rootNode.createObject("VisualStyle", displayFlags="showBehaviorModels")
    
    # Use these components on top of the scene to solve the constraint "StopperConstraint".
    #rootNode.createObject("FreeMotionAnimationLoop")
    #rootNode.createObject("GenericConstraintSolver", maxIterations=1e3, tolerance=1e-5)

    #simulation = rootNode.createChild("Simulation")
    #simulation.createObject("EulerImplicitSolver", rayleighStiffness=0.1, rayleighMass=0.1)
    #simulation.createObject("CGLinearSolver", name="precond")
    createSceneReal(rootNode)
    #ServoMotor(simulation, showWheel=True)
    animate(animation, {"target": None}, duration=10, mode="once", onDone=ExitFunc)

    return rootNode

