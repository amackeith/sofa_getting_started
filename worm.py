import os 
import Sofa
from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from splib.objectmodel import SofaPrefab, SofaObject
from stlib.physics.mixedmaterial import Rigidify
from stlib.components import addOrientedBoxRoi
from splib.numerics import vec3
from splib.objectmodel import setData, setTreeData
from splib.numerics.quat import Quat
from splib.objectmodel import SofaPrefab, SofaObject
from splib.numerics import getOrientedBoxFromTransform

dirPath = os.path.dirname(os.path.abspath(__file__))+'/'

@SofaPrefab
class FixingBox(SofaObject):
    """Fix a set of 'dofs' according to a translation & orientation"""

    def __init__(self, parent, target, name="FixingBox",
                 translation=[0.0, 0.0, 0.0], eulerRotation=[0.0, 0.0, 0.0], scale=[1.0, 1.0, 1.0]):

        ob = getOrientedBoxFromTransform(translation=translation, eulerRotation=eulerRotation, scale=scale)

        self.node = parent.createChild(name)
        self.node.createObject("BoxROI",
                               orientedBox=ob,
                               name="BoxROI",
                               position=target.dofs.getData("rest_position"),
                               drawBoxes=False)

        c = self.node.createChild("Constraint")
        target.addChild(c)

        c.createObject('RestShapeSpringsForceField',
                       points=self.node.BoxROI.getData("indices"),
                       stiffness='1e12')


def ElasticBody(parent, translation = [0., 0., 0.]):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="mesh/cube_low_res.msh",
                              translation=translation, rotation=[0, 0, 0],
                              youngModulus=10, poissonRatio=0.45, totalMass=0.032)

    '''
    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename=dirPath +\
                                                 "tripod_data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())
    '''
    return body



def myfloor(parentNode=None, name="floor", translation="0 0 0", rotation="0 0 0"):
    fl = parentNode.createChild(name)
    fl.createObject("MeshTopology", name="Topology "+name, filename="mesh/floor.obj", scale=0.03)
    fl.createObject("MechanicalObject", name="Particles "+ name)
    fl.createObject("TriangleModel", name="Triangle for collision floor", moving="0",\
        simulated="0")


def arm(node):

    arm = node.createChild("Arm")

    arm.createObject('MeshGmshLoader', name='meshLoaderCoarse', \
    filename=dirPath+"mesh/arm_low.msh", scale="0.5")
    print(dirPath+"mesh/arm_low.msh")

    arm.createObject("EulerImplicitSolver", name="EulerImplicit arm")
    arm.createObject("SparseLDLSolver", name="Solver arm")
    
    arm.createObject("TetrahedronSetTopologyContainer", name="armtopo", src="@./meshLoaderCoarse")
    arm.createObject("TetrahedronSetGeometryAlgorithms", template="Vec3d", name="GeomAlgo")

    arm.createObject("MechanicalObject", template="Vec3d", name="MechanicalModel",\
    showObject="1", showObjectScale="3")
    arm.createObject("TetrahedronFEMForceField", name="FEM", \
    youngModulus="18000", poissonRatio="0.4", method="large")
    
    arm.createObject("MeshMatrixMass", massDensity="1")

    arm_visual = arm.createChild("Arm_visual")
    arm_visual.createObject("OglModel", name="VisualModel", src="@../meshLoaderCoarse", color="blue")
    arm_visual.createObject("IdentityMapping", name="Mapping", input="@../MechanicalModel", output="@VisualModel")
    
    '''
    arm_collision = arm.createChild("Arm_collision")
    arm_collision.createObject("MeshTopology", src="@../meshLoaderCoarse")
    arm_collision.createObject("MechanicalObject", name="CollisionMO", scale="1.0")
    arm_collision.createObject("TriangleCollisionModel", name="CollisionModel", contactStiffness="199")
    arm_collision.createObject("BarycentricMapping", name="CollisionMapping", input="@../MechanicalModel")
    '''
    arm.createObject("TriangleModel", \
         name="Triangles for collision arm")
    arm.createObject("LineModel", \
         name="Line for collision arm")
    arm.createObject("PointModel", \
         name="Points for collision arm")
    arm.createObject("LinearSolverConstraintCorrection")

def createScene(node):

    node.gravity = "0 -9.8 0"
    node.name="root"




    required_plugins = ("SofaMiscCollision", "SofaSparseSolver","SofaOpenglVisual")
    for i in required_plugins:
        node.createObject("RequiredPlugin", name=i)


    node.createObject("VisualStyle", \
        displayFlags="showBehavior showCollisionModels")
    node.createObject("CollisionPipeline")
    node.createObject("BruteForceDetection")
    node.createObject("FreeMotionAnimationLoop")
    node.createObject("GenericConstraintSolver", \
        maxIterations="1000",tolerance="0.001")
    node.createObject("DefaultContactManager", response="FrictionContact",\
         responseParams="mu=0.6")
    node.createObject("MinProximityIntersection", alarmDistance="1",\
         contactDistance="0.5")

    cubeP = "0 0 0  1 0 0  0 1 0  1 1 0  0 0 1  1 0 1  0 1 1  1 1 1"


    translation = [0.0, 30., 0.]

    node.createObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)

    node.createObject("CGLinearSolver", iterations=250, tolerance=1e-20, threshold=1e-20)

    '''
    cube1 = node.createChild("Cube1")
    cube1.createObject("EulerImplicitSolver", name="EulerImplicit Cube1")
    cube1.createObject("SparseLDLSolver", name="Solver Cube1")

    cube1.createObject("MechanicalObject", name="Particles Cube1", \
         template="Vec3d", \
          position=cubeP, \
           translation=translation )
    
    cube1.createObject("MeshTopology", name="Topology Cube1", \
        hexas="0 4 6 2 1 5 7 3")
    cube1.createObject("UniformMass", name="Mass Cube1", totalMass="1")
    cube1.createObject("MeshSpringForceField", name="Springs Cube1", \
     stiffness="10", damping="1")
     
    cube1.createObject("SphereModel", \
         name="Spheres For Collision Cube1", radius="0.25")
    cube1.createObject("LinearSolverConstraintCorrection")
    '''

    ebb = ElasticBody(node, translation)
    eb = ebb.ElasticMaterialObject
    eb.init()
    print(eb.dofs.getData("rest_position"))
    adjustment = [3.0, 3.0, 0.0]
    box = addOrientedBoxRoi(node, position=eb.dofs.getData("rest_position"), name="BoxROI" + str(1),
                            translation=vec3.vadd(translation, adjustment),
                            eulerRotation=[0, 0, 0],  scale=[1, 8, 8])

    box.drawBoxes = True
    box.init()
    groupIndices = []
    groupIndices.append([ind[0] for ind in box.indices])
    #print(groupIndices)
    frames = []
    frames.append(vec3.vadd(translation, adjustment) + list(
        Quat.createFromEuler([0,0, 0], inDegree=True)))

    print("this is frames")
    print(frames)
    rigidifiedstruct = Rigidify(node, eb, groupIndices=groupIndices, \
                                frames=frames, \
                                name="RigidifiedStructure")


    #rigidifiedstruct.DeformableParts.createObject("UncoupledConstraintCorrection", name="UCup deform")
    #rigidifiedstruct.RigidParts.createObject("UncoupledConstraintCorrection", name="UCup rigid")

    # Use this to activate some rendering on the rigidified object
    setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
    setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
            drawMode=1, showColor=[1., 1., 0., 1.])
    setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)


    myfloor(node, translation="0 -100 0")
    t_plus_a = translation
    adjustment2 = [7, 0, 0]
    for i, v in enumerate(adjustment2):
        t_plus_a[i] -= v
    FixingBox(node, eb, scale=[10, 10, 10], translation=translation)
    node.FixingBox.BoxROI.drawBoxes = True
    '''
    cube2 = node.createChild("Cube2")
    cube2.createObject("EulerImplicitSolver", name="EulerImplicit Cube2")
    cube2.createObject("SparseLDLSolver", name="Solver Cube2")
    cube2.createObject("MechanicalObject", name="Particles Cube2", \
         template="Vec3d", \
          position="0 0 1  1 0 1  0 1 1  1 1 1  0 0 2  1 0 2  0 1 2  1 1 2", \
           translation="0 0 -10" )
    cube2.createObject("MeshTopology", name="Topology Cube2", \
        hexas="0 4 6 2 1 5 7 3")
    cube2.createObject("UniformMass", name="Mass Cube2", totalMass="1")
    cube2.createObject("MeshSpringForceField", name="Springs Cube2", \
     stiffness="10", damping="1")
    cube2.createObject("TriangleModel", \
         name="Triangles for collision Cube2")
    cube2.createObject("LineModel", \
         name="Line for collision Cube2")
    cube2.createObject("PointModel", \
         name="Points for collision Cube2")
    
    cube2.createObject("LinearSolverConstraintCorrection")
    return
    '''
    #arm(node)

    print("HELLOOO")
    #myElasticMaterialObject(node, "mesh/liver.msh", "WithVisual", translation=[10, 50, 10],\
    #    collisionMesh="mesh/liver.obj", surfaceColor=[1, 0,0])


'''


def __attachToActuatedArms(self, radius=60, numMotors=3, angleShift=180.0):
        deformableObject = self.node.ElasticBody.ElasticMaterialObject

        dist = radius
        numstep = numMotors
        groupIndices = []
        frames = []
        for i in range(0, numstep):
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)
            box = addOrientedBoxRoi(self.node, position=deformableObject.dofs.getData("rest_position"), name="BoxROI"+str(i),
                                    translation=vec3.vadd(translation, [0.0, 25.0, 0.0]),
                                    eulerRotation=eulerRotation, scale=[45, 15, 30])

            box.drawBoxes = False
            box.init()
            deformableObject.init()
            groupIndices.append([ind[0] for ind in box.indices])
            frames.append(vec3.vadd(translation, [0.0, 25.0, 0.0]) + list(Quat.createFromEuler([0, float(i)*360/float(numstep), 0], inDegree=True)))

        # Rigidify the deformable part at extremity to attach arms
        rigidifiedstruct = Rigidify(self.node, deformableObject, groupIndices=groupIndices, frames=frames, name="RigidifiedStructure")
        rigidifiedstruct.DeformableParts.createObject("UncoupledConstraintCorrection")
        rigidifiedstruct.RigidParts.createObject("UncoupledConstraintCorrection")

        # Use this to activate some rendering on the rigidified object
        setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
        setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
                drawMode=1, showColor=[1., 1., 0., 1.])
        setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
'''