import Sofa
import os
dirPath = os.path.dirname(os.path.abspath(__file__))+'/'
def FloorObj(parentNode=None, name="Floor",
             rotation=[0.0, 0.0, 0.0], translation=[0.0, 0.0, 0.0]):
    
    floor = parentNode.createChild('Floor')

    totalMass = 1.0
    uniformScale = "0.001"
    color = [0.44, 0.44, 0.44]
    surfaceMeshFileName = dirPath+"mesh/floor_fine.stl"

    floor.createObject('MechanicalObject',
                       name="MechState", template="Rigid3", showIndices="false",
                       translation=translation, rotation=rotation, showObjectScale=uniformScale)
    floor.createObject('UniformMass', name="mass", totalMass=totalMass)
    # floor.createObject('UncoupledConstraintCorrection')

    # collision
    objectCollis = floor.createChild('collision')
    objectCollis.createObject('MeshSTLLoader', name="loader",
                              filename=surfaceMeshFileName,
                              triangulate="true",
                              scale="0.001")
    objectCollis.createObject('MeshTopology', src="@loader")
    objectCollis.createObject('MechanicalObject', name='mechanicalObject')

    
    objectCollis.createObject('TTriangleModel', moving=False, simulated=False)
    objectCollis.createObject('TLineModel', moving=False, simulated=False)
    objectCollis.createObject('TPointModel', moving=False, simulated=False)

    
    objectCollis.createObject('RigidMapping')

    # visualization
    modelVisu = floor.createChild('Visual')
    modelVisu.createObject('MeshSTLLoader', name="loader",
                           filename=surfaceMeshFileName)
    modelVisu.createObject('OglModel',
                           src="@loader",
                           template='ExtVec3d',
                           color=color,
                           scale3d=[uniformScale]*3)
    modelVisu.createObject('RigidMapping')

    return floor


def createScene(node):

    node.gravity = "0 -9.8 0"
    node.name="root"

    node.createObject("CollisionPipeline", verbose="0", draw="0")
    node.createObject("BruteForceDetection", name="N2")
    node.createObject("NewProximityIntersection", name="Proximity", \
     alarmDistance="1", contactDistance="0.25")

    node.createObject("CollisionResponse", name="Response", response="default")

    node.createObject('MeshGmshLoader', name='meshLoaderCoarse', \
    filename=dirPath+"mesh/arm_low.msh", scale="0.5")

    node.createObject('MeshGmshLoader', name='loader', \
    filename=dirPath+"mesh/body.msh", scale="0.25")

    #node.createObject('MeshObjLoader', name='floorloader', \
    #filename="mesh/floor.obj") ### ideally I want to learn how to use this as the floor.
    
    #### arm     
    
    arm = node.createChild("Arm")

    arm.createObject("EulerImplicitSolver")
    arm.createObject("CGLinearSolver",  iterations="2000",\
    tolerance="1e-09", threshold="1e-09")
    
    arm.createObject("TetrahedronSetTopologyContainer", name="armtopo", src="@../meshLoaderCoarse")
    arm.createObject("TetrahedronSetGeometryAlgorithms", template="Vec3d", name="GeomAlgo")

    arm.createObject("MechanicalObject", template="Vec3d", name="MechanicalModel",\
    showObject="1", showObjectScale="3")
    arm.createObject("TetrahedronFEMForceField", name="FEM", \
    youngModulus="18000", poissonRatio="0.4", method="large")
    
    arm.createObject("MeshMatrixMass", massDensity="1")

    arm_visual = arm.createChild("Arm_visual")
    arm_visual.createObject("OglModel", name="VisualModel", src="@../../meshLoaderCoarse", color="blue")
    arm_visual.createObject("IdentityMapping", name="Mapping", input="@../MechanicalModel", output="@VisualModel")
    

    arm_collision = arm.createChild("Arm_collision")
    arm_collision.createObject("MeshTopology", src="@../../meshLoaderCoarse")
    arm_collision.createObject("MechanicalObject", name="CollisionMO", scale="1.0")
    arm_collision.createObject("TriangleCollisionModel", name="CollisionModel", contactStiffness="3")
    arm_collision.createObject("BarycentricMapping", name="CollisionMapping", input="@../MechanicalModel")
    #arm_collision.createObject("IdentityMapping", name="CollisionMapping", \
    #input="@../MechanicalModel", output="@CollisionModel")
    ### end arm
    
    
    
    
    body = node.createChild("Body")
    body_translation = "5 10 0"

    body.createObject("EulerImplicitSolver")
    body.createObject("CGLinearSolver",  iterations="2000",\
    tolerance="1e-09", threshold="1e-09")
    
    
    body.createObject("TetrahedronSetTopologyContainer", name="bodytopo", src="@../loader")
    body.createObject("TetrahedronSetGeometryAlgorithms", template="Vec3d", name="GeomAlgo")

    body.createObject("MechanicalObject", template="Vec3d", name="BodyMechanicalModel",\
    showObject="1", translation=body_translation, showObjectScale="3")
    body.createObject("TetrahedronFEMForceField", name="BodyFEM", \
    youngModulus="18000", poissonRatio="0.4", method="large")
    
    body.createObject("MeshMatrixMass", massDensity="1")

    
    body_visual = body.createChild("body_visual")
    body_visual.createObject("OglModel", name="VisualModel", src="@../../loader", color="red", \
    translation=body_translation)
    body_visual.createObject("IdentityMapping", name="Mapping", input="@../BodyMechanicalModel", output="@VisualModel")
    
    body_collision = body.createChild("body_collision")
    body_collision.createObject("MeshTopology", src="@../../loader")
    body_collision.createObject("MechanicalObject", name="CollisionMO", scale="1.0", translation=body_translation)
    body_collision.createObject("TriangleCollisionModel", name="CollisionModel", contactStiffness="199")
    body_collision.createObject("BarycentricMapping", name="CollisionMapping", input="@../BodyMechanicalModel")
    body.createObject("ConstantForceField", force="0 -1 0 0 0 0")


    
    ### spehre
    sphere = node.createChild("Sphere")
    sphere.createObject("EulerImplicitSolver", rayleighStiffness="0")
    sphere.createObject("CGLinearSolver", iterations="200", tolerance="1e-06", threshold="1e-06")
    sphere.createObject("MechanicalObject", template="Rigid3d", name="myParticle", \
    position="-5 10 0    0 0 0 1", showObject="1", showObjectScale="0.5")
    
    sphere.createObject("UniformMass", totalMass="1")
    sphere.createObject("ConstantForceField", force="0 -1 0 0 0 0")
    sphere.createObject("SphereCollisionModel", name="Floor", listRadius="1",\
    simulated="1", moving="1", contactStiffness="100")


    selfNode = node.createChild("Floor")

    f2 = FloorObj(selfNode, "Floor",
                  translation=[-75, -5, -75],
                  rotation=[90.0, 0.0, 0.0])

   
