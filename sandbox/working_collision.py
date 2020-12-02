import os 
import Sofa

dirPath = os.path.dirname(os.path.abspath(__file__))+'/'


def myfloor(parentNode=None, name="floor", translation="0 0 0", rotation="0 0 0"):
    fl = parentNode.createChild(name)
    fl.createObject("MeshTopology", name="Topology "+name, filename="mesh/floor.obj")
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


    node.createObject("RequiredPlugin", name="SofaMiscCollision")

    node.createObject("RequiredPlugin", name="SofaOpenglVisual")

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
    
    cube1 = node.createChild("Cube1")
    cube1.createObject("EulerImplicitSolver", name="EulerImplicit Cube1")
    cube1.createObject("SparseLDLSolver", name="Solver Cube1")
    
    cube1.createObject("MechanicalObject", name="Particles Cube1", \
         template="Vec3d", \
          position="0 0 1  1 0 1  0 1 1  1 1 1  0 0 2  1 0 2  0 1 2  1 1 2", \
           translation="0.5 2 30" )
    
    cube1.createObject("MeshTopology", name="Topology Cube1", \
        hexas="0 4 6 2 1 5 7 3")
    cube1.createObject("UniformMass", name="Mass Cube1", totalMass="1")
    cube1.createObject("MeshSpringForceField", name="Springs Cube1", \
     stiffness="100", damping="1")
     
    cube1.createObject("SphereModel", \
         name="Spheres For Collision Cube1", radius="1")
    cube1.createObject("LinearSolverConstraintCorrection")



    cube2 = node.createChild("Cube2")
    cube2.createObject("EulerImplicitSolver", name="EulerImplicit Cube2")
    cube2.createObject("SparseLDLSolver", name="Solver Cube2")
    cube2.createObject("MechanicalObject", name="Particles Cube2", \
         template="Vec3d", \
          position="0 0 1  1 0 1  0 1 1  1 1 1  0 0 2  1 0 2  0 1 2  1 1 2", \
           translation="0 0 30" )
    cube2.createObject("MeshTopology", name="Topology Cube2", \
        hexas="0 4 6 2 1 5 7 3")
    cube2.createObject("UniformMass", name="Mass Cube2", totalMass="1")
    cube2.createObject("MeshSpringForceField", name="Springs Cube2", \
     stiffness="100", damping="1")
    cube2.createObject("TriangleModel", \
         name="Triangles for collision Cube2")
    cube2.createObject("LineModel", \
         name="Line for collision Cube2")
    cube2.createObject("PointModel", \
         name="Points for collision Cube2")
    
    cube2.createObject("LinearSolverConstraintCorrection")

    
    myfloor(node, translation="0 -100 0")
    arm(node)