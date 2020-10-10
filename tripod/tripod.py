from splib.numerics import sin, cos, to_radians
from stlib.physics.deformable import ElasticMaterialObject
from actuatedarm import ActuatedArm
from stlib.physics.collision import CollisionMesh
from splib.objectmodel import SofaPrefab, SofaObject
from stlib.physics.mixedmaterial import Rigidify
from stlib.components import addOrientedBoxRoi
from splib.numerics import vec3
from splib.numerics.quat import Quat
from tutorial import *
from splib.objectmodel import setData, setTreeData
from stlib.physics.rigid import Cube


def ElasticBody(parent):
    body = parent.createChild("ElasticBody")

    e = ElasticMaterialObject(body,
                              volumeMeshFileName="data/mesh/tripod_mid.gidmsh",
                              translation=[0.0, 30, 0.0], rotation=[90, 0, 0],
                              youngModulus=800, poissonRatio=0.45, totalMass=0.032)

    visual = body.createChild("Visual")
    visual.createObject("MeshSTLLoader", name="loader", filename="data/mesh/tripod_mid.stl")
    visual.createObject("OglModel", name="renderer", src="@loader", color=[1.0, 1.0, 1.0, 0.5],
                        rotation=[90, 0, 0], translation=[0, 30, 0])

    visual.createObject("BarycentricMapping",
                        input=e.dofs.getLinkPath(),
                        output=visual.renderer.getLinkPath())

    return body


@SofaPrefab
class Tripod(SofaObject):

    def __init__(self, parent, name="Tripod", radius=60, numMotors=3, angleShift=180.0, mapped_dof=None):
        self.node = parent.createChild(name)
        self.mapped_dof = mapped_dof
        ElasticBody(self.node)

        dist = radius
        numstep = numMotors
        self.actuatedarms = []


        '''
        for i in range(0, numstep):
            name = "ActuatedArm"+str(i)
            translation, eulerRotation = self.__getTransform(i, numstep, angleShift, radius, dist)
            arm = ActuatedArm(self.node, name=name,
                              translation=translation, eulerRotation=eulerRotation)
            self.actuatedarms.append(arm)
            # Add limits to angle that correspond to limits on real robot
            arm.ServoMotor.minAngle = -2.0225
            arm.ServoMotor.maxAngle = -0.0255
        '''

        self.__attachToActuatedArms(radius, numMotors, angleShift)

        self.addCollision()

    def __getTransform(self, index, numstep, angleShift, radius, dist):
        fi = float(index)
        fnumstep = float(numstep)
        angle = fi*360/fnumstep
        angle2 = fi*360/fnumstep+angleShift
        eulerRotation = [0, angle, 0]
        translation = [dist*sin(to_radians(angle2)), -1.35, dist*cos(to_radians(angle2))]

        return translation, eulerRotation

    def addCollision(self):
        CollisionMesh(self.node.ElasticBody.ElasticMaterialObject, surfaceMeshFileName="data/mesh/tripod_low.stl", name="CollisionModel", translation=[0.0, 30, 0.0], rotation=[90, 0, 0], collisionGroup=1)

        for arm in self.actuatedarms:
            CollisionMesh(arm.ServoMotor.ServoBody,
                          surfaceMeshFileName="data/mesh/servo_collision.stl",
                          name="TopServoCollision", mappingType='RigidMapping')

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

        # Use this to activate some rendering on the rigidified object ######################################
        setData(rigidifiedstruct.RigidParts.dofs, showObject=True, showObjectScale=10, drawMode=2)
        setData(rigidifiedstruct.RigidParts.RigidifiedParticules.dofs, showObject=True, showObjectScale=1,
                 drawMode=1, showColor=[1., 1., 0., 1.])
        setData(rigidifiedstruct.DeformableParts.dofs, showObject=True, showObjectScale=1, drawMode=2)
        #####################################################################################################

        # Attach arms
        if self.mapped_dof != None:
            rigidifiedstruct.RigidParts.createObject('RestShapeSpringsForceField', name="rssff" + str(i),
                                                     points=0,
                                                     external_rest_shape=self.mapped_dof.getLinkPath(), index=1,
                                                     stiffness='1e12', angularStiffness='1e12')
        '''
        print(self.actuatedarms[0].servoarm.dofs)

        for i in range(0, numstep):
            rigidifiedstruct.RigidParts.createObject('RestShapeSpringsForceField', name="rssff"+str(i),
                                                     points=i,
                                                     external_rest_shape=self.actuatedarms[i].servoarm.dofs,
                                                     stiffness='1e12', angularStiffness='1e12')

            print("\n\n\n\n\n\n\n\n\n\n")
            print (dir(self.actuatedarms[i].servoarm.dofs))
            print("|hello")
            #exit()
        '''



def myfloor(parentNode, translation):
    dst = -88
    myfloorsm(parentNode, translation=[0, 0, 0], name="fl1")

    myfloorsm(parentNode, translation=[dst, 0, 0], name="fl2")

    myfloorsm(parentNode, translation=[dst, 0, dst], name="fl3")
    myfloorsm(parentNode, translation=[0, 0, dst], name="fl4")

def myfloorsm(parentNode=None, name="floor", translation="0 0 0", rotation="0 0 0"):
    fl = parentNode.createChild(name)
    fl.createObject("MeshTopology", name="Topology "+name, filename="mesh/floor.obj", scale=0.03,
                    )
    fl.createObject("MechanicalObject", name="Particles "+ name, translation=translation)
    fl.createObject("TriangleCollisionModel", name="Triangle for collision floor", moving="0",\
        simulated="0")

    #fl.createObject("SphereCollisionModel", name="Floor", listRadius="5", \
    #                    simulated="0", moving="0", contactStiffness="100000")

    ### spehre

    return fl


def sphere(parentNode):
    sphere = parentNode.createChild("Sphere")
    #sphere.createObject("EulerImplicitSolver", rayleighStiffness="0.1")
    #sphere.createObject("CGLinearSolver", iterations="200", tolerance="1e-06", threshold="1e-06")
    sphere.createObject("MechanicalObject", template="Rigid3d", name="myParticle", \
                        position="-5 10 0    0 0 0 1", showObject="1", showObjectScale="0.5")

    sphere.createObject("UniformMass", totalMass="1")
    sphere.createObject("ConstantForceField", force="0 -1 0 0 0 0")
    sphere.createObject("SphereCollisionModel", name="Floor", listRadius="5", \
                        simulated="1", moving="1", contactStiffness="100000")
    sphere.createObject("LinearSolverConstraintCorrection")
    return sphere

def cube(rootNode):
    cubeP = "0 0 0  1 0 0  0 1 0  1 1 0  0 0 1  1 0 1  0 1 1  1 1 1"

    cube1 = rootNode.createChild("Cube1")
    cube1.createObject("EulerImplicitSolver", name="EulerImplicit Cube1")
    cube1.createObject("SparseLDLSolver", name="Solver Cube1")

    cube1.createObject("MechanicalObject", name="Particles Cube1", \
                       template="Vec3d", \
                       position=cubeP, \
                       translation=[0, 23, 0])

    cube1.createObject("MeshTopology", name="Topology Cube1", \
                       hexas="0 4 6 2 1 5 7 3")
    cube1.createObject("UniformMass", name="Mass Cube1", totalMass="1")
    cube1.createObject("MeshSpringForceField", name="Springs Cube1", \
                       stiffness="10", damping="1")

    cube1.createObject("SphereModel", \
                       name="Spheres For Collision Cube1", radius="0.25")
    cube1.createObject("LinearSolverConstraintCorrection")


@SofaPrefab
class myCube(SofaObject):

    def __init__(self, parent, translation=[0, 23.64, -60.6], rotation="0 0 0", scale=[1.0,1.0,1.0]):

        self.node = parent.createChild("articulation")
        self.node.createObject("MechanicalObject", name="Articulations", template="Vec1d", position="0")
        Cube(self.node, name="cubehard", uniformScale=20, translation=translation)

        sub_node = self.node.createChild("sub_node")

        factor = 25
        sub_node.createObject("MechanicalObject", template="Rigid3d", name="DOFs",
                               position=[[0., 0., 0., 0.,0.,0.,1],[0., 0., factor, 0.,0.,0.,1]])
        #sub_node.createObject("BeamFEMForceField", name="FEM", radius="0.1", youngModulus="1e8",
        #                       poissonRatio="0.45")
        sub_node.createObject("MeshTopology", name="Lines", lines="0 1")
        sub_node.createObject("UniformMass", template="Rigid3d", name="Mass",
                               vertexMass="0.1 0.1 [1 0 0,0 1 0,0 0 1]")

        #sub_node.createObject("FixedConstraint", template="Rigid3d", name="fixOrigin", indices=0)
        sub_node.createObject("ArticulatedSystemMapping", input1="@../Articulations",
                              input2="@../cubehard/mstate",
                              output="@DOFs")

        sub_sub_node = sub_node.createChild("arm")
        sub_sub_node.createObject("MechanicalObject", template="Rigid3d", name="DOFs",
                                  position=[0., 0.,factor, 0.,0.,0.,1])
        sub_sub_node.createObject('RigidRigidMapping',
                               name="mapping", input="@../DOFs", index=1)

        self.node.createObject("ArticulatedHierarchyContainer")

        art_cen = self.node.createChild("articulationCenters")
        art1 = art_cen.createChild("articulationCenter1")
        art1.createObject("ArticulationCenter", parentIndex="0", childIndex="1", posOnParent="0 0 0",
                                posOnChild=[0, 0, -1*factor], articulationProcess="2" )

        artt = art1.createChild("articulations")
        artt.createObject("Articulation", translation="0", rotation="1", rotationAxis="1 0 0",
                          articulationIndex="0")

        self.node.createObject("LinearSolverConstraintCorrection")
        '''
        # The inputs
        self.node.addNewData("minAngle", "S90Properties", "min angle of rotation (in radians)", "float", -100)
        self.node.addNewData("maxAngle", "S90Properties", "max angle of rotation (in radians)", "float", 100)
        self.node.addNewData("angleIn", "S90Properties", "angle of rotation (in radians)", "float", 0)

        # Two positions (rigid): first one for the servo body, second for the servo wheel
        t3ranslation = [0, 55, -0]
        baseFrame = self.node.createChild("BaseFrame")
        baseFrame.createObject("MechanicalObject", name="dofs", template="Rigid3",
                               position=[[0., 0., 0., 0., 0., 0., 1.], [0., 0., 0., 0., 0., 0., 1.]],
                               translation=translation, rotation=rotation, scale3d=scale)

        # Angle of the wheel
        angle = self.node.createChild("Angle")
        angle.createObject("MechanicalObject", name="dofs", template="Vec1d",
                           position=self.node.getData("angleIn").getLinkPath())
        # This component is used to constrain the angle to lie between a maximum and minimum value,
        # corresponding to the limit     of the real servomotor
        angle.createObject("ArticulatedHierarchyContainer")
        angle.createObject("ArticulatedSystemMapping", input1=angle.dofs.getLinkPath(),
                           output=baseFrame.dofs.getLinkPath())
        angle.createObject('StopperConstraint', name="AngleLimits", index=0,
                           min=self.node.getData("minAngle").getLinkPath(),
                           max=self.node.getData("maxAngle").getLinkPath())
        angle.createObject("UncoupledConstraintCorrection")

        articulationCenter = angle.createChild("ArticulationCenter")
        articulationCenter.createObject("ArticulationCenter", parentIndex=0, childIndex=1, posOnParent=[0., 0., 0.],
                                        posOnChild=[0., 0., 0.])
        articulation = articulationCenter.createChild("Articulations")
        articulation.createObject("Articulation", translation=False, rotation=True, rotationAxis=[1, 0, 0],
                                  articulationIndex=0)

        #output = self.node.cubehard.mstate.getLinkPath(),
        self.node.cubehard.createObject("RigidRigidMapping", input=self.node.BaseFrame.dofs.getLinkPath(),
                output=self.node.cubehard.mstate.getLinkPath(), index=1)

        arm = self.node.createChild("arm")
        arm.createObject("MechanicalObject", name="dofs", template="Rigid3",
                           position=[0.,0.,0.,0.,0.,0.,1.0])


        arm.createObject("RigidRigidMapping", input=self.node.BaseFrame.dofs.getLinkPath(),
                         output=self.node.arm.dofs.getLinkPath(), index=0)

        # The output
        self.node.addNewData("angleOut", "S90Properties", "angle of rotation (in degree)", "float",
                             angle.dofs.getData("position").getLinkPath())

        self.node.BaseFrame.init()
        self.node.BaseFrame.dofs.rotation = [0., 0., 0.]
        self.node.BaseFrame.dofs.translation = [0., 0., 0.]
        '''







def createScene(rootNode):
    scene = Scene(rootNode)
    rootNode.dt = 0.001







    addContact(scene)
    fl = myfloor(scene.Modelling, translation="0 -100 0")
    #sp = sphere(scene.Modelling)
    scene.VisualStyle.displayFlags = "showBehavior"
    cube = myCube(scene.Modelling)
    #print(dir(cube.mstate))
    tripod = Tripod(scene.Modelling, numMotors=1,
                    mapped_dof=scene.Modelling.articulation.sub_node.arm.DOFs)



    #scene.Simulation.addChild(tripod.RigidifiedStructure)
    scene.Simulation.addChild(scene.Modelling)
    #scene.Simulation.addChild(fl)
    #scene.Simulation.addChild(sp)
    motors = scene.Simulation.createChild("Motors")
    #motors.addChild(tripod.ActuatedArm0)
    #motors.addChild(tripod.ActuatedArm1)
    #motors.addChild(tripod.ActuatedArm2)


