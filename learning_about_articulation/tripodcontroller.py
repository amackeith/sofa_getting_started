import Sofa
from tutorial import *
from splib.numerics import RigidDof
from splib.animation import animate
from splib.constants import Key
from tripod import Tripod

def setupanimation(actuators, step, angularstep, factor):
    """This function is called repeatidely in an animation.
       It moves the actuators by translating & rotating them according to the factor
       value.
    """
    for actuator in actuators:
        rigid = RigidDof( actuator.ServoMotor.BaseFrame.dofs )
        rigid.setPosition( rigid.rest_position + rigid.forward * step * factor )
        actuator.angleIn = angularstep * factor
        
def saveTripodPosition(actuators, step, angularstep, factor):
        t = []
        for actuator in actuators:
                t.append(actuator.ServoMotor.BaseFrame.dofs.getData("position"))
                t.append(actuator.ServoMotor.getData("angleIn"))

        dumpPosition(t, "tripodRestPosition.json")


class TripodController(Sofa.PythonScriptController):
    """This controller has two roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if thr user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
    """

    def __init__(self, node, actuators):
        self.stepsize = 0.1
        self.actuators = actuators

    def onKeyPressed(self, key):
        self.initTripod(key)
        self.animateTripod(key)

    def initTripod(self, key):
        if key == Key.A:
            animate(setupanimation, 
                    {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, 
                    duration=0.2,
                    onDone=saveTripodPosition)

    def animateTripod(self, key):
        if key == Key.uparrow:
            self.actuators[0].ServoMotor.angleIn = self.actuators[0].ServoMotor.angleOut + self.stepsize
        elif key == Key.downarrow:
            self.actuators[0].ServoMotor.angleIn = self.actuators[0].ServoMotor.angleOut - self.stepsize

        if key == Key.leftarrow:
            self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut + self.stepsize
        elif key == Key.rightarrow:
            self.actuators[1].ServoMotor.angleIn = self.actuators[1].ServoMotor.angleOut - self.stepsize

        if key == Key.plus:
            self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut + self.stepsize
        elif key == Key.minus:
            self.actuators[2].ServoMotor.angleIn = self.actuators[2].ServoMotor.angleOut - self.stepsize


class TripodControllerWithCom(TripodController):
    """This controller has three roles:
       - if the user presses up/left/right/down/plus/minus, the servomotor angle
         is changed.
       - if thr user presses A, an animation is started to move the servomotor to the initial position
         of the real robot.
       - if thr user presses B start the communication with controller card, send
         servomotor commands
    """

    def __init__(self, node, actuators, serialportctrl):
        TripodController.__init__(self, node, actuators)
        self.serialportctrl = serialportctrl

    def initTripod(self, key):
        if key == Key.A and self.serialportctrl.state == "init":
            self.serialportctrl.state = "no-comm"
            animate(setupanimation, {"actuators": self.actuators, "step": 35.0, "angularstep": -1.4965}, duration=0.2)

        # Inclusion of the keystroke to start data sending = establishing communication ('comm')
        if key == Key.B and self.serialportctrl.state == "no-comm":
            self.serialportctrl.state = "comm"



def createScene(rootNode):

    scene = Scene(rootNode)
    scene.VisualStyle.displayFlags = "showBehavior"

    tripod = Tripod(scene.Modelling, numMotors=1)

    TripodController(scene, [tripod.ActuatedArm0,])
    #tripod.ActuatedArm1,
    #tripod.ActuatedArm2])

    scene.Simulation.addChild(tripod.RigidifiedStructure)
    motors = scene.Simulation.createChild("Motors")
    motors.addChild(tripod.ActuatedArm0)
    #motors.addChild(tripod.ActuatedArm1)
    #motors.addChild(tripod.ActuatedArm2)
