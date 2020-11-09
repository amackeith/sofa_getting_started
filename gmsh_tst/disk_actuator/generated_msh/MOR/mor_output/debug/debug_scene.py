# -*- coding: utf-8 -*-
import os
import imp
import platform
from sys import argv

#   STLIB IMPORT
try:
	from stlib.scene.wrapper import Wrapper
except:
    raise ImportError("ModelOrderReduction plugin depend on SPLIB"\
                     +"Please install it : https://github.com/SofaDefrost/STLIB")

# MOR IMPORT
from mor.utility import sceneCreation as u

slash = '/'
if "Windows" in platform.platform():
    slash = "\\"

# Our Original Scene IMPORT
originalScene = r'/home/arthur/Documents/sofa/my_projects/test_scene/gmsh_tst/disk_actuator/generated_msh/MOR/disk_actuator.py'
originalScene = os.path.normpath(originalScene)
originalScene = imp.load_source(originalScene.split(slash)[-1], originalScene)

paramWrapper = ('/disk', {'paramForcefield': {'periodSaveGIE': 6, 'prepareECSW': True, 'modesPath': '/home/arthur/Documents/sofa/my_projects/test_scene/gmsh_tst/disk_actuator/generated_msh/MOR/mor_output/data/modes.txt', 'nbTrainingSet': 8}, 'paramMORMapping': {'input': '@../MechanicalObject', 'modesPath': '/home/arthur/Documents/sofa/my_projects/test_scene/gmsh_tst/disk_actuator/generated_msh/MOR/mor_output/data/modes.txt'}, 'paramMappedMatrixMapping': {'object1': '@./MechanicalObject', 'object2': '@./MechanicalObject', 'template': 'Vec1d,Vec1d', 'timeInvariantMapping2': True, 'performECSW': False, 'timeInvariantMapping1': True, 'nodeToParse': '@./disk'}})

def createScene(rootNode):

    if (len(argv) > 1):
        stateFileName = str(argv[1])
    else:	
        stateFileName="stateFile.state"
    originalScene.createScene(rootNode)

    path , param = paramWrapper
    pathToNode = path[1:]

    u.createDebug(rootNode,pathToNode,stateFileName)