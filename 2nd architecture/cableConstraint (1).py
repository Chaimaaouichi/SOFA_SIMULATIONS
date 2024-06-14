# -*- coding: utf-8 -*-

import Sofa
import os
from stlib3.scene import Scene
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel
from  PIDController import FingerController

path = os.path.dirname(os.path.abspath(__file__))+'/mesh/'

def createScene(rootNode):
    rootNode.addObject("RequiredPlugin", pluginName=["SoftRobots",'SofaConstraint','SofaEngine','SofaLoader','SofaDeformable','Sofa.Component.AnimationLoop','Sofa.Component.Constraint.Lagrangian.Correction','Sofa.Component.Constraint.Lagrangian.Solver',
	'Sofa.Component.Engine.Select','Sofa.Component.LinearSolver.Direct','Sofa.Component.Mapping.Linear','Sofa.Component.Mass','Sofa.Component.ODESolver.Backward','Sofa.Component.SolidMechanics.FEM.Elastic',
	'Sofa.Component.SolidMechanics.Spring','Sofa.Component.StateContainer','Sofa.Component.Topology.Container.Dynamic','Sofa.Component.Visual'])
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0], dt=0.01)

    rootNode.addObject("FreeMotionAnimationLoop")
    rootNode.addObject("GenericConstraintSolver", maxIterations=1000, tolerance=0.001)

    finger = ElasticMaterialObject(name="finger",
                                   volumeMeshFileName=os.path.join(path, "finger.vtk"),
                                   poissonRatio=0.45,
                                   youngModulus=600,
                                   totalMass=0.05)
    rootNode.addChild(finger)

    finger.addObject('BoxROI', name='ROI1', box=[-15, 0, 0, 5, 10, 15], drawBoxes=True)
    finger.addObject('RestShapeSpringsForceField', points='@ROI1.indices', stiffness=1e12)

    cable = finger.addChild('cable')

    cable.addObject('MechanicalObject',
                    name='MechanicalObject',
                    position=[
                        [-17.5, 12.5, 2.5], [-32.5, 12.5, 2.5], [-47.5, 12.5, 2.5], [-62.5, 12.5, 2.5], [-77.5, 12.5, 2.5],
                        [-83.5, 12.5, 4.5], [-85.5, 12.5, 6.5], [-85.5, 12.5, 8.5], [-83.5, 12.5, 10.5],
                        [-77.5, 12.5, 12.5], [-62.5, 12.5, 12.5], [-47.5, 12.5, 12.5], [-32.5, 12.5, 12.5], [-17.5, 12.5, 12.5]])

    cable.addObject('CableConstraint', name="aCableActuator", indices=list(range(0, 14)), pullPoint=[0.0, 12.5, 2.5])
    cable.addObject('BarycentricMapping')

    cable.addObject(FingerController(node=cable))

    finger.addChild(VisualModel(visualMeshPath=os.path.join(path, "finger.stl"), color=[0.0, 0.7, 0.7]))
    finger.VisualModel.addObject('BarycentricMapping', name='mapping')

    return rootNode
