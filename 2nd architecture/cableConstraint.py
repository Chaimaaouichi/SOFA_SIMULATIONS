import Sofa
import os
from stlib3.scene import Scene
from stlib3.physics.deformable import ElasticMaterialObject
from stlib3.visuals import VisualModel
from PIDController import FingerController

path = os.path.dirname(os.path.abspath(__file__)) + '/mesh/'

def createScene(rootNode):
    rootNode.addObject("RequiredPlugin", pluginName=["SoftRobots", 'SofaConstraint', 'SofaEngine', 'SofaLoader', 'SofaDeformable', 'Sofa.Component.AnimationLoop', 'Sofa.Component.Constraint.Lagrangian.Correction', 'Sofa.Component.Constraint.Lagrangian.Solver',
        'Sofa.Component.Engine.Select', 'Sofa.Component.LinearSolver.Direct', 'Sofa.Component.Mapping.Linear', 'Sofa.Component.Mass', 'Sofa.Component.ODESolver.Backward', 'Sofa.Component.SolidMechanics.FEM.Elastic',
        'Sofa.Component.SolidMechanics.Spring', 'Sofa.Component.StateContainer', 'Sofa.Component.Topology.Container.Dynamic', 'Sofa.Component.Visual'])
    scene = Scene(rootNode, gravity=[0.0, -9810.0, 0.0], dt=0.001)

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
                        [-17.5, 12.5, 7.5],[-20.8, 12.5, 7.5],[-24.1, 12.5, 7.5],[-27.4, 12.5, 7.5],[-30.7, 12.5, 7.5],[-34.0, 12.5, 7.5],[-37.3, 12.5, 7.5],[-40.6, 12.5, 7.5],[-43.9, 12.5, 7.5],[-47.2, 12.5, 7.5],[-50.5, 12.5, 7.5],[-53.8, 12.5, 7.5],[-57.1, 12.5, 7.5],[-60.4, 12.5, 7.5],[-63.7, 12.5, 7.5],[-67.0, 12.5, 7.5],[-70.3, 12.5, 7.5],[-73.6, 12.5, 7.5],[-76.9, 12.5, 7.5],[-80.2, 12.5, 7.5],[-83.5, 12.5, 7.5]
])
                        

    cable.addObject('CableConstraint', name="aCableActuator", indices=list(range(0, 21)), pullPoint=[0.0, 12.5, 7.5])
    cable.addObject('BarycentricMapping')
    
    # Instantiate the FingerController
    finger_controller = FingerController(node=cable)
    cable.addObject(finger_controller)

    finger.addChild(VisualModel(visualMeshPath=os.path.join(path, "finger.stl"), color=[0.0, 0.7, 0.7]))
    finger.VisualModel.addObject('BarycentricMapping', name='mapping')
    min_pos_index = finger_controller.minPosIndex
    if min_pos_index is None:
        raise ValueError("minPosIndex is not set in FingerController")

    print(f"Index of the point to track: {min_pos_index}")

    finger.addObject('Monitor',
                     template='Vec3d',
                     name='MonitorTrackedPoint',
                     listening='1',
                     indices=str(min_pos_index),
                     showPositions='1',
                     PositionsColor='0 1 0 1',  # Green color for positions
                     sizeFactor='5.0',
                     ExportPositions='true')  # Enable export of positions

    return rootNode

