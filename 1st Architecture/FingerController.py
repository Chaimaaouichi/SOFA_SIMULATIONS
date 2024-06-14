#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
from Sofa.constants import Key

class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.node = kw["node"]
        self.mechanicalObject = self.node.getObject('MechanicalObject')
        self.maxPosIndex = None
        self.findMinXPosition()

    def findMinXPosition(self):
    # Find the position at the end of the shape (which has the smallest x coordinate)
     positions = self.mechanicalObject.findData('position').value
     min_x = float('inf')
     for i, pos in enumerate(positions):
        if pos[0] < min_x:
            self.minPosIndex = i
            min_x = pos[0]
     print(f"Index of min x position: {self.minPosIndex}")
     print(f"Min x position: {positions[self.minPosIndex]}")

    def onKeypressedEvent(self, e):
        inputvalue = self.node.aCableActuator.value
        displacement = inputvalue.value[0]

        if e["key"] == Key.plus:
            displacement += 1.0
        elif e["key"] == Key.minus:
            displacement -= 1.0
            if displacement < 0:
                displacement = 0

        inputvalue.value = [displacement]

        # Print the applied tension, displacement, and new position of the robot
        self.printDebugInfo(displacement)

    def printDebugInfo(self, displacement):
        # Retrieve the new position of the robot and convert it to a list
        positions = self.mechanicalObject.findData('position').value
        min_position = list(positions[self.minPosIndex])
        print(f"Tension applied: {displacement}")
        print(f"New displacement: {displacement}")
        print(f"New position of the point with min x-coordinate: {min_position}")
