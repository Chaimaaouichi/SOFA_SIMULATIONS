#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
from Sofa.constants import Key
import time

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output
class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.node = kw["node"]
        self.mechanicalObject = self.node.getObject('MechanicalObject')
        self.minPosIndex = None
        self.findMinXPosition()
        self.setpoint = -20  # Desired x position of the top point (example value)
        self.pid = PIDController(kp=1.0, ki=0.1, kd=0.01, setpoint=self.setpoint)
        self.prev_time = time.time()

    def findMinXPosition(self):
        # Find the position with the minimal x coordinate
        positions = self.mechanicalObject.findData('position').value
        min_x = float('inf')
        for i, pos in enumerate(positions):
            if pos[0] < min_x:
                self.minPosIndex = i
                min_x = pos[0]
        print(f"Index of min x position: {self.minPosIndex}")
        print(f"Min x position: {positions[self.minPosIndex]}")

    def onAnimateBeginEvent(self, e):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Get the current x position of the point with the minimal x coordinate
        current_x_position = self.mechanicalObject.findData('position').value[self.minPosIndex][0]

        # Update the PID controller
        tension = self.pid.update(current_x_position, dt)

        # Apply the calculated tension
        self.node.aCableActuator.value = [max(0, tension)]  # Ensure tension is non-negative

        # Print the debug info
        self.printDebugInfo(tension, current_x_position)

    def printDebugInfo(self, tension, current_x_position):
        print(f"Desired x position: {self.setpoint}")
        print(f"Current x position: {current_x_position}")
        print(f"Tension applied: {tension}")
