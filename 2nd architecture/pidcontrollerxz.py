#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
from Sofa.constants import Key
import time
import math
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint  # setpoint is now a tuple (x_setpoint, z_setpoint)
        self.prev_error = (0, 0)
        self.integral = (0, 0)
        self.max_tension = 50

    def update(self, current_value, dt):
        error_x = self.setpoint[0] - current_value[0]
        error_z = self.setpoint[1] - current_value[1]
        
        self.integral = (self.integral[0] + error_x * dt, self.integral[1] + error_z * dt)
        derivative_x = (error_x - self.prev_error[0]) / dt
        derivative_z = (error_z - self.prev_error[1]) / dt
        
        output_x = self.kp * error_x + self.ki * self.integral[0] + self.kd * derivative_x
        output_z = self.kp * error_z + self.ki * self.integral[1] + self.kd * derivative_z
        
        self.prev_error = (error_x, error_z)
        
        # Combine the x and z tensions
        combined_tension = math.sqrt(output_x**2 + output_z**2)
        combined_tension = min(combined_tension, self.max_tension)
        
        return combined_tension

class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.node = kw["node"]
        self.mechanicalObject = self.node.getObject('MechanicalObject')
        '''self.target_position = (-120.071, 0, 0)'''
        self.minPosIndex =7
        '''self.findClosestPosition()'''
        
        self.setpoint = (-70, 7.5)  # Desired (x, z) position of the target point
        self.pid = PIDController(kp=1, ki=0.2, kd=0.01, setpoint=self.setpoint)
        self.prev_time = time.time()
        self.tensions = []  # List to store tension values
        self.time_stamps = []  # List to store time stamp
        self.positions = []  # List to store positions
        self.filepath = "/home/rmal/backboneSofa/tension_values.txt"
        self.position_filepath = "/home/rmal/backboneSofa/positions.txt"
        self.steady_state_threshold = 1  # Define your steady-state error threshold
        self.stability_window = 20  # Number of samples to consider for stability
        self.prev_outputs = []
        # Clear the content of the files before starting the simulation
        self.clear_files()

    def clear_files(self):
        open(self.filepath, 'w').close()
        open(self.position_filepath, 'w').close()

    

    def onAnimateBeginEvent(self, e):
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # Get the current (x, z) position of the point with the minimal x coordinate
        current_position = self.mechanicalObject.findData('position').value[self.minPosIndex]
        current_x_position = current_position[0]
        current_z_position = current_position[2]

        # Update the PID controller
        combined_tension = self.pid.update((current_x_position, current_z_position), dt)

        self.tensions.append(combined_tension)
        self.time_stamps.append(current_time - self.time_stamps[0] if self.time_stamps else 0)
        self.positions.append((current_x_position, current_z_position))

        # Save the tension and time to a file
        with open(self.filepath, "a") as file:
            file.write(f"{self.time_stamps[-1]},{combined_tension}\n")

        # Save the position and time to a separate file
        with open(self.position_filepath, "a") as pos_file:
            pos_file.write(f"{self.time_stamps[-1]},{current_x_position},{current_z_position}\n")

        # Apply the calculated combined tension
        self.node.aCableActuator.value = [combined_tension]  # Apply combined tension

        # Print the debug info
        self.printDebugInfo(combined_tension, current_x_position, current_z_position)

        # Check if the error is within the steady-state threshold and the system is stable
        error_x = self.setpoint[0] - current_x_position
        error_z = self.setpoint[1] - current_z_position
        is_close_to_setpoint = math.isclose(error_x, 0.0, abs_tol=self.steady_state_threshold) and math.isclose(error_z, 0.0, abs_tol=self.steady_state_threshold)
        is_stable = self.checkStability(combined_tension)

        if is_close_to_setpoint and is_stable:
            self.stopSimulation()
        
    def checkStability(self, current_output):
        # Maintain the last 'stability_window' outputs to check for stability
        self.prev_outputs.append(current_output)
        if len(self.prev_outputs) > self.stability_window:
            self.prev_outputs.pop(0)
        # Check if the variation in outputs is within a small range, indicating stability
        output_variation = max(self.prev_outputs) - min(self.prev_outputs)
        stable = output_variation < 0.1  # Adjust the threshold as needed
        return stable

    def printDebugInfo(self, combined_tension, current_x_position, current_z_position):
        print(f"Desired position: {self.setpoint}")
        print(f"Current position: ({current_x_position}, {current_z_position})")
        print(f"Combined tension applied: {combined_tension}")

    def stopSimulation(self):
        self.node.getRootContext().animate = False
        print("Simulation stopped: PID controller reached steady-state error and the system is stable.")
