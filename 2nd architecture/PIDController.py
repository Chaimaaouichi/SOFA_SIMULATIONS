#!/usr/bin/env python
# -*- coding: utf-8 -*-

import Sofa.Core
from Sofa.constants import Key
import time
import math
import os
from PIL import Image
from PIL import ImageGrab
from openpyxl import load_workbook
def capture_screenshot(filename):
    # Capture the entire screen
    screenshot = ImageGrab.grab(bbox=(1000, 550, 1230, 700))
    # Save the screenshot to a file
    screenshot.save(filename)
    # Close the screenshot
    screenshot.close()
class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0
        self.max_tension =180

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        output=min(output,self.max_tension)
        output=max(0,output)
        
        return output

class FingerController(Sofa.Core.Controller):
    def __init__(self, *a, **kw):
        super().__init__(*a, **kw)
        self.node = kw["node"]
        self.mechanicalObject = self.node.getObject('MechanicalObject')
        self.minPosIndex = None
        self.findMinXPosition()
        '''self.setpoint =-95'''
        self.setpoint = float(os.getenv('TARGET_POSITION', '-60')) # Desired x position of the top point (example value)
        self.pid = PIDController(kp=1, ki=0.1, kd=0.001, setpoint=self.setpoint)
        self.prev_time = time.time()
        self.tensions = []  # List to store tension values
        self.time_stamps = []  # List to store time stamp
        self.positions = []  # List to store positions
        self.filepath = "/home/rmal/backboneSofa/tension_values.txt"
        self.position_filepath = "/home/rmal/backboneSofa/positions.txt"
        self.steady_state_threshold = 0.6 # Define your steady-state error threshold
        self.stability_window = 20  # Number of samples to consider for stability
        self.prev_outputs = []
        # Clear the content of the files before starting the simulation
        self.clear_files()
        self.step_counter = 0
        # Path to the Excel file
        self.excel_path = "/home/rmal/FinalFinal/simulation_data.xlsx"

    def clear_files(self):
        open(self.filepath, 'w').close()
        open(self.position_filepath, 'w').close()

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
        if self.step_counter == 0:
            print("Simulation start")
            '''capture_screenshot("beginning_screenshot.png")'''
            self.step_counter += 1

        # Get the current x position of the point with the minimal x coordinate
        current_x_position = self.mechanicalObject.findData('position').value[self.minPosIndex][0]

        # Update the PID controller
        tension = self.pid.update(current_x_position, dt)

        self.tensions.append(tension)
        self.time_stamps.append(current_time - self.time_stamps[0] if self.time_stamps else 0)
        self.positions.append(current_x_position)

        # Save the tension and time to a file
        with open(self.filepath, "a") as file:
            file.write(f"{self.time_stamps[-1]},{tension}\n")

        # Save the position and time to a separate file
        with open(self.position_filepath, "a") as pos_file:
            pos_file.write(f"{self.time_stamps[-1]},{current_x_position}\n")

        # Apply the calculated tension
        self.node.aCableActuator.value = [max(0, tension)]  # Ensure tension is non-negative

        # Print the debug info
        self.printDebugInfo(tension, current_x_position)

        # Check if the error is within the steady-state threshold and the system is stable
        error = self.setpoint - current_x_position
        is_close_to_setpoint = math.isclose(error, 0.0, abs_tol=self.steady_state_threshold)
        is_stable = self.checkStability(tension)

        if is_close_to_setpoint and is_stable:
            self.stopSimulation()

    def checkStability(self, current_output):
        # Maintain the last 'stability_window' outputs to check for stability
        self.prev_outputs.append(current_output)
        if len(self.prev_outputs) > self.stability_window:
            self.prev_outputs.pop(0)
        # Check if the variation in outputs is within a small range, indicating stability
        output_variation = max(self.prev_outputs) - min(self.prev_outputs)
        stable = output_variation < 0.05 # Adjust the threshold as needed
        return stable

    def printDebugInfo(self, tension, current_x_position):
        print(f"Desired x position: {self.setpoint}")
        print(f"Current x position: {current_x_position}")
        print(f"Tension applied: {tension}")

    def stopSimulation(self):
        self.node.getRootContext().animate = False
        print("Simulation stopped: PID controller reached steady-state error and the system is stable.")

        # Take a screenshot at the end of the simulation
        screenshot_file = f"screenshot_{int(time.time())}.png"
        capture_screenshot(screenshot_file)

        # Register data to the Excel data list
        initial_position = self.positions[0] if self.positions else None
        last_tension = self.tensions[-1] if self.tensions else None
        last_position = self.positions[-1] if self.positions else None
        new_data = {
            'time': self.time_stamps[-1], 
            'target_position': self.setpoint, 
            'initial_position': initial_position, 
            'last_tension': last_tension,
            'last_position': last_position,
            'screenshot_file': screenshot_file
        }

        # Append the new data to the existing Excel file
        if os.path.exists(self.excel_path):
            workbook = load_workbook(self.excel_path)
            sheet = workbook.active
            # Append the new data to the existing sheet
            sheet.append([new_data['time'], new_data['target_position'], new_data['initial_position'], new_data['last_tension'],new_data['last_position'],new_data['screenshot_file']])
            workbook.save(self.excel_path)
        else:
            # If the file does not exist, create it with the new data
            df = pd.DataFrame([new_data])
            df.to_excel(self.excel_path, index=False, engine='openpyxl')

        # Notify SOFA that the simulation is done
        self.node.getRootContext().findData('animate').value = False
        print("Simulation has been properly stopped and data has been saved.")
