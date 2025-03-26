#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Version History
# v1.0: Python2.7 -- 2015//05/27
# v2.0: Update to Python3 -- 2020/04/01
# v2.1: Stiffler (bare) modifications -- 2022/02/02
# v3.0: Stiffler Quality of Life changes
# v3.1: Stiffler Bug Fixes and QoL - 2025/02/18

###########################################################################
# Copyright (c) 2015-2020 iRobot Corporation#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#   Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
#   Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in
#   the documentation and/or other materials provided with the
#   distribution.
#
#   Neither the name of iRobot Corporation nor the names
#   of its contributors may be used to endorse or promote products
#   derived from this software without specific prior written
#   permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
###########################################################################

import logging
import struct
import sys
import glob
import time
import tkinter as tk
from tkinter import messagebox, simpledialog
from functools import wraps

# Create Library
import createlib as cl
from createlib.create_oi import CHARGING_STATE, ROBOT

try:
    import serial
except ImportError:
    messagebox.showerror('Import error', 'Please install pyserial using: pip install pyserial')
    raise

# Constants
TEXTWIDTH = 100
TEXTHEIGHT = 24
VELOCITYCHANGE = 200
ROTATIONCHANGE = 300
DOCK_TIMEOUT = 30  # Timeout for docking in seconds
LIGHTS_INTERVAL = 2
SAFE_DRIVE_INTERVAL = 0.25

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

def require_robot(func):
    """Decorator to ensure robot is connected before executing a function."""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.robot is None:
            messagebox.showinfo('Error', "Robot Not Connected!")
            return
        return func(self, *args, **kwargs)
    return wrapper

class TetheredDriveApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("iRobot Create 2 Tethered Drive")

        self.robot = None
        self.velocity = 0
        self.rotation = 0
        
        # custom variables
        self.lights = 1
        self.safe_drive_mode = 1

        self._setup_ui()
        self.bind("<KeyPress>", self.handle_keypress)
        self.bind("<KeyRelease>", self.handle_keyrelease)

    def _setup_ui(self):
        """Sets up UI elements like the menu and text output."""
        self.option_add('*tearOff', False)

        menubar = tk.Menu(self)
        self.configure(menu=menubar)

        create_menu = tk.Menu(menubar, tearoff=False)
        menubar.add_cascade(label="Create", menu=create_menu)
        create_menu.add_command(label="Connect", command=self.on_connect)
        create_menu.add_command(label="Help", command=self.on_help)
        create_menu.add_command(label="Quit", command=self.on_quit)

        self.text = tk.Text(self, height=TEXTHEIGHT, width=TEXTWIDTH, wrap=tk.WORD)
        scroll = tk.Scrollbar(self, command=self.text.yview)
        self.text.configure(yscrollcommand=scroll.set)
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.text.insert(tk.END, self._help_text())

    def _help_text(self):
        """Returns help text for key mappings."""
        supported_keys = {
            "P": "Passive Mode",
            "S": "Safe Mode",
            "F": "Full Mode",
            "C": "Clean",
            "D": "Dock",
            "R": "Reset",
            "A": "Sensor Dump",
            "B": "Bumps and Wheeldrops",
            "I": "Internal Info",
            "W": "Wall and Cliff Sensors",
            "L": "Light Show",
            "Z": "Safe Driving -- Bumps and Wheel Drops",
            "X": "Safe Driving -- Light Bumper",
            "V": "Distance Driving",
            "Space": "Beep",
            "Arrows": "Motion",
            "Escape": "Quick Shutdown",
        }
        return "\n".join([f"{key}: {desc}" for key, desc in supported_keys.items()]) + \
               "\n\nIf nothing happens after you connect, try pressing 'P' and then 'S' to enter safe mode."

    def handle_keypress(self, event):
        """Handles keypress events."""
        key = event.keysym.upper()
        key_mapping = {
            "P": lambda: self.robot.start(),
            "S": lambda: self.robot.safe(),
            "F": lambda: self.robot.full(),
            "C": lambda: self.robot.clean(),
            "R": lambda: self.robot.reset(),
            "SPACE": lambda: self._beep_song(),
            "A": lambda: logging.info(self._format_sensor_data(self.robot.get_sensors())),
            "B": lambda: self.display_bumps_wheeldrops(),
            "I": lambda: self.display_internal_info(),
            "W": lambda: self.display_wall_cliffs(),
            "L": lambda: self.toggle_lights(),
            "Z": lambda: self.toggle_safe_drive(mode=1),
            "X": lambda: self.toggle_safe_drive(mode=2),
            "V": lambda: self.toggle_drive_distance(),
            "UP": lambda: self._set_motion(velocity=VELOCITYCHANGE),
            "DOWN": lambda: self._set_motion(velocity=-VELOCITYCHANGE),
            "LEFT": lambda: self._set_motion(rotation=ROTATIONCHANGE),
            "RIGHT": lambda: self._set_motion(rotation=-ROTATIONCHANGE),
            "ESCAPE": self.on_quit
        }
        if key in key_mapping:
            key_mapping[key]()

    def handle_keyrelease(self, event):
        """Handles key release events to stop movement."""
        key = event.keysym.upper()
        if key in {"UP", "DOWN"}:
            self._set_motion(velocity=0)
        elif key in {"LEFT", "RIGHT"}:
            self._set_motion(rotation=0)

    def _set_motion(self, velocity=None, rotation=None):
        """
        Updates motion values and sends drive command.
        """
        if velocity is not None:
            self.velocity = velocity
        if rotation is not None:
            self.rotation = rotation

        vr = int(self.velocity + (self.rotation / 2))
        vl = int(self.velocity - (self.rotation / 2))
        if self.robot:
            self.robot.drive_direct(vl, vr)

    def _format_sensor_data(self, sensors):
        """
        Formats sensor data for logging.
        """
        return "\n".join([f"{key}: {value}" for key, value in sensors._asdict().items()])


    def on_connect(self):
        """
        Handles connection to the robot.
        """
        if self.robot:
            messagebox.showinfo('Oops', "You're already connected to the robot!")
            return

        try:
            ports = self._get_serial_ports()
            port = simpledialog.askstring('Port?', 'Enter COM port to open.\nAvailable options:\n' + '\n'.join(ports))
            if port:
                self.robot = cl.Create2(port=port, baud=115200)
                # Initalize but do not start thread timers
                self.lights_timer = cl.RepeatTimer(interval=LIGHTS_INTERVAL, function=self.lightshow, autostart=False)
                self.safe_drive_timer = cl.RepeatTimer(interval=SAFE_DRIVE_INTERVAL, function=self.safe_drive, autostart=False)
                messagebox.showinfo('Connected', "Connection succeeded!")
                logging.info(f"Connected to robot on {port}")
        except Exception as e:
            logging.error(f"Failed to connect: {e}")
            messagebox.showerror('Connection Failed', f"Couldn't connect to {port}")

    def on_help(self):
        """
        Displays help text.
        """
        messagebox.showinfo('Help', self._help_text())

    def on_quit(self):
        """
        Handles quitting the application.
        """
        if messagebox.askyesno('Really?', 'Are you sure you want to quit?'):
            if self.robot:
                del self.robot
            self.destroy()

    def _get_serial_ports(self):
        """
        Lists serial ports
        From http://stackoverflow.com/questions/12090503/listing-available-com-ports-with-python

        :raises EnvironmentError:
            On unsupported or unknown platforms
        :returns:
            A list of available serial ports
        """
        if sys.platform.startswith('win'):
            ports = ['COM' + str(i + 1) for i in range(256)]

        elif sys.platform.startswith('linux') or sys.platform.startswith('cygwin'):
            # this is to exclude your current terminal "/dev/tty"
            ports = glob.glob('/dev/tty[A-Za-z]*')

        elif sys.platform.startswith('darwin'):
            ports = glob.glob('/dev/tty.*')

        else:
            raise EnvironmentError('Unsupported platform')

        result = []
        for port in ports:
            try:
                s = serial.Serial(port)
                s.close()
                result.append(port)
            except (OSError, serial.SerialException):
                pass
        return result


    # ----------------------- Custom functions ------------------------------
    def _beep_song(self):
        beep_song = [64, 16]
        self.robot.createSong(3, beep_song)
        self.robot.playSong(3)
    
    def get_sensors(self):
        """ Returns all sensor values as a namedtuple directly."""
        return self.robot.get_sensors()

    def get_light_bumper(self, threshold=50):
        """Returns the high resolution light bumper values as a list. Ignores values below the threshold."""
        sensors = self.get_sensors()
        
        # Retrieve all light bump sensors values
        light_bumper_values = [
            sensors.light_bumper_left, 
            sensors.light_bumper_front_left, 
            sensors.light_bumper_center_left,
            sensors.light_bumper_center_right, 
            sensors.light_bumper_front_right, 
            sensors.light_bumper_right
        ]
        
        # Apply threshold filtering: Any value below the threshold is treated as 0
        filtered_values = [value if value > threshold else 0 for value in light_bumper_values]
        
        return filtered_values

    # ----- Tkinter Display Sensor Values --------
    def _decode_charger_state(self, code):
        """Returns the associated charging state string based on the given value (0-5)."""
        return {state.value: state.name.replace("_", " ").title() for state in CHARGING_STATE}[code]
    
    def display_bumps_wheeldrops(self):
        """Displays the Bumps and Wheeldrops sensor values in a Tkinter window."""
        sensors = self.get_sensors("bumps_wheeldrops")
        info_string = (
            f'\nLeft Wheel: {"Dropped" if sensors.wheeldrop_left else "Raised"}'
            f'\nRight Wheel: {"Dropped" if sensors.wheeldrop_right else "Raised"}'
            f'\nLeft Bump: {"Bump" if sensors.bump_left else "No Bump"}'
            f'\nRight Bump: {"Bump" if sensors.bump_right else "No Bump"}'
        )
        messagebox.showinfo("Bumps and Wheeldrops", info_string)
    
    def display_internal_info(self):
        """Displays internal information related sensor values in a Tkinter window."""
        sensors = self.get_sensors()
        info_string = (
            f'\nCharger State: {self._decode_charger_state(sensors.charger_state)}'
            f'\nVoltage: {sensors.voltage} mV'
            f'\nCurrent: {sensors.current} mA'
            f'\nTemperature: {sensors.temperature} degrees Celsius'
            f'\nBattery Charge: {sensors.battery_charge} mAh'
            f'\nBattery Capacity: {sensors.battery_capacity} mAh'
        )

        messagebox.showinfo("Interal Information", info_string)
    
    def display_wall_cliffs(self):
        """Displays Wall and Cliffs sensor values in a Tkinter window."""
        sensors = self.get_sensors()
        info_string = (
            f'\nWall Signal: {sensors.light_bumper_right}'
            f'\nCliff Left Signal: {sensors.cliff_left_signal}'
            f'\nCliff Front Left Signal: {sensors.cliff_front_left_signal}'
            f'\nCliff Front Right Signal: {sensors.cliff_front_right_signal}'
            f'\nCliff Right Signal: {sensors.cliff_right_signal}'
        )

        messagebox.showinfo("Wall and Cliffs", info_string)

    # ----- Lights --------
    def toggle_lights(self):
        """Starts the periodic timer for the lights show if stopped, or stops/pauses it if started."""
        self.lights_timer.start() if self.lights_timer._stopped else self.lights_timer.stop()

    def lightshow(self):
        """ Turns on one of two lighting configurations based on the value of self.lights.
    
        Value of self.lights:
            - 1: Debris, Check Robot, and Power (RED, full intensity) 
            - 0: Clean and Dock
        """
        if self.lights:
            # Turn on the Debris and Check Robot LEDs, and set Power LED to full intensity RED
            self.robot.led(led_bits=9, power_color=255, power_intensity=255)
        else:
            # Turn on the Clean and Dock LEDs, and turn off the Power LED
            self.robot.led(led_bits=6, power_color=0, power_intensity=0)
        
        # Toggle/switch configurations
        self.lights ^= 1


    # ----- Driving --------
    def drive_forward(self, velocity=VELOCITYCHANGE):
        """Drive forward with the specified or default velocity."""
        self._set_motion(velocity=velocity)
    
    def drive_stop(self):
        """Stop driving."""
        self._set_motion(velocity=0)
    
    def get_distance(self, sensors):
        """Calculates and returns the distance traveled by the robot since the last read.

        Notes:
        - Uses the 'distance' attribute from the sensors, which reports the distance traveled in millimeters.
        - The value resets to zero each time it's read.
        """
        try:
            # Directly access the 'distance' attribute
            distance_traveled = getattr(sensors, 'distance', None)
            
            if distance_traveled is None:
                logging.error("Failed to retrieve distance data from sensors.")
                return 0.0

            # Log the distance reading for debugging
            logging.info(f"Distance received from sensors: {distance_traveled} mm")
            
            # Return the distance traveled
            return distance_traveled
        
        except AttributeError:
            logging.error("Distance attribute not found in sensor data.")
            return 0.0



    def obstacle_detected(self, sensors, mode=0):
        """Returns whether or not there is an obstacle detected using the given sensors and detection mode.
        
            Args:
                sensors: All or relevant Create2 sensor values.
                mode: Obstacle detection mode. Uses one or more group of sensors to determine obstacles in the way.
                    - 0: Bumps and Wheeldrops AND Light Bumper
                    - 1: Bumps and Wheeldrops
                    - 2: Light Bumper
        """
        detection = False
        triggered_sensors = []

        # Check bumps and wheeldrops
        bumps_wheeldrops = sensors.bumps_wheeldrops  # Accessing namedtuple attribute directly
        if bumps_wheeldrops.wheeldrop_left:
            triggered_sensors.append("Wheel Drop: Left Wheel")
        if bumps_wheeldrops.wheeldrop_right:
            triggered_sensors.append("Wheel Drop: Right Wheel")
        if bumps_wheeldrops.bump_left:
            triggered_sensors.append("Bump Detected: Left Bumper")
        if bumps_wheeldrops.bump_right:
            triggered_sensors.append("Bump Detected: Right Bumper")

        # Check light bumpers if mode 0 or 2
        if mode in (0, 2):
            light_bumper = self.get_light_bumper()
            if any(light_bumper):  # Checks if any light bumper is triggered (non-zero)
                detection = True
                if light_bumper[0] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Left")
                if light_bumper[1] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Front Left")
                if light_bumper[2] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Center Left")
                if light_bumper[3] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Center Right")
                if light_bumper[4] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Front Right")
                if light_bumper[5] > 0:
                    triggered_sensors.append("Light Bumper Triggered: Right")
        
        # Print triggered sensors if any
        if triggered_sensors:
            detection = True
            print(f"\n🔔 Obstacle Detected! Triggered Sensors:\n - " + "\n - ".join(triggered_sensors))

        if mode == 0:
            return detection
        elif mode == 1:
            return any([
                bumps_wheeldrops.wheeldrop_left,
                bumps_wheeldrops.wheeldrop_right,
                bumps_wheeldrops.bump_left,
                bumps_wheeldrops.bump_right
            ])
        elif mode == 2:
            return any(light_bumper)
        
        return False



    def toggle_safe_drive(self, mode):
        """Starts the periodic timer for the safe driving if stopped or mode change, or stops/pauses it if started."""  
        # Check if change in safe driving mode
        if self.safe_drive_mode != mode:
            # Stop safe driving to change modes
            self._stop_safe_drive()
            self.safe_drive_mode = mode
        
        # Start safe driving timer if stopped or stop/pause if started
        self._start_safe_drive() if self.safe_drive_timer._stopped else self._stop_safe_drive()
    
    def safe_drive(self):
        """Drives forward unless/until bump or wheeldrop OR light bumper detected."""
        sensors = self.get_sensors()
        
        # Stop driving if obstacle detected using selected obstacle detection mode, or continue driving forward
        if self.obstacle_detected(sensors, mode=self.safe_drive_mode): 
            self._stop_safe_drive()  

    
    def _start_safe_drive(self):
        """Starts driving and safe drive timer."""
        if self.safe_drive_timer._stopped:
            self.safe_drive_timer.start()
        self.drive_forward()

    
    def _stop_safe_drive(self):
        """Stops driving and safe drive timer."""
        if not self.safe_drive_timer._stopped:
            self.safe_drive_timer.stop()
        self.drive_stop()

    def toggle_drive_distance(self):
        """Gets user input for and calls drive_distance function if valid."""
        # Get velocity and distance from user using Tkinter
        result = simpledialog.askstring("Distance Drive Velocity", "Enter the driving velocity (mm/sec) and distance (mm) separated by a comma: ")
        
        try:
            # Split into velocity and distance and try to cast as float
            velocity, distance = str(result).split(",")
            velocity, distance = float(velocity), float(distance)

            # Start drive distance
            self.drive_distance(velocity, distance)
        except ValueError as e:
            # Display error when input is invalid, either from incorrect value types or format
            messagebox.showerror("Invalid Input", 'Enter velocity and distance as float values separated by a comma.'
                '\n\nExample: Drvie 10 mm/sec for 40 mm, enter "10, 40" (without the quotation marks).')

    def drive_distance(self, velocity, distance):
        """Drives the specified velocity until the given distance is reached or an obstacle is detected.
        
            Args:
                velocity (float): Driving velocity in mm/sec (steps of about 28.5 mm/s.)
                distance (float): Driving distance in mm.
        """
        # Reset distance traveled
        distance_traveled = 0.0  # Initialize distance traveled to zero

        # Start driving using the given velocity
        self.drive_forward(velocity=velocity)
        
        while distance_traveled <= distance:
            sensors = self.get_sensors()  # Continuously get updated sensor data
            
            # Update distance traveled
            increment = self.get_distance(sensors)
            
            # Log the distance increment received
            logging.info(f"Increment received from sensors: {increment:.2f} mm")

            # Only add increment if it's a valid reading (i.e., not zero or a random negative value)
            if abs(increment) > 0:
                distance_traveled += abs(increment)
            
            logging.info(f"Total Distance Traveled: {distance_traveled:.2f} mm")
            
            # Stop driving if an obstacle is encountered
            if self.obstacle_detected(sensors):
                logging.info("Obstacle detected. Stopping drive.")
                self.drive_stop()
                break
            
            # Give time for the robot to actually move between readings
            time.sleep(0.1)  # Adjust this delay as needed
        
        # Stop driving and display distance driven
        self.drive_stop()
        messagebox.showinfo("Distance Driven", f'\nTotal Distance Traveled: {distance_traveled:.2f} mm')  



    # ----------------------- Main Driver ------------------------------
if __name__ == "__main__":
    app = TetheredDriveApp()
    app.mainloop()