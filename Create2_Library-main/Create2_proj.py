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

# Custom constants
DISTANCE_DRIVE_POLLING = 0.1   # Distance driving polling wait in seconds
LIGHTS_INTERVAL = 2            # LED toggle timer interval in seconds
SAFE_DRIVE_INTERVAL = 0.1      # Safe driving timer interval in seconds
LIGHT_BUMPER_THRESHOLD = 150   # Minimum threshold for light bumper detection
SAFE_DRIVE_VELOCITY = 57       # Drive in steps of 28.5

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")


# ---------- Helper for Dropdown Connection ----------
class PortSelectDialog(simpledialog.Dialog):
    def __init__(self, parent, title, ports):
        self.ports = ports
        self.selected_port = None
        super().__init__(parent, title)

    def body(self, master):
        tk.Label(master, text="Select a COM port:").pack(padx=10, pady=5)

        self.var = tk.StringVar(master)
        self.var.set(self.ports[0])  # default to first port

        self.dropdown = tk.OptionMenu(master, self.var, *self.ports)
        self.dropdown.pack(padx=10, pady=5)
        return self.dropdown  # initial focus

    def apply(self):
        self.selected_port = self.var.get()

def require_robot(func):
    """Decorator to ensure robot is connected before executing a function."""
    @wraps(func)
    def wrapper(self, *args, **kwargs):
        if self.robot is None:
            messagebox.showinfo('Error', "Robot Not Connected!")
            return
        return func(self, *args, **kwargs)
    return wrapper

# ---------- Main App ----------
class TetheredDriveApp(tk.Tk):

    def __init__(self):
        super().__init__()
        self.title("iRobot Create 2 Tethered Drive")

        # Robot state
        self.robot = None
        self.velocity = 0
        self.rotation = 0

        # Bumper buffers for wall following
        self.bump_buffer = [[0] * 6 for _ in range(6)]  # 6 entries, each with 6 sensors
        self.buffer_index = 0

        # Light bumper circular buffer (if you still need this)
        self.light_bumper_buffer = [0] * 5

        # Polling
        self.sensor_poll_timer = None
        self.sensor_polling_active = False

        # Safe drive timer (initialize to None or your timer class if used)
        self.safe_drive_timer = None
        self.safe_drive_mode = 0

        # Light show
        self.lights = 0
        self.lights_timer = None  # Set this later if using CustomTimer

        # Setup UI
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

        # Use a horizontal frame to hold two text widgets
        frame = tk.Frame(self)
        frame.pack(fill=tk.BOTH, expand=True)

        # Left panel - Instructions
        self.instruction_text = tk.Text(frame, height=TEXTHEIGHT, width=int(TEXTWIDTH/2), wrap=tk.WORD)
        instruction_scroll = tk.Scrollbar(frame, command=self.instruction_text.yview)
        self.instruction_text.configure(yscrollcommand=instruction_scroll.set)

        # Right panel - Sensor live data
        self.sensor_text = tk.Text(frame, height=TEXTHEIGHT, width=int(TEXTWIDTH/2), wrap=tk.WORD)
        sensor_scroll = tk.Scrollbar(frame, command=self.sensor_text.yview)
        self.sensor_text.configure(yscrollcommand=sensor_scroll.set)

        # Layout
        self.instruction_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        instruction_scroll.pack(side=tk.LEFT, fill=tk.Y)
        self.sensor_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        sensor_scroll.pack(side=tk.LEFT, fill=tk.Y)

        # Set up instruction text initially
        self.instruction_text.insert(tk.END, self._help_text())
        self.instruction_text.config(state=tk.DISABLED)  # make read-only


    def _help_text(self):
        """Returns help text for key mappings."""
        supported_keys = {
            "P": "Passive Mode",
            "S": "Safe Mode",
            "F": "Full Mode",
            "C": "Clean",
            "D": "Dock",
            "R": "Reset",
            "B": "Sensor Dump",
            "SPACE": "Beep",
            "O": "Toggle Sensor Polling",
            "W": "Start Wall-Following Behavior",
            "K": "Start Docking Behavior",
            "L": "Display Bumps and Wheeldrops",
            "UP / DOWN / LEFT / RIGHT": "Manual Motion Control",
            "ESCAPE": "Quick Shutdown",
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
            "B": lambda: self._show_sensor_dump(),
            "O": self.toggle_sensor_polling,
            "UP": lambda: self._set_motion(velocity=VELOCITYCHANGE),
            "DOWN": lambda: self._set_motion(velocity=-VELOCITYCHANGE),
            "LEFT": lambda: self._set_motion(rotation=ROTATIONCHANGE),
            "RIGHT": lambda: self._set_motion(rotation=-ROTATIONCHANGE),
            "W": self.wall_follow_behavior,     # Starts wall-following behavior
            "K": self.docking_behavior,         # Starts docking behavior
            "L": self.display_bumps_wheeldrops, # Shows wheel drops and bumps
            "ESCAPE": self.on_quit
        }

        if key in key_mapping:
            key_mapping[key]()

    @require_robot
    def _show_sensor_dump(self):
        """Displays a full sensor dump in the sensor_text Tkinter box."""
        sensors = self.robot.get_sensors()
        sensor_text = self._format_sensor_data(sensors)

        self.sensor_text.config(state=tk.NORMAL)
        self.sensor_text.delete("1.0", tk.END)
        self.sensor_text.insert(tk.END, sensor_text)
        self.sensor_text.config(state=tk.DISABLED)



    def handle_keyrelease(self, event):
        """Handles key release events to stop movement."""
        key = event.keysym.upper()
        if key in {"UP", "DOWN"}:
            self._set_motion(velocity=0)
        elif key in {"LEFT", "RIGHT"}:
            self._set_motion(rotation=0)

    @require_robot
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

            if not ports:
                messagebox.showerror('No Devices', 'No serial devices found. Please check your connection.')
                return

            if len(ports) == 1:
                port = ports[0]
                logging.info(f"Only one port found: {port}. Connecting automatically.")
            else:
                # Use dropdown
                dialog = PortSelectDialog(self, "Select Port", ports)
                if dialog.selected_port:
                    port = dialog.selected_port
                else:
                    logging.warning('Connection cancelled by user.')
                    return

            self.robot = cl.Create2(port=port, baud=115200)
            messagebox.showinfo('Connected', f"Connected successfully to {port}!")
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

    # ----- Sensor Getters --------
    def get_sensors(self):
        """ Returns all sensor values as a namedtuple."""
        return self.robot.get_sensors()

    @require_robot
    def _beep_song(self):
        beep_song = [64, 16]
        self.robot.createSong(3, beep_song)
        self.robot.playSong(3)

    @require_robot
    def toggle_sensor_polling(self):
        """
        Toggles automatic sensor polling on/off.
        """
        if self.sensor_polling_active:
            if self.sensor_poll_timer:
                self.sensor_poll_timer.stop()
                self.sensor_poll_timer = None
            self.sensor_polling_active = False
            logging.info("Sensor polling stopped.")
        else:
            from createlib.custom_timer import CustomTimer
            self.sensor_poll_timer = CustomTimer(0.5, self.poll_sensors, False, repeat=True)
            self.sensor_polling_active = True
            logging.info("Sensor polling started.")

    @require_robot
    def poll_sensors(self, show_log=False):
        """
        Called automatically to poll and display sensor data.
        """
        sensors = self.robot.get_sensors()
        sensor_text = self._format_sensor_data(sensors)

        if show_log:
            logging.info(sensor_text)

        current_text = self.sensor_text.get("1.0", tk.END).strip()

        if current_text != sensor_text:
            self.sensor_text.config(state=tk.NORMAL)
            self.sensor_text.delete("1.0", tk.END)
            self.sensor_text.insert(tk.END, sensor_text)
            self.sensor_text.config(state=tk.DISABLED)



     # ----------------------- Our functions ------------------------------


    def get_light_bumper(self, sensors):
        """Returns the high resolution light bumper values as a list."""
        return [sensors.light_bumper_left, 
                sensors.light_bumper_front_left, 
                sensors.light_bumper_center_left,
                sensors.light_bumper_center_right, 
                sensors.light_bumper_front_right, 
                sensors.light_bumper_right]

    # ----- Tkinter Display Sensor Values --------
    def _decode_charger_state(self, code):
        """Returns the associated charging state string based on the given value (0-5)."""
        return {state.value: state.name.replace("_", " ").title() for state in CHARGING_STATE}[code]
    
    def display_bumps_wheeldrops(self):
        """Displays the Bumps and Wheeldrops sensor values in a Tkinter window."""
        sensors = self.get_sensors().bumps_wheeldrops  # Get Bumps and Wheeldrops namedtuple

        # Build formatted string of Bumps and Wheeldrops values
        info_string = (
            f'\nLeft Wheel: {"Dropped" if sensors.wheeldrop_left else "Raised"}'
            f'\nRight Wheel: {"Dropped" if sensors.wheeldrop_right else "Raised"}'
            f'\nLeft Bump: {"Bump" if sensors.bump_left else "No Bump"}'
            f'\nRight Bump: {"Bump" if sensors.bump_right else "No Bump"}'
        )

        # Display formatted string in Tkinter window
        messagebox.showinfo("Bumps and Wheeldrops", info_string)
    
    def display_internal_info(self):
        """Displays internal information related sensor values in a Tkinter window."""
        sensors = self.get_sensors()  # Get all sensor values

        # Build formatted string of Packet ID #3 values with proper units
        info_string = (
            f'\nCharger State: {self._decode_charger_state(sensors.charger_state)}'
            f'\nVoltage: {sensors.voltage} mV'
            f'\nCurrent: {sensors.current} mA'
            f'\nTemperature: {sensors.temperature} degrees Celsius'
            f'\nBattery Charge: {sensors.battery_charge} mAh'
            f'\nBattery Capacity: {sensors.battery_capacity} mAh'
        )

        # Display formatted string in Tkinter window
        messagebox.showinfo("Interal Information", info_string)
    
    def display_wall_cliffs(self):
        """Displays Wall and Cliffs sensor values in a Tkinter window."""
        sensors = self.get_sensors()  # Get Bumps and Wheeldrops namedtuple
        
        # Build formatted string of Wall and Cliff signal sensor values
        info_string = (
            f'\nWall Signal: {sensors.light_bumper_right}'
            f'\nCliff Left Signal: {sensors.cliff_left_signal}'
            f'\nCliff Front Left Signal: {sensors.cliff_front_left_signal}'
            f'\nCliff Front Right Signal: {sensors.cliff_front_right_signal}'
            f'\nCliff Right Signal: {sensors.cliff_right_signal}'
        )

        # Display formatted string in Tkinter window
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
    def drive_forward(self, velocity=SAFE_DRIVE_VELOCITY):
        """Drive forward with the specified or default velocity."""
        self._set_motion(velocity=velocity)
    
    def drive_stop(self):
        """Stop driving."""
        self._set_motion(velocity=0)

    def obstacle_detected(self, sensors, mode=0):
        """Returns whether or not there is an obstacle detected using the given sensors and detection mode.
        
            Args:
                sensors: All or relevant Create2 sensor values.
                mode: Obstacle detection mode. Uses one or more group of sensors to determine obstacles in the way.
                    - 0: Bumps and Wheeldrops AND Light Bumper (over threshold value)
                    - 1: Bumps and Wheeldrops
                    - 2: Light Bumper (over threshold value)
        """
        return {
            0: any(sensors.bumps_wheeldrops) or any(value >= LIGHT_BUMPER_THRESHOLD for value in self.get_light_bumper(sensors)),
            1: any(sensors.bumps_wheeldrops),
            2: any(value >= LIGHT_BUMPER_THRESHOLD for value in self.get_light_bumper(sensors))
        }[mode]

    def toggle_safe_drive(self, mode):
        """Starts the periodic timer for the safe driving if stopped, or stops/pauses it if started."""  
        if self.safe_drive_timer._stopped:
            # Update the safe driving mode and then start
            self.safe_drive_mode = mode
            self._start_safe_drive()
        else:
            # Stop/pause the safe driving mode
            self._stop_safe_drive()
    
    def safe_drive(self):
        """Drives forward unless/until bump or wheeldrop OR light bumper detected."""
        # Stop driving if obstacle detected using selected obstacle detection mode, or continue driving forward
        if self.obstacle_detected(self.get_sensors(), mode=self.safe_drive_mode): 
            self._stop_safe_drive()
    
    def _start_safe_drive(self):
        """Starts safe driving and safe drive timer"""
        self.safe_drive_timer.start()
        self.drive_forward()
    
    def _stop_safe_drive(self):
        """Stops safe driving and safe drive timer"""
        self.safe_drive_timer.stop()
        self.drive_stop()

    def toggle_drive_distance(self):
        """Gets user input for and calls driveDistance function if valid."""
        # Get distance from user using Tkinter
        result = simpledialog.askstring("Distance Drive Velocity", f"Enter distance (mm) to drive at {SAFE_DRIVE_VELOCITY} (mm/s): ")
        
        try:
            # Try to cast as float
            distance = float(result)

            # Start drive distance
            self.driveDistance(SAFE_DRIVE_VELOCITY, distance)
        except ValueError as e:
            # Display error when input is invalid, either from incorrect value types or format
            messagebox.showerror("Invalid Input", 'Enter distance (mm) to drive as a float value.'
                '\n\nExample: To drive 500 mm, enter "500" (without the quotation marks).')

    def driveDistance(self, velocity, distance):
        """Drives the specified velocity until the given distance is reached or an obstacle is detected.
        
            Args:
                velocity (float): Driving velocity in mm/sec (steps of about 28.5 mm/s.)
                distance (float): Driving distance in mm.
        """
        # Reset distance traveled by querying for distance sensor
        distance_traveled = 0.0 * self.get_sensors().distance
        time.sleep(DISTANCE_DRIVE_POLLING)

        # Start driving using the given velocity
        logging.info("Starting drive distance.")
        self.drive_forward(velocity=velocity)
        
        # Drive while distance has not been reached
        while distance_traveled < distance:
            # Get updated sensor data
            sensors = self.get_sensors()

            # Get distance traveled since last queried
            distance_traveled += abs(sensors.distance)

            # Stop driving if obstacle encountered
            if self.obstacle_detected(sensors):
                self.drive_stop()
                logging.info(f'Obstacle detected--stopping drive.')
                break

            # Give time for robot to move between readings
            time.sleep(DISTANCE_DRIVE_POLLING)
    
        # Stop driving and display distance driven
        self.drive_stop()
        messagebox.showinfo("Distance Driven", f'\nTotal Distance Traveled: {distance_traveled:.2f} mm')  



    @require_robot
    def wall_follow_behavior(self):
        """
        Initiates wall-following behavior using circular array for recent bumper readings.
        """


    @require_robot
    def wall_follow_behavior(self):
        """
        Initiates wall-following behavior using circular array for recent bumper readings.
        If bump is detected, robot rotates 45 degrees repeatedly until no bump is detected.
        """
        self.drive_until_wall()
        logging.info("Wall-follow PID control starting...")

        try:
            while True:
                sensors = self.get_sensors()
                bump_values = self.get_light_bumper(sensors)

                # Update circular buffer with new reading
                self.bump_buffer[self.buffer_index] = bump_values
                self.buffer_index = (self.buffer_index + 1) % 6

                # Compute average signal strength across the buffer
                avg_signal = [sum(x[i] for x in self.bump_buffer) / 6 for i in range(6)]

                # Lower threshold for wall-following sensitivity
                threshold = 10

                # Handle bump detection (rotate 45 degrees repeatedly until clear)
                while sensors.bumps_wheeldrops.bump_left or sensors.bumps_wheeldrops.bump_right:
                    logging.info("Bump detected. Rotating 45 degrees...")

                    self.drive_stop()
                    time.sleep(0.2)

                    # Rotate 45 degrees: 100 ms * 6 ~= 45 deg for Create 2
                    self.robot.drive_direct(100, -100)  # Rotate clockwise
                    time.sleep(0.6)
                    self.drive_stop()
                    time.sleep(0.2)

                    # Recheck sensors
                    sensors = self.get_sensors()

                # Continue wall-following
                if avg_signal[2] > threshold:  # Strong signal on center left
                    self.robot.drive_direct(30, 60)
                elif avg_signal[3] > threshold:  # Strong signal on center right
                    self.robot.drive_direct(60, 30)
                else:
                    self.robot.drive_direct(50, 50)

                time.sleep(0.1)

        except KeyboardInterrupt:
            self.drive_stop()
            logging.info("Wall-following behavior interrupted.")



    @require_robot
    def drive_until_wall(self, velocity=150):
        """
        Drives forward until a wall is detected via light bumpers.
        Uses circular buffer logic to smooth detection.
        """
        logging.info("Driving forward until wall is detected (non-collision)...")
        self.drive_forward(velocity)

        bump_buffer = [0] * 6
        buffer_index = 0

        try:
            while True:
                sensors = self.get_sensors()
                bump_values = self.get_light_bumper(sensors)

                # Store center left and right values into circular buffer
                center_value = max(bump_values[2], bump_values[3])
                bump_buffer[buffer_index] = center_value
                buffer_index = (buffer_index + 1) % 6

                # Average over buffer to determine if wall is near
                avg_signal = sum(bump_buffer) / len(bump_buffer)
                if avg_signal >= LIGHT_BUMPER_THRESHOLD:
                    logging.info("Wall detected via smoothed light bumpers.")
                    break

                time.sleep(DISTANCE_DRIVE_POLLING)

        finally:
            self.drive_stop()
            logging.info("Stopped before wall.")


    @require_robot
    def docking_behavior(self):
        # TODO: Implement docking logic here
        pass


# ----------------------- Main Driver ------------------------------
if __name__ == "__main__":
    banner = """
==================================================
   iRobot Create 2 Tethered Drive Interface
==================================================
Press "Connect" from the menu to begin.
Use the keyboard to drive the robot once connected!
Press "O" to toggle live sensor monitoring.
==================================================
    """
    print(banner)
    
    app = TetheredDriveApp()
    app.mainloop()
