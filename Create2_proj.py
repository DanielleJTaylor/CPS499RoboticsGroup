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
import math
import struct
import sys
import glob
import time
import tkinter as tk
from tkinter import messagebox, simpledialog
from functools import wraps

# Create Library
import createlib as cl
from createlib.circular_array import CircularArray

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
BIG_TURN_THRESHOLD = 500    # Minimum error/control value needed to make a big turn
SMALL_TURN_THRESHOLD = 200  # Minimum error/control value needed to make a small turn

SET_POINT = 500             # Wall following set point for PID
WALL_THRESHOLD = 500        # Front/center wall light bumper detection threshold
VELOCITY_STEPS = 28.5       # Velocity controls in steps of 28.5 mm/s


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

        # required variables
        self.robot = None
        self.velocity = 0
        self.rotation = 0

        # TODO: custom variables
        self.wall_follow_timer = None
        self.wall_follow_active = False
        
        # Automatic polling
        self.sensor_poll_timer = None
        self.sensor_polling_active = False

        # GUI Elements
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
            "W": "Wall Following",
            "D": "Dock Robot",
            "Space": "Beep",
            "O": "Toggle Sensor Polling",
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
            "B": lambda: logging.info(self._format_sensor_data(self.robot.get_sensors())),
            "W": lambda: self.toggle_wall_follow(),
            "D": lambda: self.dock_robot(),
            "O": self.toggle_sensor_polling,
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
    

    @require_robot
    def toggle_wall_follow(self):
        """Toggles wall following on/off."""

        if self.wall_follow_timer:
            self.wall_follow_timer.stop()
            self.wall_follow_timer = None
            self.wall_follow_active = False
            logging.info("Wall following stopped.")
        else:
            from createlib.custom_timer import CustomTimer
            self.wall_follow_timer = CustomTimer(0.5, self.wall_following, autostart=True, repeat=False)
            self.wall_follow_active = True
            logging.info("Wall following started.")


    @require_robot
    def align(self):
        """Aligns robot so right side is parallel to the wall."""
        sensors = self.sensor_reading()

        if sensors.light_bumper_right < 10 and sensors.light_bumper_front_right < 10:
            # Clockwise turn in place 
            self._set_motion(0, -VELOCITY_STEPS * 2)
        else:
            # Counterclockwise turn in place
            self._set_motion(0, VELOCITY_STEPS * 2)
        
        while self.wall_follow_active:
            sensors = self.sensor_reading()

            # Rotate until no wall is detected and aligned with wall to the right
            if not self.wall_detected(sensors) and sensors.light_bumper_right >= SET_POINT:
                break
        
            time.sleep(0.1)
        
        # Stop robot (may want to comment this)
        self._set_motion(0, 0)


    @require_robot
    def drive_to_wall(self):
        """Drives robot straight and stops right before hitting a wall."""
        # Start driving robot
        self._set_motion(VELOCITY_STEPS * 3, 0)

        while self.wall_follow_active:
            sensors = self.sensor_reading()

            # Drive until a wall is detected
            if self.wall_detected(sensors):
                break
        
            time.sleep(0.1)
        
        # Stop robot (may want to comment this)
        self._set_motion(0, 0)
        

    @require_robot
    def sensor_reading(self):
        """Returns sensor values."""
        return self.robot.get_sensors()
    

    @require_robot
    def get_front_walls(self, sensors):
        """Returns front and center light bumper sensors."""
        return [sensors.light_bumper_center_left, sensors.light_bumper_center_right, 
                sensors.light_bumper_front_left, sensors.light_bumper_front_right]
    

    @require_robot
    def wall_detected(self, sensors):
        """Returns true if wall is detected in front of robot based on light bumper sensors."""
        # Get front and center light bumper sensors
        wall_sensors = self.get_front_walls(sensors) 

        # Return true if any front or center light bumper sensor value is greater than constant threshold
        if any(value > WALL_THRESHOLD for value in wall_sensors):
            return True
        
        return False
    

    @require_robot
    def squash(self, uk):
        """Determines the direction and amount of turn needed given control value.

        To simplify, let's assume we use just a P controller (using the propotional value
        and none of the others). So,
            Uk = current error = SET_POINT - LBR,
        where
            LBR is the light bumper right sensor value.
        
        The value of Uk tells us what turn we need to make:
            Uk = 0           -->  LBR = SET_POINT      -->  No correction needed, drive straight.
            Uk = SET_POINT   -->  LBR = 0              -->  No wall detected to the right, BIG RIGHT TURN.
            Uk = -SET_POINT  -->  LBR = 2 * SET_POINT  -->  Wall too close to the right, BIG LEFT TURN.
        """
        if uk <= -1000:
            return -30
        
        elif uk >= 1000:
            return 30
        
        elif uk <= -BIG_TURN_THRESHOLD:
            # Big left
            return -20
        
        elif uk >= BIG_TURN_THRESHOLD:
            # Big right
            return 20
        
        elif -BIG_TURN_THRESHOLD < uk < -SMALL_TURN_THRESHOLD:
            # Small left
            return -10
        
        elif SMALL_TURN_THRESHOLD < uk < BIG_TURN_THRESHOLD:
            # Small right
            return 10
        
        else:
            # Going straight
            return 0


    @require_robot
    def pid(self, errors, Kp, Ki, Kd, dt):
        """Basic PID controller for robot wall following."""
        # Accumulated past errors = change in time * sum of errors
        integral = errors.sum() * dt

        # Future trend = (current error - previous error) / change in time
        derivative = (errors.get_current() - errors.get_prev()) / dt

        # Control = (Proportional gain * current error) 
        #           + (integral gain * accumulated past errors) 
        #           + (derivative gain * future trend)

        Uk = Kp * errors.get_current() + Ki * integral + Kd * derivative

        # Return values
        return Uk, integral, derivative



    @require_robot
    def dock_detected(self, sensors):
        """TODO: Returns true if dock is detected."""
        if sensors.ir_opcode_right >= 161:
            return True

        return False

    @require_robot
    def find_and_approach_dock(self):
        """
        Spins in place until dock is detected on the right side,
        then approaches it.
        """
        dt = 0.1  # sensor check interval
        MAX_SEARCH_TIME = 15  # seconds
        start_time = time.time()

        logging.info("Searching for dock on the right side...")

        while True:
            sensors = self.sensor_reading()

            # If dock signal appears on right side
            if sensors.ir_opcode_right > 169:
                logging.info(f"Dock detected on right side: Opcode {sensors.ir_opcode_right}")
                break

            # Spin in place counter-clockwise
            self._set_motion(velocity=0, rotation=VELOCITY_STEPS)

            # Timeout safety
            if time.time() - start_time > MAX_SEARCH_TIME:
                logging.warning("Dock not found during spin. Timing out.")
                self._set_motion(0, 0)
                return

            time.sleep(dt)




    @require_robot
    def dock_robot(self):
        """
        Drives toward the dock using IR opcode signals until charging begins.
        Uses circular array to smooth steering decisions.
        Rotates until dock IR is detected, then attempts to dock up to 2 times.
        """
        from createlib.circular_array import CircularArray

        MAX_ATTEMPTS = 4
        dt = 0.1
        attempt = 0
        errors = CircularArray(10)

        logging.info("Scanning for IR dock signal...")

        # Step 1: Rotate in place until any IR opcode is detected
        self.find_and_approach_dock()
        time.sleep(0.1)

        # Step 2: Docking attempts
        while attempt < MAX_ATTEMPTS:
            logging.info(f"Docking attempt {attempt + 1}...")

            # Optional sweep to orient toward dock
            self._set_motion(velocity=-30, rotation=VELOCITY_STEPS * 3)
            time.sleep(1.0)
            self._set_motion(velocity=30, rotation=0)
            time.sleep(1.5)
            self._set_motion(velocity=0, rotation=-30)

            start_time = time.time()
            last_signal_time = time.time()  # <-- should be defined here


            while True:
                sensors = self.sensor_reading()

                # Enqueue IR readings
                errors.enqueue(sensors.ir_opcode)
                errors.enqueue(sensors.ir_opcode_left)
                errors.enqueue(sensors.ir_opcode_right)


                # Success: charging
                if sensors.charger_state == 2:
                    self._set_motion(0, 0)
                    logging.info("Docking successful. Charging started.")
                    return

                # Bump = bad approach → back up and retry
                bumps = sensors.bumps_wheeldrops
                if bumps.bump_left or bumps.bump_right:
                    logging.warning("Bump detected. Docking failed.")

                    # Back up
                    self._set_motion(velocity=-150, rotation=0)
                    time.sleep(1.5)

                    # Stop
                    self._set_motion(0, 0)
                    time.sleep(0.5)

                    # 🔁 Add this line:
                    self.find_and_approach_dock()
                    break
                

                # Check for IR signal presence
                if any(op in [161, 164, 165, 168, 169, 172, 173] for op in [sensors.ir_opcode, sensors.ir_opcode_left, sensors.ir_opcode_right]):
                    last_signal_time = time.time()

                # Lost signal recovery
                if time.time() - last_signal_time > 11:
                    logging.warning("Lost dock signal. Reacquiring...")
                    self._set_motion(0, 0)
                    time.sleep(0.5)
                    self.find_and_approach_dock()
                    break


                # Steering Logic (Right side only)
                if sensors.ir_opcode_right == 173:
                    self._set_motion(35, 0)
                elif sensors.ir_opcode_right == 172:
                    self._set_motion(30, 0)
                elif sensors.ir_opcode_right in [168, 169]:
                    self._set_motion(20, -VELOCITY_STEPS)
                elif sensors.ir_opcode_right in [161, 164, 165]:
                    self._set_motion(20, VELOCITY_STEPS)
                elif sensors.ir_opcode_right:
                    self._set_motion(15, 0)
                else:
                    self._set_motion(velocity=0, rotation=VELOCITY_STEPS)

                time.sleep(dt)


                # Timeout
                if time.time() - start_time > DOCK_TIMEOUT:
                    self.find_and_approach_dock()
                    time.sleep(0.1)

                time.sleep(dt)


            attempt += 1

        # Final failure
        self._set_motion(0, 0)
        logging.error("Docking failed after all attempts.")
        messagebox.showwarning("Docking Failed", "The robot could not dock after retries.")



    @require_robot
    def wall_following(self):
        """Drives robot along the wall until dock is detected."""

        # Create a circular error to store the error values
        errors = CircularArray(10)

        # Gain parameters -- to be tested/changed if needed
        Kp = 1.0        # Proportional term
        Ki = 0.001       # Integral term
        Kd = 0.015       # Derivative term
        
        dt = 0.1        # Sample rate

        # Drive to wall and align robot so right side is parallel to wall
        self.drive_to_wall()
        self.align()

        while self.wall_follow_active:
            # Get updated sensor values
            sensors = self.sensor_reading()

            # Stop if dock is detected
            if self.dock_detected(sensors): 
                break

            # Compute current error (baseline value - reading)
            error = SET_POINT - sensors.light_bumper_right
            errors.enqueue(error)

            # Get control, integral, and derivative
            Uk, integral, derivative = self.pid(errors, Kp, Ki, Kd, dt)
            
            # Compute left and right wheel velocities using control
            vl = 50 + (self.squash(Uk))
            vr = 50 - (self.squash(Uk))

            # For debugging
            print(f'Sensor Reading: {sensors.light_bumper_right}, Current Error: {error}\
                  \nProportional: {Kp * error}\
                  \nIntegral: {Ki * integral}\
                  \nDerivative: {Kd * derivative}\
                  \nControl: {Uk}\
                  \nvl: {vl}, vr: {vr}')

            # Send drive command to robot
            self.robot.drive_pwm(int(vr), int(vl))

            # Re-align with wall if detected
            if self.wall_detected(sensors):
                errors.reset()
                self.align()

            # Sensor polling rate
            time.sleep(dt)

        # Stop the robot
        self._set_motion(0, 0)

        # TEMPORARY -- to stop the robot wall following before docking if needed
        if not self.wall_follow_active:
            return
        
        # Dock the robot
        self.dock_robot()


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
