##############################################
# The MIT License (MIT)
# Copyright (c) 2017 Kevin Walchko
# Copyright (c) 2021 Nicholas Stiffler
# see LICENSE for full details
##############################################
# This decodes packets and returns a namedtuple (immutable)
##############################################
# Changelog:
#   + decode() function


from struct import Struct
from collections import namedtuple
from createlib.create_oi import WHEEL_OVERCURRENT, BUMPS_WHEEL_DROPS, BUTTONS, CHARGE_SOURCE, LIGHT_BUMPER, STASIS

def decode(format, data):
	""" Wrapper to decode various formats """
	return {
		'bool_byte'     : Struct('?').unpack,  # 1 byte bool
		'signed_byte'   : Struct('b').unpack,  # 1 signed byte
		'unsigned_byte' : Struct('B').unpack,  # 1 unsigned byte
		'signed_short'  : Struct('>h').unpack, # 2 signed bytes (short)
		'unsigned_short': Struct('>H').unpack, # 2 unsigned bytes (ushort)
	}[format](data)[0]

# Utilize namedtuples() to break up data that has relevant 'bits' 
# (page 37-38 OI). 
#   Benefit is tuple-like objects that have fields 
#   accessible by attribute lookup as well as being 
#   indexable and iterable.
BumpsAndWheelDrop = namedtuple('BumpsAndWheelDrop', ['wheeldrop_left', 'wheeldrop_right','bump_left', 'bump_right'])
WheelOvercurrents = namedtuple('WheelOvercurrents', ['left_wheel_overcurrent', 'right_wheel_overcurrent', 'main_brush_overcurrent' , 'side_brush_overcurrent' ])
Buttons           = namedtuple('Buttons', ['clock', 'schedule', 'day', 'hour', 'minute', 'dock', 'spot', 'clean'])
ChargingSources   = namedtuple('ChargingSources', ['home_base','internal_charger'])
LightBumper       = namedtuple('LightBumper', ['right', 'front_right', 'center_right', 'center_left', 'front_left', 'left'])
Stasis            = namedtuple('Stasis', ['disabled', 'toggling'])

# Every Sensor on the Create 2
Sensors = namedtuple('Sensors', [
    'bumps_wheeldrops',
    'wall',
    'cliff_left',
    'cliff_front_left',
    'cliff_front_right',
    'cliff_right',
    'virtual_wall',
    'overcurrents',
    'dirt_detect',
    'ir_opcode',
    'buttons',
    'distance',
    'angle',
    'charger_state',
    'voltage',
    'current',
    'temperature',
    'battery_charge',
    'battery_capacity',
    'wall_signal',
    'cliff_left_signal',
    'cliff_front_left_signal',
    'cliff_front_right_signal',
    'cliff_right_signal',
    'charger_available',
    'open_interface_mode',
    'song_number',
    'song_playing',
    'oi_stream_num_packets',
    'velocity',
    'radius',
    'velocity_right',
    'velocity_left',
    'encoder_counts_left',
    'encoder_counts_right',
    'light_bumper',
    'light_bumper_left',
    'light_bumper_front_left',
    'light_bumper_center_left',
    'light_bumper_center_right',
    'light_bumper_front_right',
    'light_bumper_right',
    'ir_opcode_left',
    'ir_opcode_right',
    'left_motor_current',
    'right_motor_current',
    'main_brush_current',
    'side_brush_current',
    'statis'
])

def SensorPacketDecoder(data):
    """
    This function decodes a Create 2 packet id 100  with 
    packet size (80) containging packets 7-58 and returns 
    a Sensor object, which is a namedtuple. 

    The Sensor class holds all sensor values for the Create 2.
    """

    if len(data) != 80:
        raise Exception(f"Sensor data not 80 bytes long, it is: {len(data)} bytes")

    # Notes:
    # ======
    # packets 32, 33 or data bits 36, 37, 38 - unused
    # packet 16 or data bit 9 - unused

    # Step 1: Process the named_tuples for data with relevant bits

    # Packet ID: 7 Bumps and wheel drops
    d = decode('unsigned_byte', data[0:1])
    bumps_wheeldrops = BumpsAndWheelDrop(
        bool(d & BUMPS_WHEEL_DROPS.WHEEL_DROP_LEFT),
        bool(d & BUMPS_WHEEL_DROPS.WHEEL_DROP_RIGHT),    
        bool(d & BUMPS_WHEEL_DROPS.BUMP_LEFT),
        bool(d & BUMPS_WHEEL_DROPS.BUMP_RIGHT)
    )

    # Packet ID: 14 Wheel Overcurrents
    d = decode('unsigned_byte', data[7:8])
    overcurrents = WheelOvercurrents(
        bool(d & WHEEL_OVERCURRENT.LEFT_WHEEL),
        bool(d & WHEEL_OVERCURRENT.RIGHT_WHEEL),        
        bool(d & WHEEL_OVERCURRENT.MAIN_BRUSH),
        bool(d & WHEEL_OVERCURRENT.SIDE_BRUSH),
    )

    # Packet ID: 18 Buttons
    d = decode('unsigned_byte', data[11:12])
    buttons = Buttons(
        bool(d & BUTTONS.CLOCK),
        bool(d & BUTTONS.SCHEDULE),
        bool(d & BUTTONS.DAY),
        bool(d & BUTTONS.HOUR),
        bool(d & BUTTONS.MINUTE),
        bool(d & BUTTONS.DOCK),
        bool(d & BUTTONS.SPOT),
        bool(d & BUTTONS.CLEAN)
    )

    # Packet ID: 34 Charger Available    
    d = decode('unsigned_byte', data[39:40])
    charging_sources = ChargingSources(
        bool(d & CHARGE_SOURCE.HOME_BASE),
        bool(d & CHARGE_SOURCE.INTERNAL)
    )

    # Packet ID: 45 Light Bumper
    d = decode('unsigned_byte', data[56:57])
    light_bumper = LightBumper(
        bool(d & LIGHT_BUMPER.RIGHT),
        bool(d & LIGHT_BUMPER.FRONT_RIGHT),
        bool(d & LIGHT_BUMPER.CENTER_RIGHT),
        bool(d & LIGHT_BUMPER.CENTER_LEFT),
        bool(d & LIGHT_BUMPER.FRONT_LEFT),
        bool(d & LIGHT_BUMPER.LEFT)
    )

    # Packet ID: 58 Stasis
    d = decode('unsigned_byte', data[79:80])
    stasis = Stasis(
        bool(d & STASIS.DISABLED),
        bool(d & STASIS.TOGGLING),

    )

    sensors = Sensors(                          # Packet | Description
        bumps_wheeldrops,                       # 7  (nt)
        decode('bool_byte', data[1:2]),         # 8  | wall
        decode('bool_byte', data[2:3]),         # 9  | cliff left
        decode('bool_byte', data[3:4]),         # 10 | cliff front left
        decode('bool_byte', data[4:5]),         # 11 | cliff front right
        decode('bool_byte', data[5:6]),         # 12 | cliff right
        decode('bool_byte', data[6:7]),         # 13 | virtual wall
        overcurrents,                           # 14 (nt)
        decode('signed_byte', data[8:9]),       # 15 | dirt detect
        # packet 16 or data[9:10]               # 16 -- unused
        decode('unsigned_byte', data[10:11]),   # 17 | ir opcode
        buttons,                                # 18 (nt)
        decode('signed_short', data[12:14]),    # 19 | distance
        decode('signed_short', data[14:16]),    # 20 | angle
        decode('unsigned_byte', data[16:17]),   # 21 | charge state
        decode('unsigned_short', data[17:19]),  # 22 | voltage
        decode('signed_short', data[19:21]),    # 23 | current
        decode('signed_byte', data[21:22]),     # 24 | temperature in C, use CtoF if needed
        decode('unsigned_short', data[22:24]),  # 25 | battery charge
        decode('unsigned_short', data[24:26]),  # 26 | battery capacity
        decode('unsigned_short', data[26:28]),  # 27 | wall
        decode('unsigned_short', data[28:30]),  # 28 | cliff left
        decode('unsigned_short', data[30:32]),  # 29 | cliff ront left
        decode('unsigned_short', data[32:34]),  # 30 | cliff front right
        decode('unsigned_short', data[34:36]),  # 31 | cliff right
        # packets 32 and 33 or data[36:38]      # 32 -- unused
        #                                       # 33 -- unused
        charging_sources,                       # 34 (nt)
        decode('unsigned_byte', data[40:41]),   # 35 | oi mode
        decode('unsigned_byte', data[41:42]),   # 36 | song number
        decode('bool_byte', data[42:43]),       # 37 | song playing
        decode('unsigned_byte', data[43:44]),   # 38 | oi stream num packets
        decode('signed_short', data[44:46]),    # 39 | velocity
        decode('signed_short', data[46:48]),    # 40 | turn radius
        decode('signed_short', data[48:50]),    # 41 | velocity right
        decode('signed_short', data[50:52]),    # 42 | velocity left
        decode('unsigned_short', data[52:54]),  # 43 | encoder left
        decode('unsigned_short', data[54:56]),  # 44 | encoder right
        light_bumper,                           # 45 (nt)
        decode('unsigned_short', data[57:59]),  # 46 | light bump left
        decode('unsigned_short', data[59:61]),  # 47 | light bmp front left
        decode('unsigned_short', data[61:63]),  # 48 | light bump center left
        decode('unsigned_short', data[63:65]),  # 49 | light bump center right
        decode('unsigned_short', data[65:67]),  # 50 | light bump front right
        decode('unsigned_short', data[67:69]),  # 51 | light bump right
        decode('unsigned_byte', data[69:70]),   # 52 | ir opcode left
        decode('unsigned_byte', data[70:71]),   # 53 | ir opcode right
        decode('signed_short', data[71:73]),    # 54 | left motor current
        decode('signed_short', data[73:75]),    # 55 | right motor current
        decode('signed_short', data[75:77]),    # 56 | main brush current
        decode('signed_short', data[77:79]),    # 57 | side brush current
        stasis                                  # 58 (nt)
    )

    return sensors