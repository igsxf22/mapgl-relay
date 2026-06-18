import time
import math
import json
from dataclasses import dataclass
from dronekit import Command, connect, VehicleMode, LocationGlobalRelative
from dronekit import connect, VehicleMode, LocationGlobalRelative, Vehicle
from pymavlink import mavutil

"""
ParamName    Value	Default	Units	Min     Max
WP_LOITER_RAD	1	300	    m	    -32767  32767
WP_RADIUS	    25	50	    m	     1      32767
"""
"""
Canberra Airfield (lat:-35.362749 lon:149.165353)
West Jerrabomberra Camp A (lat:-35.360338 lon:149.151874)
West Jerrabomberra Location 2 (lat:-35.36153 lon:149.154562)
Mugga Mugga Hospital (lat:-35.354167 lon:149.15056)
"""

LOCATIONS = {
    "Canberra Airfield": LocationGlobalRelative(-35.362749, 149.165353, 20),
    "West Jerrabomberra Camp A": LocationGlobalRelative(-35.360338, 149.151874, 20),
    "West Jerrabomberra Location 2": LocationGlobalRelative(-35.36153, 149.154562, 20),
    "Mugga Mugga Hospital": LocationGlobalRelative(-35.354167, 149.15056, 20)
}


# --- Channels and Servos ---

def set_servo_pwm(vehicle, channel, pwm):
    """ Set the PWM value for a specific servo channel. """
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,
        channel,
        pwm,
        0, 0, 0, 0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def create_servo_listener(self, name, message):
    # Create DroneKit listener for SERVO_OUTPUT_RAW messages
    for i in range(1, 12):
        key = f'servo{i}_raw'
        channel = getattr(message, key)
        self.channels_out[i] = channel

def create_vehicle_channels_out(vehicle):
    """ Listener to create and update SERVO_OUTPUT_RAW channels_out attribute. """
    vehicle.channels_out = {i: 1500 for i in range(1, 12)}
    vehicle.add_message_listener('SERVO_OUTPUT_RAW', create_servo_listener)

# ---- Waypoint navigation attributes and listeners ----

def create_wpnav_attrs(vehicle):
    """ Listener to create and update vehicle waypoint navigation attributes """
    vehicle.wp_destination = None
    vehicle.wp_dist = None
    vehicle.wp_eta = None

    def _listener(self, name, message):
        if self.wp_destination is None:
            self.wp_dist = None
            self.wp_eta = None
            return
        self.wp_eta = message.wp_dist // self.airspeed if self.airspeed > 0 else None
        self.wp_dist = message.wp_dist  # in meters

    vehicle.add_message_listener('NAV_CONTROLLER_OUTPUT', _listener)

def create_arrival_listener(vehicle):
    """ Listener to set vehicle.arrived = True when within WP_RADIUS of the waypoint. """

    def _listener(self, name, message):
        if self.wp_destination is None:
            self.arrived = False
            return

        orbit_radius = self.parameters.get("WP_LOITER_RAD", 150)
        self.arrived = message.wp_dist <= orbit_radius
        if self.arrived:
            print('[arrival_listener] Arrived at waypoint! <Action can be triggered here>')

    vehicle.arrived = False
    vehicle.add_message_listener('NAV_CONTROLLER_OUTPUT', _listener)


# --- Navigation helpers ---

def get_coordinate_from_distance_heading(origin, heading, distance):
    """
    Calculate new GPS coordinate given origin, heading (degrees), and distance (meters).
    Uses simple equirectangular approximation, suitable for short distances.
    """
    R = 6378137.0  # Earth radius in meters
    lat1 = math.radians(origin[0])
    lon1 = math.radians(origin[1])
    bearing = math.radians(heading)

    lat2 = math.asin(math.sin(lat1) * math.cos(distance / R) +
                     math.cos(lat1) * math.sin(distance / R) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(distance / R) * math.cos(lat1),
                             math.cos(distance / R) - math.sin(lat1) * math.sin(lat2))

    return (math.degrees(lat2), math.degrees(lon2))


# ── Mission upload helpers ────────────────────────────────────────────────────

def upload_mission(vehicle, missionlist, end_in_loiter=False):
    print("\nUploading mission...")
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()

    for command in missionlist:
        cmds.add(command)

    if end_in_loiter:
        last_wp = missionlist[-1]
        loiter_wp = create_cmd_nav_loiter_unlim(last_wp.x, last_wp.y, last_wp.z)
        cmds.add(loiter_wp)

    vehicle.commands.next = 0
    vehicle.commands.upload()
    print("[upload_mission] Mission uploaded.")

def clear_mission(vehicle):
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()
    vehicle.mode = VehicleMode("LOITER")
    print("[clear_mission] Mission cleared - switched to LOITER.")

def get_commands(vehicle, to_json=False) -> str | list:
    """ Download and simplify the vehicle's current mission commands. Returns JSON string if to_json=True. """
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()

    MAV_CMD_MAP = {
        16: "MAV_CMD_NAV_WAYPOINT",
        22: "MAV_CMD_NAV_TAKEOFF",
        21: "MAV_CMD_NAV_LAND",
        112: "MAV_CMD_CONDITION_DELAY",
        200: "MAV_CMD_DO_SET_MODE",
        178: "MAV_CMD_DO_SET_PARAMETER",
        17: "MAV_CMD_NAV_LOITER_UNLIM",
        31: "MAV_CMD_NAV_LOITER_TO_ALT",
        20: "MAV_CMD_NAV_RETURN_TO_LAUNCH",
    }
    simplified = []
    for cmd in cmds:
        c = {
            "command": MAV_CMD_MAP.get(cmd.__dict__['command'], "UNKNOWN"),
            "x": cmd.__dict__['x'],
            "y": cmd.__dict__['y'],
            "z": cmd.__dict__['z']
        }
        simplified.append(c)
    if to_json:
        return json.dumps(simplified, indent=2)
    return simplified


# --- NAV WP command builders ---

def create_cmd_nav_waypoint(lat, lon, alt):
    return Command(0, 0, 0,
                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                   mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                   lat, lon, alt)

def create_cmd_nav_loiter_unlim(lat, lon, alt):
    return Command(0, 0, 0,
                   mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                   mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM,
                   0, 0, 0, 0, 0, 0,
                   lat, lon, alt)

# -- Data Classes and Schemas ---

@dataclass
class VehicleState:
    timestamp: float
    mode: str
    battery: float
    lat: float
    lon: float
    alt: float
    speed: float

def get_vehicle_state(vehicle) -> VehicleState:
    return VehicleState(
        timestamp=round(time.time(), 4),
        mode=vehicle.mode.name,
        battery=round(vehicle.battery.level, 2),
        lat=round(vehicle.location.global_relative_frame.lat, 8),
        lon=round(vehicle.location.global_relative_frame.lon, 8),
        alt=round(vehicle.location.global_relative_frame.alt, 8),
        speed=round(vehicle.groundspeed)
    )


# Define callback for `vehicle.location.global_relative_frame` observer
def altitude_callback(self, attr_name, value):
    """
    Create a callback for altitude.
    Triggered when `vehicle.location.global_relative_frame` updates. 
    Checks if altitude exceeds target and prints a message if so.
    Args:
        'self' - the associated vehicle object
        'attr_name' - the observed attribute (`location` for this callback)
        'value' - the updated attribute value.
    """
    global altitude_target
    alt = value.global_relative_frame.alt
    if altitude_target['target'] < alt and altitude_target['state'] == 'pending':
        print(f" CALLBACK: Alt exceeded. Set: {altitude_target}", alt)
        altitude_target={
            'target': altitude_target['target'],
            'state': 'triggered',
            'time_set': altitude_target['time_set'],
            'time_triggered': time.time()
        }



# Tests
if __name__ == "__main__":
    
    vehicle = connect('tcp:127.0.0.1:5763', wait_ready=True)
    print("Connected to vehicle on TCP=127.0.0.1:5763")
    time.sleep(0.2)

    # Init state
    vehicle_state = get_vehicle_state(vehicle)
    
    # Test output channel functions
    create_vehicle_channels_out(vehicle)
    print("Initial channels_out:", vehicle.channels_out)

    time.sleep(0.2)

    # Set Servo 5 to HIGH (2000 PWM)
    print("Setting servo 5 to HIGH (2000 PWM)")
    set_servo_pwm(vehicle, channel=5, pwm=2000)
    time.sleep(0.2)

    print("Channels_out after setting servo 5 HIGH:", vehicle.channels_out)
    time.sleep(0.2)


    # Create waypoint attributes and listeners
    create_wpnav_attrs(vehicle)
    print("Created waypoint distance and ETA attributes.")
    time.sleep(0.2)

    print("Initial wp_destination:", vehicle.wp_destination)
    print("Initial wp_dist:", vehicle.wp_dist)
    print("Initial wp_eta:", vehicle.wp_eta)
    time.sleep(0.2)

    clear_mission(vehicle)

    # Build a mission with the new destinations
    mission = [create_cmd_nav_waypoint(dest.lat, dest.lon, dest.alt) for dest in LOCATIONS.values()]
    upload_mission(vehicle, mission, end_in_loiter=True)
    time.sleep(1)


    # Use with altitude target to monitor vehicle altitude and trigger 
    # actions when the target altitude is reached
    altitude_target = {
        'target': 55,  # Target altitude in meters
        'state': 'pending',  # 'pending', 'triggered'
        'time_set': time.time(),
        'time_triggered': None
    }
    print("\nAdd `attitude` attribute callback/observer on `vehicle`")
    vehicle.add_attribute_listener('location', altitude_callback)

    # If on ground, arm and takeoff
    if vehicle.location.global_relative_frame.alt < 1.0:
        while not vehicle.armed:
            print("Arming vehicle...")
            vehicle.armed = True
            time.sleep(1)
        print("Vehicle armed. Taking off...")
        vehicle.mode = VehicleMode("TAKEOFF")
        

    while True:

        vehicle_state = get_vehicle_state(vehicle)
        print(json.dumps(vehicle_state.__dict__, indent=2))


        if vehicle.mode.name == "TAKEOFF":
            print("Vehicle is taking off. Waiting to reach target altitude...")
            time.sleep(3)
            if vehicle.location.global_relative_frame.alt >= altitude_target['target']:
                print("Target altitude reached.")
                altitude_target['state'] = 'triggered'
            continue

        # Example of switching out of TAKEOFF mode
        if (
            vehicle.location.global_relative_frame.alt > altitude_target['target'] and
            vehicle.mode.name == "TAKEOFF"
        ):
            print("Takeoff complete. Switching to AUTO")
            vehicle.mode = VehicleMode("AUTO")
            
        # If restarting script and vehicle is in LOITER mode after takeoff, check if commands exist
        # Switch to AUTO if commands exist, otherwise stay in LOITER
        if vehicle.mode.name == "LOITER":
            try:
                mission_commands = get_commands(vehicle, to_json=False)
                if mission_commands:
                    print("Mission commands exist. Switching to AUTO.")
                    vehicle.mode = VehicleMode("AUTO")
                else:
                    print("No mission commands. Staying in LOITER.")
            
            # Exit on keyboard interrupt
            except KeyboardInterrupt:
                print("Keyboard interrupt received. Exiting...")
                break


        if vehicle.mode.name == "AUTO":
            try:
                mission_commands = get_commands(vehicle, to_json=False)
                print("Current Mission Commands:\n", mission_commands)

                print("\nCurrent WP Index:", vehicle.commands.next)
                current_command = mission_commands[vehicle.commands.next] if vehicle.commands.next < len(mission_commands) else None


                print("Current Command:", current_command)
                # If no more commands, switch to LOITER
                if not current_command:
                    vehicle.mode = VehicleMode("LOITER")
                    print("No more commands. Switching to LOITER.")
                    continue
                
                if current_command['command'] == "MAV_CMD_NAV_WAYPOINT":
                    vehicle.wp_destination = LocationGlobalRelative(
                        current_command.get('x', vehicle.location.global_relative_frame.lat),
                        current_command.get('y', vehicle.location.global_relative_frame.lon),
                        current_command.get('z', vehicle.location.global_relative_frame.alt)
                    )
                time.sleep(5)

                print()
                print("Current wp_destination:", vehicle.wp_destination)
                print("Current wp_dist:", vehicle.wp_dist)
                print("Current wp_eta:", vehicle.wp_eta)

                print()

            except KeyboardInterrupt:
                print("Keyboard interrupt received. Exiting...")
                vehicle.mode = VehicleMode("LOITER")

        elif vehicle.mode.name == "RTL":
            print("Vehicle is returning to launch. Waiting for LAND command...")

        else:
            time.sleep(2)

    # JSON dump commands at end
    print("\n\nFinal Mission Commands (JSON):\n", get_commands(vehicle, to_json=True))

    vehicle.close()