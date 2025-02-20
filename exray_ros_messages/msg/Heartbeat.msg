# The heartbeat message shows that a system is present and responding.
# The type of the MAV and autopilot hardware allows the receiving system to handle further messages appropriately (e.g., by adapting the user interface based on the autopilot type).

uint8 sysid            # ID of the system.
uint8 compid           # Component ID within the system.
uint8 type             # Type of the MAV (e.g., quadrotor, helicopter, etc.). Defined in MAV_TYPE ENUM.
uint8 autopilot        # Autopilot type. Defined in MAV_AUTOPILOT ENUM.
uint8 base_mode        # System mode bitfield. Defined in MODE_FLAG ENUM.
uint32 custom_mode     # A bitfield for use for autopilot-specific flags.
uint8 system_status    # System status flag. Defined in MAV_STATE ENUM.
uint8 mavlink_version  # MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version.


# MAV_TYPE Definitions (Enumerated Values).
uint8 MAV_TYPE_GENERIC=0              # Generic micro air vehicle.
uint8 MAV_TYPE_FIXED_WING=1           # Fixed wing aircraft.
uint8 MAV_TYPE_QUADROTOR=2            # Quadrotor.
uint8 MAV_TYPE_COAXIAL=3              # Coaxial helicopter.
uint8 MAV_TYPE_HELICOPTER=4           # Normal helicopter with tail rotor.
uint8 MAV_TYPE_ANTENNA_TRACKER=5      # Ground installation.
uint8 MAV_TYPE_GCS=6                  # Operator control unit / ground control station.
uint8 MAV_TYPE_AIRSHIP=7              # Airship, controlled.
uint8 MAV_TYPE_FREE_BALLOON=8         # Free balloon, uncontrolled.
uint8 MAV_TYPE_ROCKET=9               # Rocket.
uint8 MAV_TYPE_GROUND_ROVER=10        # Ground rover.
uint8 MAV_TYPE_SURFACE_BOAT=11        # Surface vessel, boat, ship.
uint8 MAV_TYPE_SUBMARINE=12           # Submarine.
uint8 MAV_TYPE_HEXAROTOR=13           # Hexarotor.
uint8 MAV_TYPE_OCTOROTOR=14           # Octorotor.
uint8 MAV_TYPE_TRICOPTER=15           # Tricopter.
uint8 MAV_TYPE_FLAPPING_WING=16       # Flapping wing.
uint8 MAV_TYPE_KITE=17                # Kite.
uint8 MAV_TYPE_ONBOARD_CONTROLLER=18  # Onboard companion controller.
uint8 MAV_TYPE_VTOL_DUOROTOR=19       # Two-rotor VTOL using control surfaces in vertical operation in addition. Tailsitter.
uint8 MAV_TYPE_VTOL_QUADROTOR=20      # Quad-rotor VTOL using a V-shaped quad config in vertical operation. Tailsitter.
uint8 MAV_TYPE_VTOL_TILTROTOR=21      # Tiltrotor VTOL.
uint8 MAV_TYPE_VTOL_RESERVED2=22      # VTOL reserved 2.
uint8 MAV_TYPE_VTOL_RESERVED3=23      # VTOL reserved 3.
uint8 MAV_TYPE_VTOL_RESERVED4=24      # VTOL reserved 4.
uint8 MAV_TYPE_VTOL_RESERVED5=25      # VTOL reserved 5.
uint8 MAV_TYPE_GIMBAL=26              # Gimbal.
uint8 MAV_TYPE_ADSB=27                # ADSB system.

# MAV_AUTOPILOT Definitions (Enumerated Values).
uint8 MAV_AUTOPILOT_GENERIC=0                                        # Generic autopilot, full support for everything.
uint8 MAV_AUTOPILOT_RESERVED=1                                       # Reserved for future use.
uint8 MAV_AUTOPILOT_SLUGS=2                                          # SLUGS autopilot, http://slugsuav.soe.ucsc.edu
uint8 MAV_AUTOPILOT_ARDUPILOTMEGA=3                                  # ArduPilotMega / ArduCopter, http://diydrones.com
uint8 MAV_AUTOPILOT_OPENPILOT=4                                      # OpenPilot, http://openpilot.org
uint8 MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY=5                         # Generic autopilot only supporting simple waypoints.
uint8 MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY=6   # Generic autopilot supporting waypoints and other simple navigation commands.
uint8 MAV_AUTOPILOT_GENERIC_MISSION_FULL=7                           # Generic autopilot supporting the full mission command set.
uint8 MAV_AUTOPILOT_INVALID=8                                        # No valid autopilot, e.g. a GCS or other MAVLink component.
uint8 MAV_AUTOPILOT_PPZ=9                                            # PPZ UAV - http://nongnu.org/paparazzi
uint8 MAV_AUTOPILOT_UDB=10                                           # UAV Dev Board.
uint8 MAV_AUTOPILOT_FP=11                                            # FlexiPilot.
uint8 MAV_AUTOPILOT_PX4=12                                           # PX4 Autopilot - http://pixhawk.ethz.ch/px4/
uint8 MAV_AUTOPILOT_SMACCMPILOT=13                                   # SMACCMPilot - http://smaccmpilot.org
uint8 MAV_AUTOPILOT_AUTOQUAD=14                                      # AutoQuad -- http://autoquad.org
uint8 MAV_AUTOPILOT_ARMAZILA=15                                      # Armazila -- http://armazila.com
uint8 MAV_AUTOPILOT_AEROB=16                                         # Aerob -- http://aerob.ru
uint8 MAV_AUTOPILOT_ASLUAV=17                                        # ASLUAV autopilot -- http://www.asl.ethz.ch

# MODE_FLAG Enum defining System Modes (Bitfield).
uint8 MODE_FLAG_SAFETY_ARMED=128         # 0b10000000 MAV safety set to armed. Motors are enabled / running / can start. Ready to fly.
uint8 MODE_FLAG_MANUAL_INPUT_ENABLED=64  # 0b01000000 remote control input is enabled.
uint8 MODE_FLAG_HIL_ENABLED=32           # 0b00100000 hardware in the loop simulation. All motors / actuators are blocked, but internal software is full operational.
uint8 MODE_FLAG_STABILIZE_ENABLED=16     # 0b00010000 system stabilizes electronically its attitude (and optionally position). It needs however further control inputs to move around.
uint8 MODE_FLAG_GUIDED_ENABLED=8         # 0b00001000 guided mode enabled, system flies waypoints / mission items.
uint8 MODE_FLAG_AUTO_ENABLED=4           # 0b00000100 autonomous mode enabled, system finds its own goal positions. Guided flag can be set or not, depends on the actual implementation.
uint8 MODE_FLAG_TEST_ENABLED=2           # 0b00000010 system has a test mode enabled. This flag is intended for temporary system tests and should not be used for stable implementations.
uint8 MODE_FLAG_CUSTOM_MODE_ENABLED=1    # 0b00000001 Reserved for future use.


# MAV_STATE Enum defining System Status (Enumerated Values).
uint8 MAV_STATE_UNINIT=0        # Uninitialized system, state is unknown.
uint8 MAV_STATE_BOOT=1          # System is booting up.
uint8 MAV_STATE_CALIBRATING=2   # System is calibrating and not flight-ready.
uint8 MAV_STATE_STANDBY=3       # System is grounded and on standby. It can be launched any time.
uint8 MAV_STATE_ACTIVE=4        # System is active and might be already airborne. Motors are engaged.
uint8 MAV_STATE_CRITICAL=5      # System is in a non-normal flight mode. It can however still navigate.
uint8 MAV_STATE_EMERGENCY=6     # System is in a non-normal flight mode. It lost control over parts or over the whole airframe. It is in mayday and going down.
uint8 MAV_STATE_POWEROFF=7      # System just initialized its power-down sequence, will shut down now.
