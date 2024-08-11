from pymavlink import mavutil, mavwp
from pymavlink import mavextra
from pymavlink import mavexpression
import time

master: mavutil.mavudp | None = mavutil.mavlink_connection('udp:localhost:14550')

mavutil.set_dialect("ardupilotmega")
from pymavlink.dialects.v10 import ardupilotmega
assert mavutil.mavlink is ardupilotmega
from pymavlink.dialects.v10.ardupilotmega import MAVLink_message as msg_t, MAVLink_sys_status_message as stat_t
apmu: ardupilotmega = mavutil.mavlink

master.wait_heartbeat()

def prearm_health(pre_timeout=15) -> int:
    """
    Checks the health status of the onboard control sensors before arming.

    Args:
        pre_timeout (int): The timeout value in seconds for checking the health status. Default is 15 seconds.

    Returns:
        int: The health status of the onboard control sensors. Possible values are:
            - 0: Unknown
            - -1: Bad
            - 1: Good
    """
    pre_threshold = time.time() + pre_timeout
    health = 0 #0 unknown, -1 bad, 1 good
    while True and time.time() < pre_threshold:
        msg: stat_t | None = master.recv_match(type='SYS_STATUS', blocking=True)
        if msg:
            bits = apmu.MAV_SYS_STATUS_PREARM_CHECK
            present = ((msg.onboard_control_sensors_present & bits) == bits)
            enabled = ((msg.onboard_control_sensors_enabled & bits) == bits)
            if not present or not enabled:
                continue
            
            prearmable = ((msg.onboard_control_sensors_health & bits) == bits)
            if prearmable:
                health = 1
                break
            else:
                health = -1
    return health

def try_arm(arm_timeout=10):
    arm_threshold = time.time() + arm_timeout
    health = 0 #0 unknown, -1 bad, 1 good
    while True and time.time() < arm_threshold:
        master.arducopter_arm()
        master.wait_heartbeat()
        if master.motors_armed():
            print('a')
            return True
        time.sleep(0.5)
    return False

def safe_arm(pre_timeout=25, post_timeout=10):
    health = prearm_health(pre_timeout)
    if health == 1:
        try_arm(post_timeout)

    

# def read_loop():
#     while True:
#         # grab a mavlink message
#         msg = master.recv_match(blocking=True)

#         # handle the message based on its type
#         msg_type = msg.get_type()
#         if msg_type == "VFR_HUD":
#             handle_hud(msg)
#         if msg_type == "BAD_DATA":
#             if mavutil.all_printable(msg.data):
#                 sys.stdout.write(msg.data)
#                 sys.stdout.flush()
#         elif msg_type == "RC_CHANNELS":
#             handle_rc_raw(msg)
#         elif msg_type == "VFR_HUD":
#             handle_hud(msg)
#         elif msg_type == "ATTITUDE":
#             handle_attitude(msg)
#         elif msg_type == "NAV_CONTROLLER_OUTPUT":
#             handle_target(msg)
#         elif msg_type == "GLOBAL_POSITION_INT":
#             handle_position(msg)
#         elif msg_type == "STATUSTEXT":
#             handle_status(msg)
#         elif msg_type == "SYSTEM_TIME":
#             handle_time(msg)
#         elif msg_type == "MISSION_COUNT":
#             handle_mission(msg)
#         elif msg_type == "PARAM_VALUE":
#             handle_param(msg)
#         elif msg_type == "GPS_RAW_INT":
#             handle_gps(msg)
#         elif msg_type == "HEARTBEAT":
#             handle_heartbeat(msg)

def squerror(rads, duty, period, total):
    while time.time() < init + total:
        pinit = time.time()
        master.param_set_send("SIM_GYR1_BIAS_X", rads)
        time.sleep(period*duty)
        master.param_set_send("SIM_GYR1_BIAS_X", 0)
        time.sleep(period*(1-duty))

# init = time.time()
# squerror(0.783, 0.5, 2, 90)
safe_arm()

