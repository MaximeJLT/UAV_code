# check_params.py
from connection import connect_udp
from arm_pipeline import _read_param_float

master = connect_udp()

params_to_check = [
    "TRIM_ARSPD_CM",
    "AIRSPEED_CRUISE",
    "Q_TRANSITION_MS",
    "ARSPD_FBW_MAX",
    "ARSPD_FBW_MIN",
    "CRUISE_SPEED",
    "WP_SPEED",
    "AIRSPEED_MAX",
    "AIRSPEED_MIN", 
    "AIRSPEED_CRUISE",
    "Q_TRANSITION_MS",
    "Q_TRANS_DECEL",
    "FBWB_CLIMB_RATE",
    "TECS_SPDWEIGHT",
    "TECS_CRUISE_THR",
    "AIRSPEED_MAX",
    "AIRSPEED_MIN",
    "AIRSPEED_CRUISE",
    "Q_TRANSITION_MS",
    "Q_TRANS_DECEL",
    "Q_ASSIST_SPEED",
    "Q_TRAN_FAIL_ACT",
    "TECS_SPDWEIGHT",
    "TECS_TIME_CONST",
    "TECS_THR_DAMP",
    "SERVO_AUTO_TRIM",
    "Q_M_THST_HOVER",
]

for name in params_to_check:
    val = _read_param_float(master, name, timeout=2.0)
    if val is not None:
        print(f"  {name} = {val}")
    else:
        print(f"  {name} = *** NOT FOUND ***")