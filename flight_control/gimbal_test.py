from connection_17_02 import connect_udp
from arm_pipeline import pipeline_guided_arm_takeoff_hover
from gimbal import set_gimbal_angles_deg, scan_yaw_deg

master = connect_udp()

#--------------------------main de test gimbal-----------------------------------

# Si tu veux juste tester la gimbal sans vol, commente la ligne suivante
pipeline_guided_arm_takeoff_hover(master)

print("Gimbal: yaw +30 deg")
set_gimbal_angles_deg(master, pitch_deg=0.0, yaw_deg=30.0)

print("Gimbal: yaw -30 deg")
set_gimbal_angles_deg(master, pitch_deg=0.0, yaw_deg=-30.0)

print("Gimbal scan")
scan_yaw_deg(master, yaw_min=-60, yaw_max=60, step_deg=10, dt=0.2, cycles=1)

print("Done")