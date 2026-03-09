from pymavlink import mavutil
import time

#------------------------Temps defini en attendant la boucle de l'IA-----------------------------

def send_velocity_ned(master, vx, vy, vz, duration_s): #Pour les tests
    type_mask = 0b0000111111000111  # use only vx,vy,vz

    start = time.time()
    while time.time() - start < duration_s:
        master.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (on met 0, SITL s'en fout)
            master.target_system, #communication with UAV
            master.target_component, #autopilot communication
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,      # x,y,z ignored
            vx, vy, vz,   # velocities used in m/s
            0, 0, 0,      # acceleration ignored
            0, 0          # yaw, yaw_rate ignored (flat roation)
        )
        time.sleep(0.1) #frequency for ardupilot

def stop(master):
    send_velocity_ned(master, 0, 0, 0, 0.5) #stop the drone

def send_velocity_once(master, vx, vy, vz): #Pour l'implémentation dans la boucle
    """
    Envoie UNE consigne de vitesse (1 message).
    À appeler dans une boucle (ex: 10 Hz).
    """
    type_mask = 0b0000111111000111  # use only vx,vy,vz
    master.mav.set_position_target_local_ned_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
