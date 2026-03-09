import time
import math
from pymavlink import mavutil

# -----------------------------------------------------------------------
# Gimbal control pour ArduPilot >= 4.0
#
# On utilise le message natif GIMBAL_MANAGER_SET_PITCHYAW (msg id 287)
# au lieu de MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW (command_long).
#
# Pourquoi :
#   command_long attend un ACK → ArduPilot répond FAILED en SITL
#   (MNT_TYPE=0, aucun gimbal configuré) → pollution des logs
#
#   GIMBAL_MANAGER_SET_PITCHYAW est un message de streaming (pas de ACK)
#   → silencieux en SITL, fonctionnera sur hardware réel avec gimbal configuré
#
# Angles convention ArduPilot :
#   pitch_deg : 0 = horizontal, -90 = nadir (vers le sol)
#   yaw_deg   : relatif drone si GIMBAL_MANAGER_FLAGS_YAW_LOCK absent
#               absolu (nord) si GIMBAL_MANAGER_FLAGS_YAW_LOCK présent
# -----------------------------------------------------------------------

GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16   # bit 4 = yaw absolu (nord)


def set_gimbal_angles_deg(master, pitch_deg, yaw_deg, roll_deg=0.0,
                          yaw_lock=False, gimbal_device_id=0):
    """
    Commande le gimbal via GIMBAL_MANAGER_SET_PITCHYAW (message natif, pas d'ACK).
    pitch_deg : 0 = horizontal, -90 = nadir
    yaw_deg   : relatif drone (yaw_lock=False) ou absolu nord (yaw_lock=True)
    Silencieux en SITL sans gimbal configuré.
    """
    flags = GIMBAL_MANAGER_FLAGS_YAW_LOCK if yaw_lock else 0

    # Conversion degrés → radians (GIMBAL_MANAGER_SET_PITCHYAW utilise des radians)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad   = math.radians(yaw_deg)

    master.mav.gimbal_manager_set_pitchyaw_send(
        master.target_system,
        master.target_component,
        flags,
        gimbal_device_id,
        pitch_rad,         # pitch en radians
        yaw_rad,           # yaw en radians
        float('nan'),      # pitch rate ignoré
        float('nan'),      # yaw rate ignoré
    )


def set_gimbal_angles_rad(master, pitch_rad, yaw_rad, yaw_lock=False):
    """Wrapper en radians."""
    set_gimbal_angles_deg(
        master,
        pitch_deg=math.degrees(pitch_rad),
        yaw_deg=math.degrees(yaw_rad),
        yaw_lock=yaw_lock
    )


def scan_yaw_deg(master, yaw_min=-45.0, yaw_max=45.0,
                 pitch_deg=0.0, step_deg=5.0, dt=0.2, cycles=1):
    """
    Balayage gimbal en yaw (relatif drone).
    pitch_deg fixe pendant le scan.
    """
    for _ in range(cycles):
        yaw = yaw_min
        while yaw <= yaw_max + 1e-6:
            set_gimbal_angles_deg(master, pitch_deg=pitch_deg, yaw_deg=yaw)
            time.sleep(dt)
            yaw += step_deg
        yaw = yaw_max
        while yaw >= yaw_min - 1e-6:
            set_gimbal_angles_deg(master, pitch_deg=pitch_deg, yaw_deg=yaw)
            time.sleep(dt)
            yaw -= step_deg


def point_gimbal_at_target(master, drone_lat, drone_lon, drone_alt_m,
                           target_lat, target_lon, target_alt_m=0.0):
    """
    Calcule et envoie les angles pitch/yaw pour pointer le gimbal
    vers une cible GPS depuis la position courante du drone.
    Utilise yaw_lock=True (yaw absolu / nord).
    """
    METERS_PER_DEG_LAT = 111320.0
    m_per_lon = 111320.0 * math.cos(math.radians(drone_lat))

    dN = (target_lat - drone_lat) * METERS_PER_DEG_LAT
    dE = (target_lon - drone_lon) * m_per_lon
    dZ = target_alt_m - drone_alt_m

    horiz_dist = math.sqrt(dN**2 + dE**2)

    yaw_deg = math.degrees(math.atan2(dE, dN))
    pitch_deg = math.degrees(math.atan2(dZ, horiz_dist)) if horiz_dist > 0.5 else -90.0

    set_gimbal_angles_deg(master, pitch_deg=pitch_deg, yaw_deg=yaw_deg, yaw_lock=True)
    return pitch_deg, yaw_deg
