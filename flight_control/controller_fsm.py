# controller_fsm.py
import time
import math
import threading
from enum import Enum, auto

from pymavlink import mavutil
from arm_pipeline import (
    pipeline_quadplane_vtol_takeoff_to_auto,
    transition_fw_to_vtol,
    set_mode_and_confirm,
    check_arspd_use,
)
from goto import send_hold_position, release_rc_override, navigate_to_target_vtol
from gimbal import set_gimbal_angles_deg
from connection import send_gcs_heartbeat, connect_udp, connect_serial, check_airspeed_sensor

METERS_PER_DEG_LAT = 111320.0

# -----------------------------------------------------------------------
# Distance de freinage QuadPlane FW → VTOL
#
# A ~18 m/s (vitesse de transition ArduPlane), le freinage en QLOITER
# prend ~100–150 m selon les PIDs et le vent.
# On déclenche la transition FW→VTOL quand le drone est encore à
# TRANSITION_TRIGGER_M de la cible, pour qu'il soit à l'arrêt à l'arrivée.
#
# À calibrer sur le terrain selon le vrai comportement de l'UAV :
#   - trop court → drone dépasse encore la cible
#   - trop long  → transition déclenchée trop tôt, drone freine loin
# -----------------------------------------------------------------------
TRANSITION_TRIGGER_M = 200.0   # distance à laquelle déclencher FW→VTOL (m)
APPROACH_RADIUS_M    = 15.0    # rayon pour démarrer le chrono de hold (m)
APPROACH_TIMEOUT_S   = 120.0   # timeout approche après transition (s)


def gcs_keepalive_tick(master, last_hb, period_s=1.0):
    now = time.time()
    if now - last_hb >= period_s:
        send_gcs_heartbeat(master)
        return now
    return last_hb

def meters_per_deg_lon(lat_deg: float) -> float:
    return 111320.0 * math.cos(math.radians(lat_deg))

def dist_to_wp_m(ref_lat, cur_lat, cur_lon, wp_lat, wp_lon):
    m_per_lon = meters_per_deg_lon(ref_lat)
    dN = (wp_lat - cur_lat) * METERS_PER_DEG_LAT
    dE = (wp_lon - cur_lon) * m_per_lon
    return math.sqrt(dN*dN + dE*dE)

def ned_to_latlon(center_lat, center_lon, dN_m, dE_m):
    lat = center_lat + dN_m / METERS_PER_DEG_LAT
    lon = center_lon + dE_m / meters_per_deg_lon(center_lat)
    return lat, lon

def generate_hypodrome_wps(center_lat, center_lon, L=98.0, W=48.0,
                           step_straight_m=10.0, step_arc_deg=15.0):
    R = W / 2.0
    straight_len = max(0.0, L - 2.0 * R)
    half_straight = straight_len / 2.0
    cN_north = +half_straight
    cN_south = -half_straight
    wps = []

    E = +R
    n = -half_straight
    while n <= +half_straight + 1e-6:
        wps.append(ned_to_latlon(center_lat, center_lon, n, E))
        n += step_straight_m

    ang = 0.0
    while ang <= 180.0 + 1e-6:
        phi = math.radians(ang)
        N = cN_north + R * math.cos(phi)
        E = R * math.sin(phi)
        wps.append(ned_to_latlon(center_lat, center_lon, N, E))
        ang += step_arc_deg

    E = -R
    n = +half_straight
    while n >= -half_straight - 1e-6:
        wps.append(ned_to_latlon(center_lat, center_lon, n, E))
        n -= step_straight_m

    ang = 180.0
    while ang <= 360.0 + 1e-6:
        phi = math.radians(ang)
        N = cN_south + R * math.cos(phi)
        E = R * math.sin(phi)
        wps.append(ned_to_latlon(center_lat, center_lon, N, E))
        ang += step_arc_deg

    return wps


def wait_disarmed(master, timeout=120):
    t0 = time.time()
    last_hb = 0.0
    while time.time() - t0 < timeout:
        last_hb = gcs_keepalive_tick(master, last_hb, period_s=1.0)
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if hb:
            armed = (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
            if not armed:
                print("✅ UAV disarmed")
                return True
    raise RuntimeError("Disarming timeout")


def _kill_switch_listener(master):
    """
    Thread de fond qui écoute la touche K.
    Appuyer sur K + Entrée déclenche le kill switch.
    """
    print("⚠️  Kill switch actif – appuie sur K + Entrée pour désarmer en vol")
    while True:
        try:
            key = input()
            if key.strip().upper() == "K":
                from arm_pipeline import emergency_kill
                emergency_kill(master)
                break
        except Exception:
            break


def wait_landed(master, timeout=240):
    """
    EXTENDED_SYS_STATE.landed_state:
      1 = ON_GROUND
    """
    t0 = time.time()
    last_print = 0.0
    last_hb = 0.0
    while time.time() - t0 < timeout:
        last_hb = gcs_keepalive_tick(master, last_hb, period_s=1.0)
        msg = master.recv_match(type="EXTENDED_SYS_STATE", blocking=True, timeout=1)
        if msg and int(msg.landed_state) == 1:
            print("✅ UAV landed")
            return True
        if time.time() - last_print > 2.0:
            print("🟡 Waiting for landing...")
            last_print = time.time()
    raise RuntimeError("Timeout waiting for landing")


class State(Enum):
    SEARCH_FW            = auto()
    TRACK_DETECTED       = auto()
    ANTICIPATE_TRANSITION = auto()   # nouveau : attend d'être à TRANSITION_TRIGGER_M
    TRANSITION_TO_VTOL   = auto()
    VTOL_HOLD_OVER_TARGET = auto()
    RETURN_HOME          = auto()
    FAILSAFE             = auto()


def main():
    # ---- Params ----
    ALT_TARGET_M = 30.0
    FW_SEARCH_AIRSPD_MPS = 14.0
    DT           = 0.5
    PITCH_DEG    = -30.0
    HOLD_TIME_S  = 30.0

    # ---- Setup ----
    # Vol réel via télémétrie série (SiK / RFD900 / etc.)
    # Adapter le port selon l'OS :
    #   Linux   → "/dev/ttyUSB0"  ou  "/dev/ttyACM0"
    #   Windows → "COM3", "COM5", etc.
    master = connect_serial(port="/dev/ttyUSB0", baud=57600)
    # master = connect_udp()   # ← décommenter pour revenir en SITL/UDP

    kill_thread = threading.Thread(target=_kill_switch_listener, args=(master,), daemon=True)
    kill_thread.start()

    # --- Vérification pré-vol ---
    print("\n=== PRÉ-VOL: vérification capteur pitot ===")
    airspeed_ok = check_airspeed_sensor(master, timeout=5.0)
    if airspeed_ok:
        # Vérifier aussi ARSPD_USE même si le capteur est sain
        check_arspd_use(master)
    else:
        print("⚠️  Capteur pitot non détecté – DO_CHANGE_SPEED sera rejeté")
        print("   Continuer ? (Ctrl+C pour annuler, Enter pour continuer sans pitot)")
        try:
            input()
        except KeyboardInterrupt:
            print("Annulé.")
            return
    print("==========================================\n")

    start_time = time.time()

    pipeline_quadplane_vtol_takeoff_to_auto(master, target_alt=ALT_TARGET_M, airspeed_mps=FW_SEARCH_AIRSPD_MPS)

    # scan gimbal
    yaw_offset_deg = -45.0
    direction = 1

    # state vars
    state = State.SEARCH_FW
    target_latlon = None
    last_lat = None
    last_lon = None
    last_gcs_hb = time.time()

    while True:

        last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

        # --- Position (non-blocking) ---
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=False)
        if msg is not None:
            last_lat = msg.lat / 1e7
            last_lon = msg.lon / 1e7

        if last_lat is None or last_lon is None:
            time.sleep(DT)
            continue

        # ================================================================
        if state == State.SEARCH_FW:

            set_gimbal_angles_deg(master, pitch_deg=PITCH_DEG, yaw_deg=yaw_offset_deg)
            yaw_offset_deg += direction * 10.0
            if yaw_offset_deg >= 45.0:
                yaw_offset_deg, direction = 45.0, -1
            elif yaw_offset_deg <= -45.0:
                yaw_offset_deg, direction = -45.0, 1

            # ---- Intégration NN (remplacer ce bloc) ----
            # if nn_detected:
            #     target_latlon = (det_lat, det_lon)
            #     state = State.TRACK_DETECTED

            # Simulation détection après 50s
            if time.time() - start_time > 120.0:
                print("🎯 SIMULATED TARGET DETECTED")
                target_latlon = (last_lat, last_lon)
                state = State.TRACK_DETECTED

            time.sleep(DT)
            if state != State.SEARCH_FW:
                continue

        # ================================================================
        elif state == State.TRACK_DETECTED:
            if target_latlon is None:
                state = State.SEARCH_FW
                continue
            tgt_lat, tgt_lon = target_latlon
            d = dist_to_wp_m(tgt_lat, last_lat, last_lon, tgt_lat, tgt_lon)
            print(f"🎯 Target locked at {tgt_lat:.7f}, {tgt_lon:.7f} (d={d:.1f}m)")
            print(f"   Transition will trigger at d <= {TRANSITION_TRIGGER_M}m")
            state = State.ANTICIPATE_TRANSITION

        # ================================================================
        elif state == State.ANTICIPATE_TRANSITION:
            # On continue à scanner avec le gimbal et on attend que le drone
            # soit assez proche pour déclencher la transition FW→VTOL.
            # Le drone continue sa trajectoire AUTO pendant ce temps.
            tgt_lat, tgt_lon = target_latlon
            d = dist_to_wp_m(tgt_lat, last_lat, last_lon, tgt_lat, tgt_lon)

            set_gimbal_angles_deg(master, pitch_deg=PITCH_DEG, yaw_deg=0.0)

            if d <= TRANSITION_TRIGGER_M:
                print(f"🟡 Transition trigger! d={d:.1f}m <= {TRANSITION_TRIGGER_M}m -> FW→VTOL")
                state = State.TRANSITION_TO_VTOL
            else:
                print(f"   Approaching target: d={d:.1f}m (trigger at {TRANSITION_TRIGGER_M}m)")
                time.sleep(DT)

        # ================================================================
        elif state == State.TRANSITION_TO_VTOL:
            transition_fw_to_vtol(master)
            # Pas de QLOITER ici : navigate_to_target_vtol passe en AUTO
            # pour la navigation active, puis en QLOITER pour le hold statique.
            state = State.VTOL_HOLD_OVER_TARGET

        # ================================================================
        elif state == State.VTOL_HOLD_OVER_TARGET:
            tgt_lat, tgt_lon = target_latlon
            last_gcs_hb = time.time()

            print(f"🎯 Navigating to target: lat={tgt_lat:.7f}, lon={tgt_lon:.7f}")

            # --- Phase 1 : navigation active vers la cible ---
            # Utilise AUTO + mini-mission (NAV_WAYPOINT + LOITER_UNLIM).
            # ArduPlane navigue activement vers la cible au lieu de rester
            # figé là où le freinage FW→VTOL s'est arrêté.
            def _hb():
                nonlocal last_gcs_hb
                last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

            arrived = navigate_to_target_vtol(
                master,
                tgt_lat=tgt_lat,
                tgt_lon=tgt_lon,
                alt_rel_m=ALT_TARGET_M,
                arrival_radius_m=APPROACH_RADIUS_M,
                timeout_s=APPROACH_TIMEOUT_S,
                gcs_keepalive_fn=_hb,
            )

            if arrived:
                print(f"✅ On target – switching to QLOITER for stationary hold")
            else:
                print(f"⚠️  Navigation timeout – holding at current position")

            set_mode_and_confirm(master, "QLOITER", timeout=15)

            # --- Phase 2 : hold statique au-dessus de la cible ---
            print(f"⏱  HOLD stationnaire {HOLD_TIME_S:.0f}s...")
            t0 = time.time()
            while True:
                last_gcs_hb = gcs_keepalive_tick(master, last_gcs_hb, period_s=1.0)

                msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
                if msg is not None:
                    last_lat = msg.lat / 1e7
                    last_lon = msg.lon / 1e7

                send_hold_position(master, tgt_lat, tgt_lon, alt_rel_m=ALT_TARGET_M)

                d = dist_to_wp_m(tgt_lat, last_lat, last_lon, tgt_lat, tgt_lon)
                elapsed = time.time() - t0

                if elapsed >= HOLD_TIME_S:
                    print(f"✅ HOLD done: t={elapsed:.1f}s (d={d:.1f}m) -> RETURN_HOME")
                    break

                time.sleep(DT)

            release_rc_override(master)
            time.sleep(0.2)
            state = State.RETURN_HOME

        # ================================================================
        elif state == State.RETURN_HOME:
            print("RETURN HOME: switching to QRTL (safe VTOL return mode)")
            set_mode_and_confirm(master, "QRTL", timeout=15)
            print("Waiting for landing + disarm...")
            wait_landed(master, timeout=240)
            wait_disarmed(master, timeout=120)
            print("✅ RETURN HOME complete")
            break

        # ================================================================
        elif state == State.FAILSAFE:
            print("🚨 FAILSAFE: switching to QRTL")
            try:
                set_mode_and_confirm(master, "QRTL", timeout=10)
            except Exception:
                pass
            break


if __name__ == "__main__":
    main()
