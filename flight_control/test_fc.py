from pymavlink import mavutil

master = mavutil.mavlink_connection("udp:0.0.0.0:14550", source_system=255)
print("Listening on udp:0.0.0.0:14550 ...")
print("Waiting heartbeat...")

hb = master.wait_heartbeat(timeout=15)
if hb is None:
    print("❌ No heartbeat")
else:
    print(f"✅ Heartbeat! sys={master.target_system} comp={master.target_component}")