import socket
import time
import math

# --- UDP ---
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", 5010))

print("Waiting for IMU data...")

roll = 0
pitch = 0
prev_time = time.time()

while True:
    data, addr = sock.recvfrom(1024)

    try:
        values = list(map(int, data.decode().strip().split(",")))

        if len(values) != 6:
            print("Bad packet:", values)
            continue

        ax, ay, az, gx, gy, gz = values

    except Exception as e:
        print("Parse error:", e)
        continue

    # --- scaling ---
    ax /= 16384.0
    ay /= 16384.0
    az /= 16384.0

    gx /= 131.0
    gy /= 131.0

    # --- dt ---
    now = time.time()
    dt = now - prev_time
    prev_time = now

    # --- accel angles ---
    roll_acc = math.atan2(ay, az) * 180 / math.pi
    pitch_acc = math.atan2(-ax, (ay**2 + az**2) ** 0.5) * 180 / math.pi

    # --- complementary filter ---
    roll = 0.98 * (roll + gx * dt) + 0.02 * roll_acc
    pitch = 0.98 * (pitch + gy * dt) + 0.02 * pitch_acc

    # --- raw debug output ---
    print(
        f"AX:{ax:.2f} AY:{ay:.2f} AZ:{az:.2f} | "
        f"GX:{gx:.2f} GY:{gy:.2f} GZ:{gz:.2f} | "
        f"ROLL:{roll:.2f} PITCH:{pitch:.2f}"
    )

    time.sleep(0.01)