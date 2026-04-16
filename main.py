import cv2
import serial
import time

from rec import IMUReceiver
from control import Stabilizer


ser = serial.Serial("COM3", 115200)
time.sleep(2)

imu = IMUReceiver()
stabilizer = Stabilizer()


cap = cv2.VideoCapture(0)
qr_detector = cv2.QRCodeDetector()


target_dx = 0
target_dy = 0
qr_detected = False

alpha = 0.7

# Channels
yaw_cmd = 1500

# Timing
CONTROL_DT = 0.02
start_time = time.time()
last_imu_time = time.time()

while True:
    loop_start = time.time()


    #  IMU
    imu_roll, imu_pitch = imu.get_angles()
    last_imu_time = time.time()

    #  Camera (QR detection)

    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        data, bbox, _ = qr_detector.detectAndDecode(gray)

        if bbox is not None and data:
            qr_detected = True

            x_coords = [p[0] for p in bbox[0]]
            y_coords = [p[1] for p in bbox[0]]

            qr_x = int(sum(x_coords) / 4)
            qr_y = int(sum(y_coords) / 4)

            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2

            # smoothing
            target_dx = alpha * target_dx + (1 - alpha) * (qr_x - center_x)
            target_dy = alpha * target_dy + (1 - alpha) * (qr_y - center_y)

        else:
            qr_detected = False

        cv2.imshow("frame", frame)

    # 3. Target

    target_roll = 0
    target_pitch = 0

    if qr_detected:
        target_roll = -target_dx * 0.05
        target_pitch = target_dy * 0.03


    #  Stabilization (main control loop)
    roll_cmd, pitch_cmd = stabilizer.stabilize(
        imu_roll - target_roll,
        imu_pitch - target_pitch
    )


    #  Base throttle (takeoff / hover)

    elapsed = time.time() - start_time

    if elapsed < 5:
        base_throttle = 1000 + int(elapsed * 80)
    else:
        base_throttle = 1350

    #  Soft tilt compensation
    tilt = abs(imu_roll) + abs(imu_pitch)
    throttle = base_throttle + int(tilt * 0.3)

    #  Minimal QR influence on throttle
    if qr_detected and abs(target_dy) > 50:
        throttle += int(target_dy * 0.02)

    #  Limits
    roll_cmd = max(1000, min(2000, roll_cmd))
    pitch_cmd = max(1000, min(2000, pitch_cmd))
    throttle = max(1000, min(1700, throttle))

    #  Failsafe (IMU loss protection)
    if time.time() - last_imu_time > 0.1:
        roll_cmd = 1500
        pitch_cmd = 1500
        throttle = 1200

    #  Send to drone
    channels = [
        roll_cmd,
        pitch_cmd,
        throttle,
        yaw_cmd,
        1000, 1000, 1000, 1000
    ]

    line = ",".join(map(str, channels)) + "\n"
    ser.write(line.encode("ascii"))

    print(f"R:{roll_cmd} P:{pitch_cmd} T:{throttle}")

    # 50 Hz target
    elapsed_loop = time.time() - loop_start
    sleep_time = CONTROL_DT - elapsed_loop

    if sleep_time > 0:
        time.sleep(sleep_time)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()