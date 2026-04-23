import cv2
import serial
import time

from rec import IMUReceiver
from control import Stabilizer


# =========================
# Serial (with fallback)
# =========================
try:
    ser = serial.Serial("COM3", 115200)
    time.sleep(2)
    serial_enabled = True
    print("Serial OK")
except:
    serial_enabled = False
    print("TEST MODE (no COM)")


# =========================
# Modules
# =========================
imu = IMUReceiver()
stabilizer = Stabilizer()

cap = cv2.VideoCapture(0)
qr_detector = cv2.QRCodeDetector()


# =========================
# State
# =========================
target_dx = 0
target_dy = 0
qr_detected = False

alpha = 0.4

yaw_cmd = 1500

CONTROL_DT = 0.02
start_time = time.time()
last_imu_time = time.time()


# =========================
# Main loop
# =========================
while True:
    loop_start = time.time()

    # =========================
    # IMU
    # =========================
    imu_roll, imu_pitch = imu.get_angles()

    last_imu_time = time.time()

    # =========================
    # Camera (QR)
    # =========================
    ret, frame = cap.read()
    if ret:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        data, bbox, _ = qr_detector.detectAndDecode(gray)

        if bbox is not None:
            qr_detected = True

            pts = bbox.reshape(4, 2)

            #  рамка
            for i in range(4):
                pt1 = tuple(map(int, pts[i]))
                pt2 = tuple(map(int, pts[(i + 1) % 4]))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            # центр QR
            qr_x = int(sum(p[0] for p in pts) / 4)
            qr_y = int(sum(p[1] for p in pts) / 4)

            cv2.circle(frame, (qr_x, qr_y), 5, (0, 0, 255), -1)

            # центр экрана
            center_x = frame.shape[1] // 2
            center_y = frame.shape[0] // 2

            cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)

            #  линия между ними
            cv2.line(frame, (center_x, center_y), (qr_x, qr_y), (255, 255, 0), 2)

            # расчёт смещения
            target_dx = alpha * target_dx + (1 - alpha) * (qr_x - center_x)
            target_dy = alpha * target_dy + (1 - alpha) * (center_y- qr_y)

        else:
            qr_detected = False

        cv2.imshow("frame", frame)

    # =========================
    # Target
    # =========================
    target_roll = 0
    target_pitch = 0

    if qr_detected:
        target_roll = -target_dx * 0.15
        target_pitch = target_dy * 0.15

    # =========================
    # Stabilization
    # =========================
    roll_cmd, pitch_cmd = stabilizer.stabilize(
        imu_roll - target_roll,
        imu_pitch - target_pitch
    )

    # =========================
    # Throttle
    # =========================
    elapsed = time.time() - start_time

    if elapsed < 5:
        base_throttle = 1000 + int(elapsed * 80)
    else:
        base_throttle = 1350

    # tilt compensation
    tilt = abs(imu_roll) + abs(imu_pitch)
    throttle = base_throttle + int(tilt * 0.3)

    # QR correction
    if qr_detected and abs(target_dy) > 20:
        throttle += int(target_dy * 0.3)

    # =========================
    # Limits
    # =========================
    roll_cmd = max(1000, min(2000, roll_cmd))
    pitch_cmd = max(1000, min(2000, pitch_cmd))
    throttle = max(1000, min(1700, throttle))

    # =========================
    # Failsafe (IMU)
    # =========================
    if time.time() - last_imu_time > 0.1:
        roll_cmd = 1500
        pitch_cmd = 1500
        throttle = 1000

    # =========================
    # ARM (test mode always ON)
    # =========================
    arm_value = 2000

    # =========================
    # Send
    # =========================
    channels = [
        roll_cmd,
        pitch_cmd,
        throttle,
        yaw_cmd,
        1000, 1000, 1000,
        arm_value
    ]

    line = ",".join(map(str, channels)) + "\n"

    if serial_enabled:
        ser.write(line.encode("ascii"))
    else:
        print("SEND:", line.strip())

    print(f"R:{roll_cmd} P:{pitch_cmd} T:{throttle} ARM:{arm_value}")

    # =========================
    # Timing (50 Hz)
    # =========================
    elapsed_loop = time.time() - loop_start
    sleep_time = CONTROL_DT - elapsed_loop

    if sleep_time > 0:
        time.sleep(sleep_time)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# =========================
# Cleanup
# =========================
cap.release()
cv2.destroyAllWindows()
