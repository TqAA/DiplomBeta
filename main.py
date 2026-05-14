import cv2
import serial
import time


from rec import IMUReceiver
from control import Stabilizer


# =========================
# Serial
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
imu         = IMUReceiver()
stabilizer  = Stabilizer()
cap         = cv2.VideoCapture(0)
qr_detector = cv2.QRCodeDetector()


# =========================
# Config
# =========================
CONTROL_DT         = 0.02          # 50 Hz
BASE_THROTTLE      = 1350
RAMP_DURATION      = 5.0           # секунд набора газа
RAMP_START         = 1000
RAMP_END           = BASE_THROTTLE

QR_NORM_FACTOR     = 320.0         # половина ширины кадра (640px)
QR_ALPHA           = 0.4           # сглаживание смещения QR
QR_LOSS_TIMEOUT    = 5.0           # секунд до перехода в поиск

YAW_NEUTRAL        = 1500
YAW_SEARCH_OFFSET  = 18            # отклонение при поиске (~медленное вращение)
YAW_THROTTLE_COMP  = 0.3           # компенсация газа при yaw-манёвре

TILT_THROTTLE_COMP = 0.3

QR_REDETECT_INTERVAL = 30          # переинициализировать трекер каждые N кадров


# =========================
# State
# =========================
# QR
target_dx   = 0.0
target_dy   = 0.0

# Last known position
last_qr_side      = "right"        # "left" / "right" — куда уходил QR
last_qr_seen_time = 0.0
qr_ever_seen      = False

# Tracker
tracker        = None
tracker_active = False
frame_counter  = 0

# Yaw
yaw_cmd = YAW_NEUTRAL

start_time = time.time()


# =========================
# Helpers
# =========================
def decode_qr(frame):
    """Декодирует QR через cv2. Возвращает (cx, cy, pts) или None."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data, bbox, _ = qr_detector.detectAndDecode(gray)

    if bbox is None:
        return None

    pts = [(int(p[0]), int(p[1])) for p in bbox.reshape(4, 2)]

    if len(pts) < 4:
        return None

    cx = int(sum(p[0] for p in pts) / 4)
    cy = int(sum(p[1] for p in pts) / 4)
    return cx, cy, pts


def init_tracker(frame, pts):
    """Инициализирует CSRT трекер по точкам QR."""
    x_t = min(p[0] for p in pts)
    y_t = min(p[1] for p in pts)
    w_t = max(p[0] for p in pts) - x_t
    h_t = max(p[1] for p in pts) - y_t
    t = cv2.TrackerCSRT_create()
    t.init(frame, (x_t, y_t, w_t, h_t))
    return t


def draw_qr(frame, cx, cy, pts, center_x, center_y):
    """Рисует рамку QR, центр и линию до центра кадра."""
    for i in range(len(pts)):
        pt1 = tuple(map(int, pts[i]))
        pt2 = tuple(map(int, pts[(i + 1) % len(pts)]))
        cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.circle(frame, (center_x, center_y), 5, (255, 0, 0), -1)
    cv2.line(frame, (center_x, center_y), (cx, cy), (255, 255, 0), 2)


def get_yaw_cmd(qr_detected, qr_ever_seen, last_qr_seen_time, last_qr_side):
    """
    Возвращает yaw команду:
    - QR виден          → нейтраль
    - QR пропал недавно → крутимся в сторону last_qr_side
    - QR не видели 5с+  → крутимся вправо
    """
    if qr_detected:
        return YAW_NEUTRAL

    time_since_qr = time.time() - last_qr_seen_time
    searching     = not qr_ever_seen or time_since_qr > QR_LOSS_TIMEOUT

    if searching:
        direction = 1                                        # вправо по умолчанию
    else:
        direction = 1 if last_qr_side == "right" else -1

    return YAW_NEUTRAL + direction * YAW_SEARCH_OFFSET


def calc_throttle(elapsed, imu_roll, imu_pitch, yaw_cmd, qr_detected, target_dy):
    """Считает throttle с учётом ramp, tilt и yaw компенсации."""
    # Ramp
    if elapsed < RAMP_DURATION:
        t    = elapsed / RAMP_DURATION
        base = int(RAMP_START + t * (RAMP_END - RAMP_START))
    else:
        base = BASE_THROTTLE

    # Tilt compensation
    tilt     = abs(imu_roll) + abs(imu_pitch)
    throttle = base + int(tilt * TILT_THROTTLE_COMP)

    # Yaw compensation — чем дальше от нейтрали, тем больше газа
    yaw_offset = abs(yaw_cmd - YAW_NEUTRAL)
    throttle  += int(yaw_offset * YAW_THROTTLE_COMP)

    # QR vertical correction (target_dy нормирован: -1.0 до 1.0)
    if qr_detected and abs(target_dy) > 0.06:
        throttle += int(target_dy * 96)                     # 0.3 * 320 = 96

    return max(1000, min(1700, throttle))


# =========================
# Main loop
# =========================
while True:
    loop_start = time.time()

    # --- IMU ---
    imu_roll, imu_pitch = imu.get_angles()

    # --- Camera ---
    ret, frame = cap.read()
    qr_detected = False

    if ret:
        center_x = frame.shape[1] // 2
        center_y = frame.shape[0] // 2

        frame_counter += 1
        qr_result = None

        # Пробуем трекер
        if tracker_active:
            success, bbox = tracker.update(frame)
            if success:
                x, y, w, h = [int(v) for v in bbox]
                qr_cx = x + w // 2
                qr_cy = y + h // 2
                pts   = [(x, y), (x + w, y), (x + w, y + h), (x, y + h)]
                qr_result = (qr_cx, qr_cy, pts)
            else:
                # трекер потерял объект
                tracker_active = False
                tracker        = None

        # Если трекер не активен или пора переинициализировать — запускаем QR детектор
        if not tracker_active or frame_counter % QR_REDETECT_INTERVAL == 0:
            qr_result_fresh = decode_qr(frame)
            if qr_result_fresh is not None:
                qr_cx, qr_cy, pts = qr_result_fresh
                tracker        = init_tracker(frame, pts)
                tracker_active = True
                qr_result      = qr_result_fresh

        if qr_result is not None:
            qr_cx, qr_cy, pts = qr_result
            qr_detected       = True
            qr_ever_seen      = True
            last_qr_seen_time = time.time()

            # Запоминаем сторону — куда смещён QR от центра
            raw_dx = qr_cx - center_x
            if raw_dx > 0:
                last_qr_side = "right"
            elif raw_dx < 0:
                last_qr_side = "left"

            # Сглаженное смещение (нормировано)
            norm_dx = raw_dx / QR_NORM_FACTOR
            norm_dy = (center_y - qr_cy) / QR_NORM_FACTOR

            target_dx = QR_ALPHA * target_dx + (1 - QR_ALPHA) * norm_dx
            target_dy = QR_ALPHA * target_dy + (1 - QR_ALPHA) * norm_dy

            draw_qr(frame, qr_cx, qr_cy, pts, center_x, center_y)

        # HUD
        status = "TRACKING" if qr_detected else ("SEARCHING" if not qr_ever_seen or
                  time.time() - last_qr_seen_time > QR_LOSS_TIMEOUT else "LAST KNOWN")
        cv2.putText(frame, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        cv2.imshow("frame", frame)

    # --- Target angles ---
    target_roll  = 0.0
    target_pitch = 0.0

    if qr_detected:
        target_roll  = -target_dx * 15.0   # нормировано: ±1.0 → ±15°
        target_pitch =  target_dy * 15.0

    # --- Stabilization ---
    roll_cmd, pitch_cmd = stabilizer.stabilize(
        imu_roll  - target_roll,
        imu_pitch - target_pitch
    )

    # --- Yaw ---
    yaw_cmd = get_yaw_cmd(qr_detected, qr_ever_seen, last_qr_seen_time, last_qr_side)

    # --- Throttle ---
    elapsed  = time.time() - start_time
    throttle = calc_throttle(elapsed, imu_roll, imu_pitch, yaw_cmd, qr_detected, target_dy)

    # --- Failsafe ---
    if not imu.is_alive():
        roll_cmd  = 1500
        pitch_cmd = 1500
        throttle  = 1350
        yaw_cmd = 1500
        print("FAILSAFE: IMU dead")

    # --- Send ---
    channels = [roll_cmd, pitch_cmd, throttle, yaw_cmd,
                1000, 1000, 1000, 2000]
    line = ",".join(map(str, channels)) + "\n"

    if serial_enabled:
        ser.write(line.encode("ascii"))
    else:
        print(f"SEND | R:{roll_cmd} P:{pitch_cmd} T:{throttle} Y:{yaw_cmd} | {status}")

    # --- Timing ---
    elapsed_loop = time.time() - loop_start
    sleep_time   = CONTROL_DT - elapsed_loop
    if sleep_time > 0:
        time.sleep(sleep_time)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


# =========================
# Cleanup
# =========================
cap.release()
cv2.destroyAllWindows()