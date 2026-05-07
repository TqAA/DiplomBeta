import socket
import math
import time
import threading


class IMUReceiver:
    def __init__(self, port=5010):
        self.roll = 0.0
        self.pitch = 0.0
        self._lock = threading.Lock()
        self._last_received = 0.0
        self.prev_time = time.time()

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", port))
            self.sock.settimeout(1.0)
            self.enabled = True
            print("IMU OK")
            self._thread = threading.Thread(target=self._recv_loop, daemon=True)
            self._thread.start()
        except Exception as e:
            print("IMU DISABLED:", e)
            self.enabled = False

    def _recv_loop(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
            except socket.timeout:
                continue
            except Exception as e:
                print("IMU ERROR:", e)
                continue

            try:
                values = list(map(int, data.decode().strip().split(",")))
                if len(values) != 6:
                    continue
            except:
                continue

            ax, ay, az, gx, gy, gz = values
            ax /= 16384.0
            ay /= 16384.0
            az /= 16384.0
            gx /= 131.0
            gy /= 131.0

            current_time = time.time()
            dt = current_time - self.prev_time
            self.prev_time = current_time

            if dt <= 0 or dt > 0.5:
                continue

            roll_acc  = math.atan2(ay, az) * 180 / math.pi
            pitch_acc = math.atan2(-ax, (ay ** 2 + az ** 2) ** 0.5) * 180 / math.pi

            roll  = 0.96 * (self.roll  + gx * dt) + 0.04 * roll_acc
            pitch = 0.96 * (self.pitch + gy * dt) + 0.04 * pitch_acc

            with self._lock:
                self.roll = roll
                self.pitch = pitch
                self._last_received = current_time

    def get_angles(self):
        with self._lock:
            return self.roll, self.pitch

    def is_alive(self, timeout=0.5):
        with self._lock:
            return (time.time() - self._last_received) < timeout