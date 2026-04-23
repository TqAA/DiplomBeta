import socket
import math
import time

class IMUReceiver:
    def __init__(self, port=5010):
        self.roll = 0
        self.pitch = 0
        self.prev_time = time.time()

        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(("0.0.0.0", port))
            self.sock.settimeout(0.09)
            self.enabled = True
            print("IMU OK")
        except Exception as e:
            print("IMU DISABLED:", e)
            self.enabled = False

    def get_angles(self):
        if not self.enabled:
            return self.roll, self.pitch

        try:
            data, _ = self.sock.recvfrom(1024)
        except socket.timeout:
            return self.roll, self.pitch
        except Exception as e:
            print("IMU ERROR:", e)
            return self.roll, self.pitch

        try:
            values = list(map(int, data.decode().strip().split(",")))
            if len(values) != 6:
                return self.roll, self.pitch
        except:
            return self.roll, self.pitch

        ax, ay, az, gx, gy, gz = values

        ax /= 16384.0
        ay /= 16384.0
        az /= 16384.0

        gx /= 131.0
        gy /= 131.0

        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        roll_acc = math.atan2(ay, az) * 180 / math.pi
        pitch_acc = math.atan2(-ax, (ay ** 2 + az ** 2) ** 0.5) * 180 / math.pi

        self.roll = 0.90 * (self.roll + gx * dt) + 0.1 * roll_acc
        self.pitch = 0.90 * (self.pitch + gy * dt) + 0.1 * pitch_acc

        return self.roll, self.pitch