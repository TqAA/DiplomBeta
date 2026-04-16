import socket
import math
import time

class IMUReceiver:
    def __init__(self, port=5010):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", port))

        self.roll = 0
        self.pitch = 0
        self.prev_time = time.time()

    def get_angles(self):
        data, _ = self.sock.recvfrom(1024)
        values = list(map(int, data.decode().split(",")))

        ax, ay, az, gx, gy, gz = values

        # перевод
        ax /= 16384.0
        ay /= 16384.0
        az /= 16384.0

        gx /= 131.0
        gy /= 131.0

        # время
        current_time = time.time()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        # углы акселерометра
        roll_acc = math.atan2(ay, az) * 180 / math.pi
        pitch_acc = math.atan2(-ax, (ay**2 + az**2)**0.5) * 180 / math.pi

        # фильтр
        self.roll = 0.98 * (self.roll + gx * dt) + 0.02 * roll_acc
        self.pitch = 0.98 * (self.pitch + gy * dt) + 0.02 * pitch_acc

        return self.roll, self.pitch