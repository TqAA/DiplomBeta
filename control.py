import time


class PID:
    def __init__(self, kp, ki, kd, output_limit=400, integral_limit=100):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()

    def compute(self, error):
        now = time.time()
        dt  = now - self._prev_time

        if dt <= 0 or dt > 0.5:
            self._prev_time = now
            return 0

        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                               min(self.integral_limit, self._integral))

        derivative = (error - self._prev_error) / dt

        output = (self.kp * error +
                  self.ki * self._integral +
                  self.kd * derivative)

        output = max(-self.output_limit, min(self.output_limit, output))

        self._prev_error = error
        self._prev_time  = now

        return output

    def reset(self):
        self._integral   = 0.0
        self._prev_error = 0.0
        self._prev_time  = time.time()


class Stabilizer:
    def __init__(self):
        self.roll_pid  = PID(kp=4.0, ki=0.05, kd=1.2)
        self.pitch_pid = PID(kp=4.0, ki=0.05, kd=1.2)

    def stabilize(self, roll_error, pitch_error):
        roll_out  = self.roll_pid.compute(roll_error)
        pitch_out = self.pitch_pid.compute(pitch_error)

        roll_cmd  = int(1500 - roll_out)
        pitch_cmd = int(1500 + pitch_out)

        roll_cmd  = max(1000, min(2000, roll_cmd))
        pitch_cmd = max(1000, min(2000, pitch_cmd))

        return roll_cmd, pitch_cmd

    def reset(self):
        self.roll_pid.reset()
        self.pitch_pid.reset()