class Stabilizer:
    def __init__(self):
        self.kp = 5  # коэффициент (

    def stabilize(self, roll, pitch):

        roll_error = -roll
        pitch_error = -pitch

        roll_cmd = 1500 + int(self.kp * roll_error)
        pitch_cmd = 1500 + int(self.kp * pitch_error)

        # ограничение
        roll_cmd = max(1000, min(2000, roll_cmd))
        pitch_cmd = max(1000, min(2000, pitch_cmd))

        return roll_cmd, pitch_cmd