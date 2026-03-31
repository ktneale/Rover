import time
import serial


class IMUGyro:
    def __init__(
        self,
        port="/dev/ttyUSB0",
        baud=9600,
        deadband_dps=1.0,
        alpha=0.8,  # smoothing
    ):
        self.ser = serial.Serial(port, baud, timeout=1)

        self.deadband = deadband_dps
        self.alpha = alpha

        self.bias = 0.0
        self.gz_filtered = 0.0

        self.heading_deg = 0.0
        self.last_t = time.time()

    def to_signed(self, val):
        return val - 65536 if val > 32767 else val

    def read_gz(self):
        while True:
            b = self.ser.read(1)
            if b != b"\x55":
                continue

            packet_type = self.ser.read(1)
            data = self.ser.read(8)
            checksum = self.ser.read(1)

            if len(packet_type) < 1 or len(data) < 8:
                return None

            if packet_type == b"\x52":  # gyro
                gz_raw = int.from_bytes(data[4:6], "little")
                gz = self.to_signed(gz_raw) / 32768.0 * 2000.0
                return gz

    def calibrate_bias(self, seconds=2.0):
        print(f"Calibrating gyro bias ({seconds}s)... keep still")
        samples = []
        start = time.time()

        while time.time() - start < seconds:
            gz = self.read_gz()
            if gz is not None:
                samples.append(gz)

        if not samples:
            raise RuntimeError("No gyro samples for bias")

        self.bias = sum(samples) / len(samples)
        print(f"Bias = {self.bias:.3f} deg/s")

    def update(self):
        gz = self.read_gz()
        if gz is None:
            return None

        now = time.time()
        dt = now - self.last_t
        self.last_t = now

        # clamp dt to avoid spikes
        if dt > 0.05:
            dt = 0.05

        # bias correction
        gz_corr = gz - self.bias

        # deadband
        if abs(gz_corr) < self.deadband:
            gz_corr = 0.0

        # smoothing
        self.gz_filtered = (
            self.alpha * self.gz_filtered
            + (1 - self.alpha) * gz_corr
        )

        # integration
        dtheta = self.gz_filtered * dt
        self.heading_deg += dtheta

        return {
            "gz_deg_s": self.gz_filtered,
            "dtheta_deg": dtheta,
            "heading_deg": self.heading_deg,
            "dt": dt,
        }
