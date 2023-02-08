import touch_base as _touch_base
import machine
from micropython import const

Z_VALUE_1 = const(0xB1)
Z_VALUE_2 = const(0xC1)
Y_POSITION = const(0x90) # NOTE: XPT2046 datasheet has X and Y reversed!
X_POSITION = const(0xD0)  # NOTE: XPT2046 datasheet has X and Y reversed!

XPT2046_ADC_LIMIT = const(4096)
spi = machine.SPI


class XPT2046(_touch_base.TouchBase):

    def __init__(self, spi, cs, touch_threshold):
        self._spi = spi
        self.touch_threshold = touch_threshold

        if not isinstance(cs, machine.Pin):
            cs = machine.Pin(cs, machine.Pin.OUT)

        self.cs = cs

        self.write_buf = bytearray(1)
        self.read_buf = bytearray(2)
        self.write_mv = memoryview(self.write_buf)
        self.read_mv = memoryview(self.read_buf)

        super().__init__()

    def _read(self, reg):
        self.write_buf[0] = reg
        self._spi.write_readinto(self.write_mv, self.read_mv)
        return self.read_buf[0] << 8 | self.read_buf[1]

    def _get_coords(self):
        x = 0
        y = 0

        z1 = self._read(Z_VALUE_1)
        z2 = self._read(Z_VALUE_2)

        z = (z1 >> 3) + (XPT2046_ADC_LIMIT - (z2 >> 3))

        if z < self.touch_threshold:
            return

        self._read(X_POSITION)

        for _ in range(3):
            x_temp = self._read(X_POSITION)
            y_temp = self._read(Y_POSITION)

            x_temp >>= 3
            y_temp >>= 3

            x += (x_temp / XPT2046_ADC_LIMIT) * self.width
            y += (y_temp / XPT2046_ADC_LIMIT) * self.height

        x /= 3
        y /= 3

        return x, y
