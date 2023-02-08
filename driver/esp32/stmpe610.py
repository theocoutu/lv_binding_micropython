import time
import machine
from micropython import const
import touch_base as _touch_base


STMPE610_REG_CHIP_ID = const(0x00)
STMPE610_REG_ID_VER = const(0x02)
STMPE610_REG_SYS_CTRL1 = const(0x03)
STMPE610_REG_SYS_CTRL2 = const(0x04)
STMPE610_REG_SPI_CFG = const(0x08)
STMPE610_REG_INT_CTRL = const(0x09)
STMPE610_REG_INT_EN = const(0x0A)
STMPE610_REG_INT_STA = const(0x0B)
STMPE610_REG_ADC_CTRL1 = const(0x20)
STMPE610_REG_ADC_CTRL2 = const(0x21)
STMPE610_REG_TSC_CTRL = const(0x40)
STMPE610_REG_TSC_CFG = const(0x41)
STMPE610_REG_TSC_FRACTION_Z = const(0x56)
STMPE610_REG_TSC_DATA = const(0x57)
STMPE610_REG_TSC_I_DRIVE = const(0x58)
STMPE610_REG_FIFO_TH = const(0x4A)
STMPE610_REG_FIFO_STA = const(0x4B)
STMPE610_REG_FIFO_SIZE = const(0x4C)


STMPE610_REG_TSC_CTRL_EN = const(0x01)
STMPE610_REG_TSC_CTRL_XYZ = const(0x00)
STMPE610_REG_TSC_CTRL_XY = const(0x02)

STMPE610_REG_ADC_CTRL1_10BIT = const(0x00)

STMPE610_REG_ADC_CTRL2_6_5MHZ = const(0x02)

STMPE610_REG_TSC_CFG_4SAMPLE = const(0x80)
STMPE610_REG_TSC_CFG_DELAY_1MS = const(0x20)
STMPE610_REG_TSC_CFG_SETTLE_5MS = const(0x04)

STMPE610_REG_FIFO_STA_RESET = const(0x01)
STMPE610_REG_FIFO_STA_EMPTY = const(0x20)

STMPE610_REG_TSC_I_DRIVE_50MA = const(0x01)

STMPE610_REG_INT_CTRL_POL_LOW = const(0x00)
STMPE610_REG_INT_CTRL_EDGE = const(0x02)
STMPE610_REG_INT_CTRL_ENABLE = const(0x01)


class STMPE610(_touch_base.TouchBase):

    def __init__(
        self, host, miso, mosi, sck, cs, reset=None,
        swap_xy=True, invert_x=False, invert_y=False
    ):
        miso = machine.Pin(miso)
        mosi = machine.Pin(mosi)
        sck = machine.Pin(sck)
        self._spi = machine.SPI(host, miso=miso, mosi=mosi, sck=sck)
        self.cs = machine.Pin(cs, machine.Pin.OUT)

        if reset is None:
            self.rst = None
        else:
            self.rst = machine.Pin(reset, machine.Pin.OUT)

        self.buf = bytearray(1)
        self.buf1 = bytearray(1)
        self.buf2 = bytearray(2)
        super().__init__(swap_xy=swap_xy, invert_x=invert_x, invert_y=invert_y)

    def init(self):
        if self.rst is not None:
            self.rst.value(1)
            time.sleep_ms(10)
            self.rst.value(0)
            time.sleep_ms(10)

        self._read(STMPE610_REG_CHIP_ID, self.buf1)
        chip_id1 = self.buf1[0]
        self._read(STMPE610_REG_CHIP_ID + 1, self.buf1)
        chip_id2 = self.buf1[0]

        self._read(STMPE610_REG_ID_VER, self.buf1)
        version = self.buf1[0]

        chip_id = (chip_id1 << 8) | chip_id2

        print("TouchPad ID: 0x%04x" % (chip_id,))
        print("TouchPad Ver: 0x%02x", (version,))

        if chip_id != 0x0811:
            raise RuntimeError('Touch interface is not supported.')

        self._write(STMPE610_REG_SYS_CTRL1, 0x02)

        self._write(STMPE610_REG_SYS_CTRL2, 0x0)

        self._write(
            STMPE610_REG_TSC_CTRL,
            STMPE610_REG_TSC_CTRL_XYZ | STMPE610_REG_TSC_CTRL_EN
            )
        self._write(STMPE610_REG_INT_EN, 0x01)

        self._write(
            STMPE610_REG_ADC_CTRL1,
            STMPE610_REG_ADC_CTRL1_10BIT | (0x6 << 4)
            )
        self._write(STMPE610_REG_ADC_CTRL2, STMPE610_REG_ADC_CTRL2_6_5MHZ)

        self._write(
            STMPE610_REG_TSC_CFG,
            STMPE610_REG_TSC_CFG_4SAMPLE | STMPE610_REG_TSC_CFG_DELAY_1MS | STMPE610_REG_TSC_CFG_SETTLE_5MS
            )
        self._write(STMPE610_REG_TSC_FRACTION_Z, 0x6)
        self._write(STMPE610_REG_FIFO_TH, 1)

        self._write(STMPE610_REG_FIFO_STA, STMPE610_REG_FIFO_STA_RESET)
        self._write(STMPE610_REG_FIFO_STA, 0)

        self._write(STMPE610_REG_TSC_I_DRIVE, STMPE610_REG_TSC_I_DRIVE_50MA)

        self._write(STMPE610_REG_INT_STA, 0xFF)
        self._write(
            STMPE610_REG_INT_CTRL,
            STMPE610_REG_INT_CTRL_POL_LOW | STMPE610_REG_INT_CTRL_EDGE | STMPE610_REG_INT_CTRL_ENABLE
        )
    
    def _get_coords(self):
        self._read(STMPE610_REG_FIFO_STA, self.buf1)
        fifo_status = self.buf1[0]

        if fifo_status & STMPE610_REG_FIFO_STA_EMPTY:
            return

        self._read(STMPE610_REG_FIFO_SIZE, self.buf1)
        cnt = self.buf1[0]

        if cnt == 0:
            return

        x = 0
        y = 0

        for i in range(cnt):
            self._read(STMPE610_REG_TSC_DATA, self.buf1)
            buf1 = self.buf1[0]
            self._read(STMPE610_REG_TSC_DATA, self.buf1)
            buf2 = self.buf1[0]
            self._read(STMPE610_REG_TSC_DATA, self.buf1)
            buf3 = self.buf1[0]
            self._read(STMPE610_REG_TSC_DATA, self.buf1)
            z = self.buf1[0]

            if z:
                x += (buf1 << 4) | ((buf2 >> 4) & 0x0F)
                y += ((buf2 & 0x0F) << 8) | buf3

        self._write(STMPE610_REG_FIFO_STA, STMPE610_REG_FIFO_STA_RESET)
        self._write(STMPE610_REG_FIFO_STA, 0)

        self._write(STMPE610_REG_INT_STA, 0xFF)

        if x == 0 or y == 0:
            return

        x = self._data_convert(int(x / cnt), 150, 3800, 0, self.width)
        y = self._data_convert(int(y / cnt), 150, 3800, 0, self.height)

        return x, y

    def _read(self, reg, buf):
        self.cs.value(1)
        self.buf[0] = reg
        self._spi.write_readinto(self.buf, buf)
        self.cs.value(0)

    def _write(self, reg, data):
        self.cs.value(1)
        self.buf2[0] = reg
        self.buf2[1] = data
        self._spi.write(self.buf2)
        self.cs.value(0)

    @staticmethod
    def _data_convert(x, in_min, in_max, out_min, out_max):
        if x < in_min:
            x = in_min
    
        if x > in_max:
            x = in_max
        
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


