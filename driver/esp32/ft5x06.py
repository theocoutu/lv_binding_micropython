import touch_base as _touch_base
from micropython import const

FT5x06_I2C_SLAVE_ADDR = const(0x38)


FT5x06_DEV_MODE_REG = const(0x00)

FT5x06_DEV_MODE_WORKING = const(0x00)
FT5x06_GEST_ID_REG = const(0x01)
FT5x06_MAX_TOUCH_PNTS = const(0x05)

FT5x06_MSB_MASK = const(0x0F)
FT5x06_LSB_MASK = const(0xFF)

FT5x06_TD_STAT_REG = const(0x02)
FT5x06_TD_STAT_MASK = const(0x0F)

FT5x06_P1_XH_REG = const(0x03)
FT5x06_P1_XL_REG = const(0x04)
FT5x06_P1_YH_REG = const(0x05)
FT5x06_P1_YL_REG = const(0x06)
FT5x06_P1_WEIGHT_REG = const(0x07)
FT5x06_TOUCH_WEIGHT_MASK = const(0xFF)


FT5x06_P2_XH_REG = const(0x09)
FT5x06_P2_XL_REG = const(0x0A)
FT5x06_P2_YH_REG = const(0x0B)
FT5x06_P2_YL_REG = const(0x0C)
FT5x06_P2_WEIGHT_REG = const(0x0D)
FT5x06_P2_MISC_REG = const(0x0E)

FT5x06_TH_GROUP_REG = const(0x80)
FT5x06_THRESHOLD_MASK = const(0xFF)

FT5x06_ID_G_THPEAK = const(0x81)
FT5x06_ID_G_THCAL = const(0x82)
FT5x06_ID_G_THWATER = const(0x83)
FT5x06_ID_G_THTEMP = const(0x84)


FT5x06_TH_DIFF_REG = const(0x85)
FT5x06_CTRL_REG = const(0x86)

FT5x06_TIME_ENTER_MONITOR_REG = const(0x87)
FT5x06_PERIOD_ACTIVE_REG = const(0x88)
FT5x06_PERIOD_MONITOR_REG = const(0x89)
FT5x06_ID_G_AUTO_CLB_MODE = const(0xA0)
FT5x06_LIB_VER_H_REG = const(0xA1)
FT5x06_LIB_VER_L_REG = const(0xA2)
FT5x06_ID_G_CIPHER = const(0xA3)
FT5x06_ID_G_MODE = const(0xA4)
FT5x06_POWER_MODE_REG = const(0xA5)
FT5x06_FIRMWARE_ID_REG = const(0xA6)
FT5x06_ID_G_STATE = const(0xA7)
FT5x06_PANEL_ID_REG = const(0xA8)
FT5x06_ID_G_ERR = const(0xA9)
FT5x06_RELEASECODE_REG = const(0xAF)
FT5x06_CHIPSELECT_REG = const(0x36)


class FT5x06(_touch_base.TouchBase):

    def __init__(self, i2c):
        self._i2c = i2c
        self.data_buf = bytearray(5)
        self.mv = memoryview(self.data_buf)
        super().__init__()

    def init(self):
        data_buf = self._read(FT5x06_PANEL_ID_REG)
        print("Device ID: 0x%02x" % data_buf)

        data_buf = self._read(FT5x06_CHIPSELECT_REG)
        print("Chip ID: 0x%02x" % data_buf)

        data_buf = self._read(FT5x06_DEV_MODE_REG)
        print("Device mode: 0x%02x" % data_buf)

        data_buf = self._read(FT5x06_FIRMWARE_ID_REG)
        print("Firmware ID: 0x%02x" % data_buf)

        data_buf = self._read(FT5x06_RELEASECODE_REG)
        print("Release code: 0x%02x" % data_buf)

    def _read(self, register_addr):
        self._i2c.readfrom_mem_into(FT5x06_I2C_SLAVE_ADDR, register_addr, self.mv[:1])
        return self.data_buf[0]

    def _get_coords(self):
        self._i2c.readfrom_mem_into(
            FT5x06_I2C_SLAVE_ADDR,
            FT5x06_TD_STAT_REG,
            self.mv
        )

        touch_pnt_cnt = self.data_buf[0]

        if touch_pnt_cnt != 1:
            return

        x = (
            ((self.data_buf[1] & FT5x06_MSB_MASK) << 8) |
            (self.data_buf[2] & FT5x06_LSB_MASK)
        )
        y = (
            ((self.data_buf[3] & FT5x06_MSB_MASK) << 8) |
            (self.data_buf[4] & FT5x06_LSB_MASK)
        )

        return x, y
