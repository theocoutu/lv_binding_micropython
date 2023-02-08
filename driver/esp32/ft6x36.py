import touch_base as _touch_base
from micropython import const

FT6x36_I2C_SLAVE_ADDR = const(0x38)

# ** Maximum border values of the touchscreen pad that the chip can handle **
FT6X36_MAX_WIDTH = const(800)
FT6X36_MAX_HEIGHT = const(480)

# Max detectable simultaneous touch points
FT6X36_MAX_TOUCH_PNTS = const(2)

# Register of the current mode
FT6X36_DEV_MODE_REG = const(0x00)

# ** Possible modes as of FT6X36_DEV_MODE_REG **
FT6X36_DEV_MODE_WORKING = const(0x00)
FT6X36_DEV_MODE_FACTORY = const(0x04)

FT6X36_DEV_MODE_MASK = const(0x70)
FT6X36_DEV_MODE_SHIFT = const(4)

# Gesture ID register
FT6X36_GEST_ID_REG = const(0x01)

# ** Possible values returned by FT6X36_GEST_ID_REG **
FT6X36_GEST_ID_NO_GESTURE = const(0x00)
FT6X36_GEST_ID_MOVE_UP = const(0x10)
FT6X36_GEST_ID_MOVE_RIGHT = const(0x14)
FT6X36_GEST_ID_MOVE_DOWN = const(0x18)
FT6X36_GEST_ID_MOVE_LEFT = const(0x1C)
FT6X36_GEST_ID_ZOOM_IN = const(0x48)
FT6X36_GEST_ID_ZOOM_OUT = const(0x49)

# Status register: stores number of active touch points (0, 1, 2)
FT6X36_TD_STAT_REG = const(0x02)
FT6X36_TD_STAT_MASK = const(0x0F)
FT6X36_TD_STAT_SHIFT = const(0x00)

# ** Touch events **
FT6X36_TOUCH_EVT_FLAG_PRESS_DOWN = const(0x00)
FT6X36_TOUCH_EVT_FLAG_LIFT_UP = const(0x01)
FT6X36_TOUCH_EVT_FLAG_CONTACT = const(0x02)
FT6X36_TOUCH_EVT_FLAG_NO_EVENT = const(0x03)

FT6X36_TOUCH_EVT_FLAG_SHIFT = const(6)
FT6X36_TOUCH_EVT_FLAG_MASK = const(3 << FT6X36_TOUCH_EVT_FLAG_SHIFT)

FT6X36_MSB_MASK = const(0x0F)
FT6X36_MSB_SHIFT = const(0)
FT6X36_LSB_MASK = const(0xFF)
FT6X36_LSB_SHIFT = const(0)

FT6X36_P1_XH_REG = const(0x03)
FT6X36_P1_XL_REG = const(0x04)
FT6X36_P1_YH_REG = const(0x05)
FT6X36_P1_YL_REG = const(0x06)

# ** Register reporting touch pressure - read only **
FT6X36_P1_WEIGHT_REG = const(0x07)
FT6X36_TOUCH_WEIGHT_MASK = const(0xFF)
FT6X36_TOUCH_WEIGHT_SHIFT = const(0)

# Touch area register
FT6X36_P1_MISC_REG = const(0x08)

# ** Values related to FT6X36_Pn_MISC_REG **
FT6X36_TOUCH_AREA_MASK = const(0x04 << 4)
FT6X36_TOUCH_AREA_SHIFT = const(0x04)

FT6X36_P2_XH_REG = const(0x09)
FT6X36_P2_XL_REG = const(0x0A)
FT6X36_P2_YH_REG = const(0x0B)
FT6X36_P2_YL_REG = const(0x0C)
FT6X36_P2_WEIGHT_REG = const(0x0D)
FT6X36_P2_MISC_REG = const(0x0E)

# ** Threshold for touch detection **
FT6X36_TH_GROUP_REG = const(0x80)

# ** Values FT6X36_TH_GROUP_REG : threshold related **
FT6X36_THRESHOLD_MASK = const(0xFF)
FT6X36_THRESHOLD_SHIFT = const(0)


# ** Filter function coefficients **
FT6X36_TH_DIFF_REG = const(0x85)

# Control register
FT6X36_CTRL_REG = const(0x86)

# Will keep the Active mode when there is no touching
FT6X36_CTRL_KEEP_ACTIVE_MODE = const(0x00)

# Switching from Active mode to Monitor mode
# automatically when there is no touching
FT6X36_CTRL_KEEP_AUTO_SWITCH_MONITOR_MODE = const(0x01)

# The time period of switching from Active mode to
# Monitor mode when there is no touching
FT6X36_TIME_ENTER_MONITOR_REG = const(0x87)

# Report rate in Active mode
FT6X36_PERIOD_ACTIVE_REG = const(0x88)
# Report rate in Monitor mode
FT6X36_PERIOD_MONITOR_REG = const(0x89)

# The value of the minimum allowed angle while Rotating gesture mode
FT6X36_RADIAN_VALUE_REG = const(0x91)

# Maximum offset while Moving Left and Moving Right gesture
FT6X36_OFFSET_LEFT_RIGHT_REG = const(0x92)
# Maximum offset while Moving Up and Moving Down gesture
FT6X36_OFFSET_UP_DOWN_REG = const(0x93)

# Minimum distance while Moving Left and Moving Right gesture
FT6X36_DISTANCE_LEFT_RIGHT_REG = const(0x94)
# Minimum distance while Moving Up and Moving Down gesture
FT6X36_DISTANCE_UP_DOWN_REG = const(0x95)

# High 8-bit of LIB Version info
FT6X36_LIB_VER_H_REG = const(0xA1)
# Low 8-bit of LIB Version info
FT6X36_LIB_VER_L_REG = const(0xA2)

# 0x36 for ft6236; 0x06 for ft6206
FT6X36_CHIPSELECT_REG = const(0x36)

FT6X36_POWER_MODE_REG = const(0xA5)
FT6X36_FIRMWARE_ID_REG = const(0xA6)
FT6X36_RELEASECODE_REG = const(0xAF)
FT6X36_PANEL_ID_REG = const(0xA8)
FT6X36_OPMODE_REG = const(0xBC)


class FT6x36(_touch_base.TouchBase):

    def __init__(self, i2c):
        self._i2c = i2c
        self.data_buf = bytearray(5)
        self.mv = memoryview(self.data_buf)
        super().__init__()

    def init(self):
        data_buf = self._read(FT6X36_PANEL_ID_REG)
        print("Device ID: 0x%02x" % data_buf)

        data_buf = self._read(FT6X36_CHIPSELECT_REG)
        print("Chip ID: 0x%02x" % data_buf)

        data_buf = self._read(FT6X36_DEV_MODE_REG)
        print("Device mode: 0x%02x" % data_buf)

        data_buf = self._read(FT6X36_FIRMWARE_ID_REG)
        print("Firmware ID: 0x%02x" % data_buf)

        data_buf = self._read(FT6X36_RELEASECODE_REG)
        print("Release code: 0x%02x" % data_buf)

    def _read(self, register_addr):
        self._i2c.readfrom_mem_into(FT6x36_I2C_SLAVE_ADDR, register_addr, self.mv[:1])
        return self.data_buf[0]

    def _get_coords(self):
        self._i2c.readfrom_mem_into(
            FT6x36_I2C_SLAVE_ADDR,
            FT6X36_TD_STAT_REG,
            self.mv
        )

        touch_pnt_cnt = self.data_buf[0]

        if touch_pnt_cnt != 1:
            return

        x = (
            ((self.data_buf[1] & FT6X36_MSB_MASK) << 8) |
            (self.data_buf[2] & FT6X36_LSB_MASK)
        )
        y = (
            ((self.data_buf[3] & FT6X36_MSB_MASK) << 8) |
            (self.data_buf[4] & FT6X36_LSB_MASK)
        )

        return x, y
