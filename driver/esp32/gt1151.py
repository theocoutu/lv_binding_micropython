from micropython import const
import touch_base as _touch_base


GT1151_I2C_SLAVE_ADDR = const(0x14)

# Write only registers
GT1151_REG_COMMAND = const(0x8040)
GT1151_REG_ESD_CHECK = const(0x8041)
GT1151_REG_PROXIMITY_EN = const(0x8042)

# Read/write registers
# The version number of the configuration file
GT1151_REG_CONFIG_DATA = const(0x8047)
# X output maximum value (LSB 2 bytes)
GT1151_REG_MAX_X = const(0x8048)
# Y output maximum value (LSB 2 bytes)
GT1151_REG_MAX_Y = const(0x804A)
# Maximum number of output contacts: 1~5 (4 bit value 3:0, 7:4 is reserved)
GT1151_REG_MAX_TOUCH = const(0x804C)

# Module switch 1
# 7:6 Reserved, 5:4 Stretch rank, 3 X2Y, 2 SITO
# (Single sided ITO touch screen), 1:0 INT Trigger mode
GT1151_REG_MOD_SW1 = const(0x804D)
# Module switch 2
# 7:1 Reserved, 0 Touch key */
GT1151_REG_MOD_SW2 = const(0x804E)

# Number of debuffs fingers press/release
GT1151_REG_SHAKE_CNT = const(0x804F)

# X threshold
GT1151_REG_X_THRESHOLD = const(0x8057)

# Configuration update fresh
GT1151_REG_CONFIG_FRESH = const(0x8100)

# ReadOnly registers (device and coordinates info)
# Product ID (LSB 4 bytes, GT11510: 0x06 0x00 0x00 0x09)
GT1151_REG_ID = const(0x8140)
# Firmware version (LSB 2 bytes)
GT1151_REG_FW_VER = const(0x8144)

# Current output X resolution (LSB 2 bytes)
GT1151_READ_X_RES = const(0x8146)
# Current output Y resolution (LSB 2 bytes)
GT1151_READ_Y_RES = const(0x8148)
# Module vendor ID
GT1151_READ_VENDOR_ID = const(0x814A)

GT1151_READ_COORD_ADDR = const(0x814E)

GT1151_POINT1_X_ADDR = const(0x8150)
GT1151_POINT1_Y_ADDR = const(0x8152)

# Commands for REG_COMMAND
# 0: read coordinate state
GT1151_CMD_READ = const(0x00)
# 1: difference value original value
GT1151_CMD_DIFFVAL = const(0x01)
# 2: software reset
GT1151_CMD_SOFTRESET = const(0x02)
# 3: Baseline update
GT1151_CMD_BASEUPDATE = const(0x03)
# 4: Benchmark calibration
GT1151_CMD_CALIBRATE = const(0x04)
# 5: Off screen (send other invalid)
GT1151_CMD_SCREEN_OFF = const(0x05)


class GT1151(_touch_base.TouchBase):

    def __init__(self, i2c):
        self.i2c = i2c
        self.data_buf = bytearray(6)
        self.mv = memoryview(self.data_buf)

        super().__init__()

    def init(self):
        # Get product ID
        print('Touch product id:', self.product_id)
        print('Touch firmware:', self.firmware)

    def _get_coords(self):
        status = self.status

        if (status & 0x80) != 0:
            coord_num = status & 0x0F
            if coord_num != 0:
                coord_x = []
                coord_y = []
                for i in range(coord_num):
                    self.data_buf[0] = (((GT1151_POINT1_X_ADDR + (i * 8)) & 0xFF00) >> 8)
                    self.data_buf[1] = (GT1151_POINT1_X_ADDR + (i * 8)) & 0xFF
                    self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:2])
                    self.i2c.readfrom_into(GT1151_I2C_SLAVE_ADDR, self.mv[:6])

                    x = self.data_buf[0]
                    y = self.data_buf[2]
                    x |= (self.data_buf[1] << 8)
                    y |= (self.data_buf[3] << 8)

                    coord_x.append(x)
                    coord_y.append(y)

                x = int(sum(coord_x) / len(coord_x))
                y = int(sum(coord_y) / len(coord_y))

                self.status = 0
                return x, y

            self.status = 0

    def set_command_register(self, command):
        self.data_buf[0] = (GT1151_REG_COMMAND & 0xFF00) >> 8
        self.data_buf[1] = GT1151_REG_COMMAND & 0xFF
        self.data_buf[2] = command

        self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:3])

    @property
    def product_id(self):
        self.data_buf[0] = (GT1151_REG_ID & 0xFF00) >> 8
        self.data_buf[1] = GT1151_REG_ID & 0xFF
        self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:2])
        self.i2c.readfrom_into(GT1151_I2C_SLAVE_ADDR, self.mv[:4])

        buf = bytearray([self.data_buf[3], self.data_buf[2], self.data_buf[1], self.data_buf[0]])
        return buf.decode('utf-8')

    @property
    def firmware(self):
        self.data_buf[0] = (GT1151_REG_FW_VER & 0xFF00) >> 8
        self.data_buf[1] = GT1151_REG_FW_VER & 0xFF

        self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:2])
        self.i2c.readfrom_into(GT1151_I2C_SLAVE_ADDR, self.mv[:2])

        return self.data_buf[1] << 8 | self.data_buf[0]

    @property
    def status(self):
        self.data_buf[0] = (GT1151_READ_COORD_ADDR & 0xFF00) >> 8
        self.data_buf[1] = GT1151_READ_COORD_ADDR & 0xFF

        self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:2])

        self.i2c.readfrom_into(GT1151_I2C_SLAVE_ADDR, self.mv[:1])
        return self.data_buf[0]
        
    @status.setter
    def status(self, value):
        self.data_buf[0] = (GT1151_READ_COORD_ADDR & 0xFF00) >> 8
        self.data_buf[1] = GT1151_READ_COORD_ADDR & 0xFF
        self.data_buf[2] = value
        self.i2c.writeto(GT1151_I2C_SLAVE_ADDR, self.mv[:3])
