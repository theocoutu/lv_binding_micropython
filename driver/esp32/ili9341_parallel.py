import espidf as _espidf  # NOQA
import ili9488_parallel as _ili9488_parallel
from micropython import const as _const
import time

from ili9488_parallel import (
    ILI9488_POWER_CTL_ONE,
    ILI9488_POWER_CTL_TWO,
    ILI9488_FRAME_RATE_NORMAL_CTL,
    ILI9488_POSITIVE_GAMMA_CTL,
    ILI9488_NEGATIVE_GAMMA_CTL,
    ILI9488_FUNCTION_CTL,
    ILI9488_ENTRY_MODE_CTL,
    ILI9488_INIT_LENGTH_MASK,
    ILI9488_COLOR_MODE_16BIT,
    _COLOR_SPACE_RGB
)

ILI9341_POWER_CONTROL_A = _const(0xCB)
ILI9341_POWER_CONTROL_B = _const(0xCF)
ILI9341_VCOM_CONTROL_1 = _const(0xC5)
ILI9341_VCOM_CONTROL_2 = _const(0xC7)

ILI9341_DRIVER_TIMING_CONTROL_A = _const(0xE8)
ILI9341_DRIVER_TIMING_CONTROL_B = _const(0xEA)
ILI9341_POWER_ON_SEQ = _const(0xED)
ILI9341_PUMP_RATIO_CONTROL = _const(0xF7)
ILI9341_ENABLE_3G = _const(0xF2)


class ILI9341Parallel(_ili9488_parallel.ILI9488Parallel):
    _color_space = _COLOR_SPACE_RGB

    def _init_cb(self, panel):
        init_commands = (
            (ILI9341_POWER_CONTROL_B, bytearray([0x00, 0xAA, 0XE0])),
            (ILI9341_POWER_ON_SEQ, bytearray([0x67, 0x03, 0X12, 0X81])),
            (ILI9341_DRIVER_TIMING_CONTROL_A, bytearray([0x8A, 0x01, 0x78])),
            (ILI9341_POWER_CONTROL_A, bytearray([0x39, 0x2C, 0x00, 0x34, 0x02])),
            (ILI9341_PUMP_RATIO_CONTROL, bytearray([0x20])),
            (ILI9341_PUMP_RATIO_CONTROL, bytearray([0x20])),
            (ILI9341_DRIVER_TIMING_CONTROL_B, bytearray([0x00, 0x00])),
            (ILI9488_POWER_CTL_ONE, bytearray([0x23])),
            (ILI9488_POWER_CTL_TWO, bytearray([0x11])),
            (ILI9341_VCOM_CONTROL_1, bytearray([0x43, 0x4C])),
            (ILI9341_VCOM_CONTROL_2, bytearray([0xA0])),
            (_espidf.LCD_CMD.MADCTL, bytearray([self._madctl])),
            (_espidf.LCD_CMD.COLMOD, bytearray([ILI9488_COLOR_MODE_16BIT])),
            (ILI9488_FRAME_RATE_NORMAL_CTL, bytearray([0x00, 0x1B])),
            (ILI9341_ENABLE_3G, bytearray([0x00])),
            (_espidf.LCD_CMD.GAMSET, bytearray([0x01])),
            (ILI9488_POSITIVE_GAMMA_CTL, bytearray([0x1F, 0x36, 0x36, 0x3A, 0x0C, 0x05, 0x4F, 0X87, 0x3C, 0x08, 0x11, 0x35, 0x19, 0x13, 0x00])),
            (ILI9488_NEGATIVE_GAMMA_CTL, bytearray([0x00, 0x09, 0x09, 0x05, 0x13, 0x0A, 0x30, 0x78, 0x43, 0x07, 0x0E, 0x0A, 0x26, 0x2C, 0x1F])),
            (_espidf.LCD_CMD.CASET, bytearray([0x00, 0x00, 0x00, 0xEF])),
            (_espidf.LCD_CMD.RASET, bytearray([0x00, 0x00, 0x01, 0x3f])),
            (_espidf.LCD_CMD.RAMWR, bytearray([0x00])),
            (ILI9488_ENTRY_MODE_CTL, bytearray([0x07])),
            (ILI9488_FUNCTION_CTL, bytearray([0x08, 0x82, 0x27]))
        )

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.SLPOUT, None, 0)
        time.sleep_ms(100)

        for cmd, data in init_commands:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, cmd, data, len(data) & ILI9488_INIT_LENGTH_MASK)

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.SLPOUT, None, 0)
        time.sleep_ms(100)

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.DISPON, None, 0)
        time.sleep_ms(100)

        return _espidf.ESP.OK
