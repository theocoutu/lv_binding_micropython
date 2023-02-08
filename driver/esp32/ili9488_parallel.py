import espidf as _espidf  # NOQA
from micropython import const as _const
import time
import parallel_driver_base as _parallel_driver_base
from parallel_driver_base import (
    COLOR_SPACE_RGB as _COLOR_SPACE_RGB,
    COLOR_SPACE_BGR as _COLOR_SPACE_BGR,
    COLOR_FORMAT_RGB565 as _COLOR_FORMAT_RGB565,
    PORTRAIT as _PORTRAIT,
    LANDSCAPE as _LANDSCAPE,
    REVERSE_PORTRAIT as _REVERSE_PORTRAIT,
    REVERSE_LANDSCAPE as _REVERSE_LANDSCAPE,
)

PORTRAIT = _PORTRAIT
LANDSCAPE = _LANDSCAPE
REVERSE_PORTRAIT = _REVERSE_PORTRAIT
REVERSE_LANDSCAPE = _REVERSE_LANDSCAPE


ILI9488_INTRFC_MODE_CTL = _const(0xB0)
ILI9488_FRAME_RATE_NORMAL_CTL = _const(0xB1)
ILI9488_INVERSION_CTL = _const(0xB4)
ILI9488_FUNCTION_CTL = _const(0xB6)
ILI9488_ENTRY_MODE_CTL = _const(0xB7)
ILI9488_POWER_CTL_ONE = _const(0xC0)
ILI9488_POWER_CTL_TWO = _const(0xC1)
ILI9488_POWER_CTL_THREE = _const(0xC5)
ILI9488_POSITIVE_GAMMA_CTL = _const(0xE0)
ILI9488_NEGATIVE_GAMMA_CTL = _const(0xE1)
ILI9488_ADJUST_CTL_THREE = _const(0xF7)

ILI9488_COLOR_MODE_16BIT = _const(0x55)
ILI9488_COLOR_MODE_18BIT = _const(0x66)

ILI9488_INTERFACE_MODE_USE_SDO = _const(0x00)
ILI9488_INTERFACE_MODE_IGNORE_SDO = _const(0x80)

ILI9488_IMAGE_FUNCTION_DISABLE_24BIT_DATA = _const(0x00)

ILI9488_WRITE_MODE_BCTRL_DD_ON = _const(0x28)
ILI9488_FRAME_RATE_60HZ = _const(0xA0)

ILI9488_INIT_LENGTH_MASK = _const(0x1F)
ILI9488_INIT_DONE_FLAG = _const(0xFF)


class ILI9488Parallel(_parallel_driver_base.ParallelDriverBase):
    _color_space = _COLOR_SPACE_RGB
    _panel_base = _espidf.esp_lcd_new_panel_io_i80
    _dc_idle_level = 0
    _dc_cmd_level = 0
    _dc_dummy_level = 0
    _dc_data_level = 1
    _lcd_cmd_bits = 8
    _lcd_param_bits = 8
    _memory_factor = 4
    _reset_value = 1
    _color_format = _COLOR_FORMAT_RGB565

    def __init__(
        self, width, height, dc, cs, rst, wr, pclk_hz, d0, d1, d2, d3, d4, d5,
        d6, d7, reset=None, d8=None, d9=None, d10=None, d11=None,
        d12=None, d13=None, d14=None, d15=None, init=True, double_buf=False,
        rot=PORTRAIT, backlight=None, backlight_pwm=False
    ):
        self._x_gap = 0
        self._y_gap = 0

        self._madctl = _espidf.LCD_CMD.MX_BIT

        if self._color_space == _COLOR_SPACE_BGR:
            self._madctl |= _espidf.LCD_CMD.BGR_BIT

        super().__init__(
            width=width, height=height, dc=dc, cs=cs, rst=rst, wr=wr,
            pclk_hz=pclk_hz, d0=d0, d1=d1, d2=d2, d3=d3, d4=d4, d5=d5, d6=d6,
            d7=d7, d8=d8, d9=d9, d10=d10, d11=d11, d12=d12, d13=d13, d14=d14,
            d15=d15, reset=reset, init=init, double_buf=double_buf, rot=rot,
            backlight=backlight, backlight_pwm=backlight_pwm,
            color_space=self._color_space
        )

    def _reset_cb(self, panel):

        if self._reset_pin is not None:
            self._reset_pin.value(self._reset_value)
            time.sleep_ms(10)
            self._reset_pin.value(not self._reset_value)
            time.sleep_ms(10)
        else:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.SWRESET, None, 0)
            time.sleep_ms(20)
    
        return _espidf.ESP.OK

    def _init_cb(self, panel):    
        init_commands = (
            (ILI9488_POSITIVE_GAMMA_CTL, bytearray([0x00, 0x03, 0x09, 0x08, 0x16, 0x0A, 0x3F, 0x78, 0x4C, 0x09, 0x0A, 0x08, 0x16, 0x1A, 0x0F])), 
            (ILI9488_NEGATIVE_GAMMA_CTL, bytearray([0x00, 0x16, 0x19, 0x03, 0x0F, 0x05, 0x32, 0x45, 0x46, 0x04, 0x0E, 0x0D, 0x35, 0x37, 0x0F])),
            (ILI9488_POWER_CTL_ONE, bytearray([0x17, 0x15])),
            (ILI9488_POWER_CTL_TWO, bytearray([0x41])), 
            (ILI9488_POWER_CTL_THREE, bytearray([0x00, 0x12, 0x80])), 
            (_espidf.LCD_CMD.MADCTL, bytearray([self._madctl])),
            (_espidf.LCD_CMD.COLMOD, bytearray([ILI9488_COLOR_MODE_16BIT])),
            (ILI9488_INTRFC_MODE_CTL, bytearray([ILI9488_INTERFACE_MODE_USE_SDO])),
            (ILI9488_FRAME_RATE_NORMAL_CTL, bytearray([ILI9488_FRAME_RATE_60HZ])), 
            (ILI9488_INVERSION_CTL, bytearray([0x02])),
            (ILI9488_FUNCTION_CTL, bytearray([0x02, 0x02, 0x3B])), 
            (ILI9488_ENTRY_MODE_CTL, bytearray([0xC6])), 
            (ILI9488_ADJUST_CTL_THREE, bytearray([0xA9, 0x51, 0x2C, 0x02]))
        )
        for cmd, data in init_commands:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, cmd, data, len(data) & ILI9488_INIT_LENGTH_MASK)
    
        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.SLPOUT, None, 0)
        time.sleep_ms(100)

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.DISPON, None, 0)
        time.sleep_ms(100)
        
        return _espidf.ESP.OK

    def _send_coords(self, start, end, cmd):
        data = bytearray([(start >> 8) & 0xFF, start & 0xFF, ((end - 1) >> 8) & 0xFF, (end - 1) & 0xFF])
        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, cmd, data, 4)

    def _draw_bitmap(self, panel, x_start, y_start, x_end, y_end, color_data):    
        x_start += self._x_gap
        x_end += self._x_gap
        y_start += self._y_gap
        y_end += self._y_gap
    
        color_data_len = (x_end - x_start) * (y_end - y_start)
    
        self._send_coords(x_start, x_end, _espidf.LCD_CMD.CASET)
        self._send_coords(y_start, y_end, _espidf.LCD_CMD.RASET)
    
        _espidf.esp_lcd_panel_io_tx_color(self.io_handle, _espidf.LCD_CMD.RAMWR, color_data, color_data_len * 2)
    
        return _espidf.ESP.OK

    def _invert_color_cb(self, panel, invert_color_data):
        if invert_color_data:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.INVON, None, 0)
        else:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.INVOFF, None, 0)
    
        return _espidf.ESP.OK

    def _mirror_cb(self, panel, mirror_x, mirror_y):
        if mirror_x:
            self._madctl |= _espidf.LCD_CMD.MX_BIT
        else:
            self._madctl &= ~_espidf.LCD_CMD.MX_BIT
    
        if mirror_y:
            self._madctl |= _espidf.LCD_CMD.MY_BIT
        else:
            self._madctl &= ~_espidf.LCD_CMD.MY_BIT
    
        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.MADCTL, self._madctl, 1)
        return _espidf.ESP.OK


    def _swap_xy_cb(self, panel, swap_axes):
        if swap_axes:
            self._madctl |= _espidf.LCD_CMD.MV_BIT
        else:
            self._madctl &= ~_espidf.LCD_CMD.MV_BIT
            
        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.MADCTL, self._madctl, 1)
        return _espidf.ESP.OK

    def _set_gap_cb(self, panel, x_gap, y_gap):
        self._x_gap = x_gap
        self._y_gap = y_gap
        return _espidf.ESP.OK

    def _disp_off_cb(self, panel, on_off):        
        if on_off:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.DISPON, None, 0)
        else:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, _espidf.LCD_CMD.DISPOFF, None, 0)
            
        time.sleep_ms(100)
    
        return _espidf.ESP.OK
