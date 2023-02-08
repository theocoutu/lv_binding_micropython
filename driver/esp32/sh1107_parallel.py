import espidf as _espidf  # NOQA
import parallel_driver_base as _parallel_driver_base
from micropython import const
from parallel_driver_base import (
    PORTRAIT as _PORTRAIT,
    LANDSCAPE as _LANDSCAPE,
    REVERSE_PORTRAIT as _REVERSE_PORTRAIT,
    REVERSE_LANDSCAPE as _REVERSE_LANDSCAPE
)


PORTRAIT = _PORTRAIT
LANDSCAPE = _LANDSCAPE
REVERSE_PORTRAIT = _REVERSE_PORTRAIT
REVERSE_LANDSCAPE = _REVERSE_LANDSCAPE

SH1107_I2C_ADDRESS = const(0x3C)
SH1107_I2C_ADDRESS1 = const(0x3D)
SH1107_I2C_CMD = const(0x00)
SH1107_I2C_RAM = const(0x40)
SH1107_PARAM_ONOFF = const(0xAE)
SH1107_PARAM_MIRROR_X = const(0xA0)
SH1107_PARAM_MIRROR_Y = const(0xC0)
SH1107_PARAM_INVERT_COLOR = const(0xA6)


class SH1107Parallel(_parallel_driver_base.ParallelDriverBase):

    def _init_cb(self, panel):
        init_commands = (
            0xAE, 0xDC, 0x00, 0x81, 0x2F, 0x20, 0xA0, 0xC7, 0xA8, 0x7F, 0xA3,
            0x60, 0xD5, 0x51, 0xD9, 0x22, 0xDB, 0x35, 0xB0, 0xDA, 0x12, 0xA4,
            0xA6
        )

        for cmd in init_commands:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([cmd]), 1)

        return _espidf.ESP.OK

    def _draw_bitmap_cb(self, panel, x_start, y_start, x_end, y_end, color_data):
        x_start += self._x_gap
        x_end += self._x_gap
        y_start += self._y_gap
        y_end += self._y_gap

        if self._swap_axes:
            x_start, y_start = y_start, x_start
            x_end, y_end = y_end, x_end

        row_start = y_start >> 3
        row_end = y_end >> 3

        column_low = x_start & 0x0F
        column_high = (x_start >> 4) & 0x0F

        size = x_end - x_start

        for i in range(row_start, row_end):
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([0x10 | column_high]), 1)
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([0x00 | column_low]), 1)
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([0xB0 | i]), 1)

            ptr = color_data[i * x_end]
            _espidf.esp_lcd_panel_io_tx_color(self.io_handle, SH1107_I2C_RAM, bytearray([ptr]), size)

        return _espidf.ESP.OK

    def _invert_color_cb(self, panel, invert_color_data):
        if invert_color_data:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([SH1107_PARAM_INVERT_COLOR]), 1)
        else:
            _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([SH1107_PARAM_INVERT_COLOR | 0x01]), 1)

        return _espidf.ESP.OK


    def _mirror_cb(self, panel, mirror_x, mirror_y):
        param_x = SH1107_PARAM_MIRROR_X
        param_y = SH1107_PARAM_MIRROR_Y | 0x07

        if mirror_x:
            param_x = SH1107_PARAM_MIRROR_X | 0x01

        if mirror_y:
            param_y = SH1107_PARAM_MIRROR_Y | 0x08

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([param_x]), 1)
        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, bytearray([param_y]), 1)

        return _espidf.ESP.OK

    def _swap_xy_cb(self, panel, swap_axes):
        self._swap_axes = swap_axes
        return _espidf.ESP.OK

    def _set_gap_cb(self, panel, x_gap, y_gap):
        self._x_gap = x_gap
        self._y_gap = y_gap
        return _espidf.ESP.OK

    def _disp_off_cb(self, panel, on_off):
        on_off = not on_off

        if on_off:
            param = bytearray([SH1107_PARAM_ONOFF | 0x01])
        else:
            param = bytearray([SH1107_PARAM_ONOFF])

        _espidf.esp_lcd_panel_io_tx_param(self.io_handle, SH1107_I2C_CMD, param, 1)

        return _espidf.ESP.OK
