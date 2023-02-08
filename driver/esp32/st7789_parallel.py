import espidf as _espidf  # NOQA
import parallel_driver_base as _parallel_driver_base
from parallel_driver_base import (
    COLOR_SPACE_BGR as _COLOR_SPACE_BGR,
    COLOR_FORMAT_NATIVE_REVERSE as _COLOR_FORMAT_NATIVE_REVERSE,
    PORTRAIT as _PORTRAIT,
    LANDSCAPE as _LANDSCAPE,
    REVERSE_PORTRAIT as _REVERSE_PORTRAIT,
    REVERSE_LANDSCAPE as _REVERSE_LANDSCAPE,
)

PORTRAIT = _PORTRAIT
LANDSCAPE = _LANDSCAPE
REVERSE_PORTRAIT = _REVERSE_PORTRAIT
REVERSE_LANDSCAPE = _REVERSE_LANDSCAPE


class ST7789Parallel(_parallel_driver_base.ParallelDriverBase):
    _panel_base = _espidf.esp_lcd_new_panel_st7789
    _color_format = _COLOR_FORMAT_NATIVE_REVERSE
    _color_space = _COLOR_SPACE_BGR
    _dc_idle_level = 0
    _dc_cmd_level = 0
    _dc_dummy_level = 0
    _dc_data_level = 1

    def __init__(
        self,
        width, height, dc, cs, rst, wr, pclk_hz,
        d0, d1, d2, d3, d4, d5, d6, d7, d8=None, d9=None, d10=None, d11=None,
        d12=None, d13=None, d14=None, d15=None, init=True, double_buf=False,
        rot=PORTRAIT, backlight=None, backlight_pwm=False
    ):
        super().__init__(
            width=width, height=height, dc=dc,cs=cs, rst=rst, wr=wr,
            pclk_hz=pclk_hz, d0=d0, d1=d1, d2=d2, d3=d3, d4=d4, d5=d5, d6=d6,
            d7=d7, d8=d8, d9=d9, d10=d10, d11=d11, d12=d12, d13=d13, d14=d14,
            d15=d15, init=init, double_buf=double_buf, rot=rot,
            backlight=backlight, backlight_pwm=backlight_pwm,
            color_space=self._color_space
        )


