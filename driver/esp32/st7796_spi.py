import espidf as _espidf  # NOQA
import spi_driver_base as _spi_driver_base

from spi_driver_base import (
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


class ST7789SPI(_spi_driver_base.SPIDriverBase):
    _color_space = _COLOR_SPACE_BGR
    _color_format = _COLOR_FORMAT_NATIVE_REVERSE
    _panel_base = _espidf.esp_lcd_new_panel_st7789
    _lcd_cmd_bits = 8
    _lcd_param_bits = 8
    _dc_as_cmd_phase = 0
    _dc_low_on_data = 0
    _lsb_first = 0

    def __init__(
        self,
        width, height, dc, rst,
        clk, cs, mosi, miso=-1, d2=None, d3=None, d4=None, d5=None, d6=None,
        d7=None, spi_host=_espidf.HSPI_HOST, rot=PORTRAIT, init=True,
        double_buf=False, pclk_hz=(10 * 1000 * 1000),
        backlight=None, backlight_pwm=False
    ):
        super().__init__(
            width=width, height=height, dc=dc, rst=rst,
            clk=clk, cs=cs, mosi=mosi, miso=miso, d2=d2, d3=d3, d4=d4, d5=d5,
            d6=d6, d7=d7, spi_host=spi_host, rot=rot, init=init,
            double_buf=double_buf, pclk_hz=pclk_hz,
            backlight=backlight, backlight_pwm=backlight_pwm,
            color_space=self._color_space
        )
