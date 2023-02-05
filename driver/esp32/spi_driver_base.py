import espidf  # NOQA
import machine
import lvgl as lv
from micropython import const

COLOR_SPACE_RGB = espidf.ESP_LCD_COLOR_SPACE_RGB
COLOR_SPACE_BGR = espidf.ESP_LCD_COLOR_SPACE_BGR

COLOR_FORMAT_NATIVE = lv.COLOR_FORMAT.NATIVE
COLOR_FORMAT_NATIVE_REVERSE = lv.COLOR_FORMAT.NATIVE_REVERSE
COLOR_FORMAT_RGB232 = lv.COLOR_FORMAT.RGB232
COLOR_FORMAT_RGBA2328 = lv.COLOR_FORMAT.RGBA2328
COLOR_FORMAT_RGB565 = lv.COLOR_FORMAT.RGB565
COLOR_FORMAT_RGBA5658 = lv.COLOR_FORMAT.RGBA5658
COLOR_FORMAT_RGBA5551 = lv.COLOR_FORMAT.RGBA5551
COLOR_FORMAT_RGBA4444 = lv.COLOR_FORMAT.RGBA4444
COLOR_FORMAT_RGB565A8 = lv.COLOR_FORMAT.RGB565A8
COLOR_FORMAT_RGB888 = lv.COLOR_FORMAT.RGB888
COLOR_FORMAT_RGBA8888 = lv.COLOR_FORMAT.RGBA8888
COLOR_FORMAT_RGBX8888 = lv.COLOR_FORMAT.RGBX8888

PORTRAIT = const(-1)
LANDSCAPE = const(-2)
REVERSE_PORTRAIT = const(-3)
REVERSE_LANDSCAPE = const(-4)


class SPIDriverBase(object):
    _panel_base = None
    _lcd_cmd_bits = 8
    _lcd_param_bits = 8
    _dc_as_cmd_phase = 0
    _dc_low_on_data = 0
    _lsb_first = 0
    _memory_factor = 4
    _color_format = None

    def __init__(
        self,
        width, height, dc, rst,
        clk, cs, mosi, miso=-1, d2=None, d3=None, d4=None, d5=None, d6=None,
        d7=None, spi_host=espidf.HSPI_HOST, rot=PORTRAIT, init=True,
        double_buf=False, pclk_hz=(10 * 1000 * 1000),
        color_space=COLOR_SPACE_BGR, backlight=None, backlight_pwm=False
    ):
        self.dc = dc
        self.cs = cs
        self.rst = rst
        self.mosi = mosi
        self.miso = miso
        self.clk = clk
        self.d2 = d2
        self.d3 = d3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6
        self.d7 = d7
        self.spi_host = spi_host
        self.pclk_hz = pclk_hz
        self.width = width
        self.height = height
        self.rot = rot
        self.color_space = color_space
        self.monitor_acc_time = 0
        self.monitor_acc_px = 0
        self.monitor_count = 0
        self.cycles_in_ms = espidf.esp_clk_cpu_freq() // 1000
        self.start_time_ptr = espidf.C_Pointer()
        self.end_time_ptr = espidf.C_Pointer()
        self.flush_acc_setup_cycles = 0
        self.flush_acc_dma_cycles = 0

        if self._memory_factor % 4:
            raise RuntimeError('memory_factor must be a multiple of 4')

        self.memory_factor = int(self._memory_factor // (int(double_buf) + 1))

        if backlight != -1:
            self._back_pin = machine.Pin(backlight, machine.Pin.OUT)

            if backlight_pwm:
                self._backlight_pwm = machine.PWM(
                    self._back_pin,
                    freq=25000,
                    duty_u16=0
                )
            else:
                self._backlight_pwm = None
        else:
            self._back_pin = None
            self._backlight_pwm = None

        if not lv.is_initialized():
            lv.init()

        self.disp_draw_buf = lv.disp_draw_buf_t()
        self.disp_drv = lv.disp_drv_t()

        self.spi_bus_config = espidf.spi_bus_config_t()
        self.io_spi_config = espidf.esp_lcd_panel_io_spi_config_t()
        self.io_handle = espidf.esp_lcd_panel_io_handle_t()
        self.panel_handle = espidf.esp_lcd_panel_handle_t()
        self.panel_config = espidf.esp_lcd_panel_dev_config_t()

        buf = espidf.heap_caps_malloc(
            int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
            espidf.MALLOC_CAP.DMA
        )

        if double_buf:
            buf2 = espidf.heap_caps_malloc(
                int(
                    (height * width * lv.color_t.__SIZE__) // self.memory_factor
                    ),
                espidf.MALLOC_CAP.DMA
            )
        else:
            buf2 = None

        if buf is None:
            buf = espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                espidf.MALLOC_CAP.DEFAULT
            )

            if double_buf:
                buf2 = espidf.heap_caps_malloc(
                    int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                    espidf.MALLOC_CAP.DEFAULT
                )
        elif double_buf and buf2 is None:
            buf2 = espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                espidf.MALLOC_CAP.DEFAULT
            )

        if buf is None:
            raise RuntimeError(
                'Not enough free memory to create frame buffer(s) '
                'either change the memory_factor to a larger number '
                '(multiple of 4) or initilize the display first thing.'
            )

        if double_buf and buf2 is None:
            print(
                'WARNING: Not enough memory to create second frame buffer, '
                'either change the memory_factor to a larger number '
                '(multiple of 4) or initilize the display first thing.'
            )

        self.disp_draw_buf.init(buf, buf2, height * 20)

        if init:
            self.init()

    def monitor(self, _, time, px):
        self.monitor_acc_time += time
        self.monitor_acc_px += px
        self.monitor_count += 1

    def init(self):
        spi_bus_config = self.spi_bus_config
        spi_bus_config.sclk_io_num = self.clk
        spi_bus_config.mosi_io_num = self.mosi
        spi_bus_config.miso_io_num = self.miso
        spi_bus_config.quadwp_io_num = -1
        spi_bus_config.quadhd_io_num = -1
        spi_bus_config.max_transfer_sz = (
            int((self.height * self.width * lv.color_t.__SIZE__) // self.memory_factor)
        )

        io_spi_config = self.io_spi_config
        io_spi_config.cs_gpio_num = self.cs
        io_spi_config.dc_gpio_num = self.dc
        io_spi_config.pclk_hz = self.pclk_hz
        io_spi_config.trans_queue_depth = 2
        io_spi_config.on_color_trans_done = self.flush_ready
        io_spi_config.lcd_cmd_bits = self._lcd_cmd_bits
        io_spi_config.lcd_param_bits = self._lcd_param_bits
        io_spi_config.flags.dc_as_cmd_phase = self._dc_as_cmd_phase
        io_spi_config.flags.dc_low_on_data = self._dc_low_on_data
        io_spi_config.flags.lsb_first = self._lsb_first

        panel_config = self.panel_config
        panel_config.reset_gpio_num = self.rst
        panel_config.color_space = self.color_space
        panel_config.bits_per_pixel = lv.color_t.__SIZE__ * 8

        if self.d2 is None:
            io_spi_config.flags.octal_mode = 0
            io_spi_config.spi_mode = 0
        else:
            # quad mode
            if self.d7 is None:
                io_spi_config.flags.octal_mode = 0
                io_spi_config.spi_mode = 0
                spi_bus_config.quadwp_io_num = self.d2
                spi_bus_config.quadhd_io_num = self.d3
                spi_bus_config.flags = espidf.SPICOMMON_BUSFLAG.QUAD
            else:
                io_spi_config.flags.octal_mode = 1
                io_spi_config.spi_mode = 3
                spi_bus_config.data2_io_num = self.d2
                spi_bus_config.data3_io_num = self.d3
                spi_bus_config.data4_io_num = self.d4
                spi_bus_config.data5_io_num = self.d5
                spi_bus_config.data6_io_num = self.d6
                spi_bus_config.data7_io_num = self.d7
                spi_bus_config.flags = espidf.SPICOMMON_BUSFLAG.OCTAL

        espidf.esp_lcd_new_panel_io_spi(
            self.spi_host,
            io_spi_config,
            self.io_handle
        )
        self._panel_base(
            self.io_handle,
            self.panel_config,
            self.panel_handle
        )

        espidf.esp_lcd_panel_reset(self.panel_handle)
        espidf.esp_lcd_panel_init(self.panel_handle)

        disp_drv = self.disp_drv
        disp_drv.init()

        disp_drv.hor_res = self.width
        disp_drv.ver_res = self.height
        disp_drv.flush_cb = self.flush_cb
        disp_drv.draw_buf = self.disp_draw_buf
        disp_drv.user_data = self.panel_handle
        disp_drv.monitor_cb = self.monitor

        if self._color_format is not None:
            disp_drv.color_format = self._color_format

        disp_drv.register()

        self.set_rotation(self.rot)

    def flush_ready(self, panel_io, edata, user_ctx):
        lv.disp_flush_ready(self.disp_drv)
        return False

    def flush_cb(self, disp_drv, area, color_p):
        espidf.esp_lcd_panel_draw_bitmap(
            self.panel_handle,
            area.x1,
            area.y1,
            area.x2 + 1,
            area.y2 + 1,
            color_p
        )

    @property
    def backlight_level(self):
        if self._backlight_pwm is None:
            if self._back_pin is not None:
                return self._back_pin.value() * 100
        else:
            duty = self._backlight_pwm.duty_u16()
            return round((duty / 65535.0) * 100.0, 2)

    @backlight_level.setter
    def backlight_level(self, value):
        if self._back_pin is None:
            return

        if self._backlight_pwm is None:
            self._back_pin.value(int(value >= 50))
        else:
            self._backlight_pwm.duty_u16(value / 100.0 * 65535)

    def set_gap(self, x, y):
        espidf.esp_lcd_panel_set_gap(self.panel_handle, x, y)

    def set_rotation(self, rot):
        self.rot = rot

        if self.rot == PORTRAIT:
            espidf.esp_lcd_panel_mirror(self.panel_handle, False, False)
            espidf.esp_lcd_panel_swap_xy(self.panel_handle, False)

        elif self.rot == LANDSCAPE:
            espidf.esp_lcd_panel_swap_xy(self.panel_handle, True)

        elif self.rot == REVERSE_PORTRAIT:
            espidf.esp_lcd_panel_mirror(self.panel_handle, True, True)

        elif self.rot == REVERSE_LANDSCAPE:
            espidf.esp_lcd_panel_mirror(self.panel_handle, True, True)
            espidf.esp_lcd_panel_swap_xy(self.panel_handle, True)




