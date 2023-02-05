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


class ParallelDriverBase(object):
    _panel_base = None
    _dc_idle_level = 0
    _dc_cmd_level = 0
    _dc_dummy_level = 0
    _dc_data_level = 1
    _lcd_cmd_bits = 8
    _lcd_param_bits = 8
    _memory_factor = 4
    _color_format = None

    def __init__(
        self,
        width, height, dc, cs, rst, wr, pclk_hz,
        d0, d1, d2, d3, d4, d5, d6, d7, d8=None, d9=None, d10=None, d11=None,
        d12=None, d13=None, d14=None, d15=None, init=True, double_buf=False,
        rot=PORTRAIT, color_space=COLOR_SPACE_RGB, backlight=None,
        backlight_pwm=False
    ):
        self.dc = dc
        self.cs = cs
        self.wr = wr
        self.rst = rst
        self.d0 = d0
        self.d1 = d1
        self.d2 = d2
        self.d3 = d3
        self.d4 = d4
        self.d5 = d5
        self.d6 = d6
        self.d7 = d7
        self.d8 = d8
        self.d9 = d9
        self.d10 = d10
        self.d11 = d11
        self.d12 = d12
        self.d13 = d13
        self.d14 = d14
        self.d15 = d15
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

        self.i80_bus = espidf.esp_lcd_i80_bus_handle_t()
        self.i80_config = espidf.esp_lcd_i80_bus_config_t()
        self.io_handle = espidf.esp_lcd_panel_io_handle_t()
        self.io_config = espidf.esp_lcd_panel_io_i80_config_t()
        self.panel_handle = espidf.esp_lcd_panel_handle_t()
        self.panel_config = espidf.esp_lcd_panel_dev_config_t()

        buf = espidf.heap_caps_malloc(
            int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
            espidf.MALLOC_CAP.DMA
        )

        if double_buf:
            buf2 = espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
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

        self.disp_draw_buf.init(
            buf,
            buf2,
            int((height * width) // self.memory_factor)
        )

        if init:
            self.init()

    def monitor(self, _, time, px):
        self.monitor_acc_time += time
        self.monitor_acc_px += px
        self.monitor_count += 1

    def init(self):
        i80_config = self.i80_config

        i80_config.dc_gpio_num = self.dc
        i80_config.wr_gpio_num = self.wr

        if self.d15 != -1:
            i80_config.data_gpio_nums[0] = self.d0
            i80_config.data_gpio_nums[1] = self.d1
            i80_config.data_gpio_nums[2] = self.d2
            i80_config.data_gpio_nums[3] = self.d3
            i80_config.data_gpio_nums[4] = self.d4
            i80_config.data_gpio_nums[5] = self.d5
            i80_config.data_gpio_nums[6] = self.d6
            i80_config.data_gpio_nums[7] = self.d7
            i80_config.data_gpio_nums[8] = self.d8
            i80_config.data_gpio_nums[9] = self.d9
            i80_config.data_gpio_nums[10] = self.d10
            i80_config.data_gpio_nums[11] = self.d11
            i80_config.data_gpio_nums[12] = self.d12
            i80_config.data_gpio_nums[13] = self.d13
            i80_config.data_gpio_nums[14] = self.d14
            i80_config.data_gpio_nums[15] = self.d15
            i80_config.bus_width = 16
            i80_config.psram_trans_align = 16
            i80_config.sram_trans_align = 16
        else:
            i80_config.data_gpio_nums[0] = self.d0
            i80_config.data_gpio_nums[1] = self.d1
            i80_config.data_gpio_nums[2] = self.d2
            i80_config.data_gpio_nums[3] = self.d3
            i80_config.data_gpio_nums[4] = self.d4
            i80_config.data_gpio_nums[5] = self.d5
            i80_config.data_gpio_nums[6] = self.d6
            i80_config.data_gpio_nums[7] = self.d7
            i80_config.bus_width = 8
            i80_config.sram_trans_align = 8

        i80_config.clk_src = espidf.LCD_CLK_SRC.PLL160M
        i80_config.max_transfer_bytes = (
            int((self.height * self.width * lv.color_t.__SIZE__) // self.memory_factor)
        )
        i80_config.psram_trans_align = 64

        espidf.esp_lcd_new_i80_bus(i80_config, self.i80_bus)

        io_config = self.io_config
        io_config.cs_gpio_num = self.cs
        io_config.pclk_hz = self.pclk_hz
        io_config.trans_queue_depth = 2
        io_config.dc_levels.dc_idle_level = self._dc_idle_level
        io_config.dc_levels.dc_cmd_level = self._dc_cmd_level
        io_config.dc_levels.dc_dummy_level = self._dc_dummy_level
        io_config.dc_levels.dc_data_level = self._dc_data_level

        io_config.on_color_trans_done = self.flush_ready
        io_config.lcd_cmd_bits = self._lcd_cmd_bits
        io_config.lcd_param_bits = self._lcd_param_bits

        espidf.esp_lcd_new_panel_io_i80(
            self.i80_bus,
            self.io_config,
            self.io_handle
        )

        panel_config = self.panel_config
        panel_config.reset_gpio_num = self.rst
        panel_config.color_space = self.color_space
        panel_config.bits_per_pixel = lv.color_t.__SIZE__ * 8

        panel_handle = self.panel_handle

        self._panel_base(
            self.io_handle,
            panel_config,
            panel_handle
        )

        espidf.esp_lcd_panel_reset(panel_handle)
        espidf.esp_lcd_panel_init(panel_handle)
        espidf.esp_lcd_panel_invert_color(panel_handle, True)

        disp_drv = self.disp_drv
        disp_drv.init()
        disp_drv.hor_res = self.width
        disp_drv.ver_res = self.height
        disp_drv.flush_cb = self.flush_cb
        disp_drv.draw_buf = self.disp_draw_buf
        disp_drv.user_data = panel_handle
        disp_drv.monitor_cb = self.monitor

        if self._color_format is not None:
            disp_drv.color_format = self._color_format

        disp_drv.register()

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
