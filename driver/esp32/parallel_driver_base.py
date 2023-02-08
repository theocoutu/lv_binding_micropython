import espidf as _espidf  # NOQA
import machine
import time
import lvgl as lv
from micropython import const


COLOR_SPACE_RGB = _espidf.ESP_LCD_COLOR_SPACE_RGB
COLOR_SPACE_BGR = _espidf.ESP_LCD_COLOR_SPACE_BGR

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
    _panel_base = _espidf.esp_lcd_new_panel_io_i80
    _dc_idle_level = 0
    _dc_cmd_level = 0
    _dc_dummy_level = 0
    _dc_data_level = 1
    _lcd_cmd_bits = 8
    _lcd_param_bits = 8
    _memory_factor = 4
    _reset_value = 1
    _color_format = None

    def __init__(
        self, width, height, dc, cs, rst, wr, pclk_hz, d0, d1, d2, d3, d4, d5, 
        d6, d7, d8=None, d9=None, d10=None, d11=None, d12=None, d13=None, 
        d14=None, d15=None, reset=None, init=True, double_buf=False,
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
        self._rot = rot
        self.monitor_acc_time = 0
        self.monitor_acc_px = 0
        self.monitor_count = 0
        self.cycles_in_ms = _espidf.esp_clk_cpu_freq() // 1000
        self.start_time_ptr = _espidf.C_Pointer()
        self.end_time_ptr = _espidf.C_Pointer()
        self.flush_acc_setup_cycles = 0
        self.flush_acc_dma_cycles = 0

        self.color_space = color_space
        if color_space == COLOR_SPACE_RGB:
            self._color_mode = _espidf.ESP_LCD_COLOR_SPACE.RGB
        elif color_space == COLOR_SPACE_BGR:
            self._color_mode = _espidf.ESP_LCD_COLOR_SPACE.BGR
        else:
            raise RuntimeError('Invalid color space')

        if reset is None:
            self._reset_pin = None
        else:
            self._reset_pin = machine.Pin(reset, machine.Pin.OUT, value=self._reset_value)

        if self._memory_factor % 4:
            raise RuntimeError('memory_factor must be a multiple of 4')

        self.memory_factor = int(self._memory_factor // (int(double_buf) + 1))

        if backlight != -1:
            self._back_pin = machine.Pin(backlight, machine.Pin.OUT)

            if backlight_pwm:
                self._backlight_pwm = machine.PWM(self._back_pin, freq=25000, duty_u16=0)
            else:
                self._backlight_pwm = None
        else:
            self._back_pin = None
            self._backlight_pwm = None

        if not lv.is_initialized():
            lv.init()

        self.disp_draw_buf = lv.disp_draw_buf_t()
        self.disp_drv = lv.disp_drv_t()

        self.i80_bus = _espidf.esp_lcd_i80_bus_handle_t()
        self.i80_config = _espidf.esp_lcd_i80_bus_config_t()
        self.io_handle = _espidf.esp_lcd_panel_io_handle_t()
        self.io_config = _espidf.esp_lcd_panel_io_i80_config_t()
        self.panel_handle = _espidf.esp_lcd_panel_t()
        self.panel_config = _espidf.esp_lcd_panel_dev_config_t()

        buf = _espidf.heap_caps_malloc(
            int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
            _espidf.MALLOC_CAP.DMA
        )

        if double_buf:
            buf2 = _espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                _espidf.MALLOC_CAP.DMA
            )
        else:
            buf2 = None

        if buf is None:
            buf = _espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                _espidf.MALLOC_CAP.DEFAULT
            )

            if double_buf:
                buf2 = _espidf.heap_caps_malloc(
                    int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                    _espidf.MALLOC_CAP.DEFAULT
                )
        elif double_buf and buf2 is None:
            buf2 = _espidf.heap_caps_malloc(
                int((height * width * lv.color_t.__SIZE__) // self.memory_factor),
                _espidf.MALLOC_CAP.DEFAULT
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

    def _init_cb(self, panel):
        """
        override this function to inject initilization commands.

        must return 0 if there are no errors otherwise return an EXP error code
        """
        return 0

    def _reset_cb(self, panel):
        """
        override this function to inject initilization commands.

        must return _espidf.ESP.OK if there are no errors otherwise return an EXP error code

        """
        if self._reset_pin is not None:
            self._reset_pin.value(self._reset_value)
            time.sleep(10)
            self._reset_pin.value(not self._reset_value)

        return _espidf.ESP.OK

    def reset(self):
        _espidf.esp_lcd_panel_reset(self.panel_handle)

    def _delete_cb(self, panel):
        return _espidf.ESP.OK

    def delete(self):
        _espidf.esp_lcd_panel_del(self.panel_handle)

    def _draw_bitmap_cb(self, panel, x_start, y_start, x_end, y_end, color_data):
        return _espidf.ESP.OK

    def _mirror_cb(self, panel, x_axis, y_axis):
        return _espidf.ESP.OK

    def mirror(self, mirror_x, mirror_y):
        _espidf.esp_lcd_panel_mirror(self.panel_handle, mirror_x, mirror_y)

    def _swap_xy_cb(self, panel, swap_axes):
        return _espidf.ESP.OK

    def swap_xy(self, value):
        _espidf.esp_lcd_panel_swap_xy(self.panel_handle, value)

    def _set_gap_cb(self, panel, x_gap, y_gap):
        return _espidf.ESP.OK

    def set_gap(self, x, y):
        _espidf.esp_lcd_panel_set_gap(self.panel_handle, x, y)

    def _invert_color_cb(self, panel, invert_color_data):
        return _espidf.ESP.OK

    def invert_color(self, value):
        _espidf.esp_lcd_panel_invert_color(self.panel_handle, value)

    def _disp_off_cb(self, panel, off):
        return _espidf.ESP.OK

    def disp_off(self, value):
        _espidf.esp_lcd_panel_disp_off(self.panel_handle, value)

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

        i80_config.clk_src = _espidf.LCD_CLK_SRC.PLL160M
        i80_config.max_transfer_bytes = (
            int((self.height * self.width * lv.color_t.__SIZE__) // self.memory_factor)
        )
        i80_config.psram_trans_align = 64

        ret = i80_config.new_i80_bus(self.i80_bus)
        if ret != _espidf.ESP.OK:
            raise RuntimeError("Failed adding bus to display panel config")

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

        ret = _espidf.esp_lcd_new_panel_io_i80(
            self.i80_bus,
            self.io_config,
            self.io_handle
        )
        if ret != _espidf.ESP.OK:
            raise RuntimeError("Failed creating display panel io")

        panel_config = self.panel_config
        panel_config.reset_gpio_num = self.rst
        panel_config.color_space = self.color_space
        panel_config.bits_per_pixel = lv.color_t.__SIZE__ * 8

        panel_handle = self.panel_handle

        if self._panel_base is None:
            panel_handle.invert_color = self._invert_color_cb
            panel_handle.disp_off = self._disp_off_cb
            panel_handle.set_gap = self._set_gap_cb
            panel_handle.swap_xy = self._swap_xy_cb
            panel_handle.mirror = self._mirror_cb
            panel_handle.draw_bitmap = self._draw_bitmap_cb
            panel_handle.delete = self._delete_cb
            panel_handle.init = self._init_cb
            panel_handle.reset = self._reset_cb
        else:
            ret = self._panel_base(
                self.io_handle,
                panel_config,
                panel_handle
            )

            if ret != _espidf.ESP.OK:
                raise RuntimeError("Failed initializing display panel")

        ret = _espidf.esp_lcd_panel_reset(panel_handle)
        if ret != _espidf.ESP.OK:
            raise RuntimeError("Failed to reset the display panel")

        ret = _espidf.esp_lcd_panel_init(panel_handle)
        if ret != _espidf.ESP.OK:
            raise RuntimeError("Failed initializing panel")

        # ret = _espidf.esp_lcd_panel_invert_color(panel_handle, True)
        # if ret != _espidf.ESP.OK:
        #     raise RuntimeError("Failed invertng colors")

        self.rotation = self._rot

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
        _espidf.esp_lcd_panel_draw_bitmap(
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
        _espidf.esp_lcd_panel_set_gap(self.panel_handle, x, y)

    @property
    def rotation(self):
        return self._rot

    @rotation.setter
    def rotation(self, value):
        self._rot = value
        if value == PORTRAIT:
            _espidf.esp_lcd_panel_mirror(self.panel_handle, False, False)
            _espidf.esp_lcd_panel_swap_xy(self.panel_handle, False)

        elif value == LANDSCAPE:
            _espidf.esp_lcd_panel_swap_xy(self.panel_handle, True)

        elif value == REVERSE_PORTRAIT:
            _espidf.esp_lcd_panel_mirror(self.panel_handle, True, True)

        elif value == REVERSE_LANDSCAPE:
            _espidf.esp_lcd_panel_mirror(self.panel_handle, True, True)
            _espidf.esp_lcd_panel_swap_xy(self.panel_handle, True)
