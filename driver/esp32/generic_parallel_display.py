
import espidf
import lvgl as lv


class GenericParallelDisplay(object):

    def __init__(
        self,
        width, height, de, vsync, hsync, pclk, speed, hsync_pulse_width, 
        hsync_back_porch, hsync_front_porch, hsync_polarity, vsync_pulse_width, 
        vsync_back_porch, vsync_front_porch, vsync_polarity, pclk_active_neg,
        d0, d1, d2, d3, d4, d5, d6, d7, d8=None, d9=None, d10=None, d11=None, 
        d12=None, d13=None, d14=None, d15=None, init=True
    ):
        self.de = de
        self.vsync = vsync
        self.hsync = hsync
        self.pclk = pclk
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
        self.speed = speed
        self.width = width
        self.height = height
        self.hsync_pulse_width = hsync_pulse_width
        self.hsync_back_porch = hsync_back_porch
        self.hsync_front_porch = hsync_front_porch
        self.hsync_polarity = hsync_polarity
        self.vsync_pulse_width = vsync_pulse_width
        self.vsync_back_porch = vsync_back_porch
        self.vsync_front_porch = vsync_front_porch
        self.vsync_polarity = vsync_polarity
        self.pclk_active_neg = pclk_active_neg

        self.panel_handle = espidf.esp_lcd_panel_handle_t()
        self.rgb_panel = None
        
        panel_config = espidf.heap_caps_calloc(
            1, 
            espidf.esp_lcd_rgb_panel_config_t.__SIZE__, 
            espidf.MALLOC_CAP.DMA | espidf.MALLOC_CAP.INTERNAL
        )
        
        if not panel_config:
            raise RuntimeError(
                'Unable to allocate DMA memory for display panel config'
            )
            
        self.panel_config = (
            espidf.esp_lcd_rgb_panel_config_t.__cast__(panel_config)
        )
        
        self.disp_draw_buf = lv.disp_draw_buf_t()
        self.disp_drv = lv.disp_drv_t()
        self.disp_drv.init()
        self.disp_drv.hor_res = width
        self.disp_drv.ver_res = height
        self.disp_drv.flush_cb = self.flush_cb
        self.disp_drv.draw_buf = self.disp_draw_buf
        
        if init:
            self.init()
        
    def init(self):
        panel_config = self.panel_config
        
        panel_config.data_gpio_nums[0] = self.d0
        panel_config.data_gpio_nums[1] = self.d1
        panel_config.data_gpio_nums[2] = self.d2
        panel_config.data_gpio_nums[3] = self.d3
        panel_config.data_gpio_nums[4] = self.d4
        panel_config.data_gpio_nums[5] = self.d5
        panel_config.data_gpio_nums[6] = self.d6
        panel_config.data_gpio_nums[7] = self.d7
        panel_config.data_width = 8
        panel_config.sram_trans_align = 8

        if self.d15 is not None:
            panel_config.data_width = 16
            panel_config.sram_trans_align = 16
            panel_config.data_gpio_nums[0] = self.d0
            panel_config.data_gpio_nums[1] = self.d1
            panel_config.data_gpio_nums[2] = self.d2
            panel_config.data_gpio_nums[3] = self.d3
            panel_config.data_gpio_nums[4] = self.d4
            panel_config.data_gpio_nums[5] = self.d5
            panel_config.data_gpio_nums[6] = self.d6
            panel_config.data_gpio_nums[7] = self.d7
            panel_config.data_gpio_nums[8] = self.d8
            panel_config.data_gpio_nums[9] = self.d9
            panel_config.data_gpio_nums[10] = self.d10
            panel_config.data_gpio_nums[11] = self.d11
            panel_config.data_gpio_nums[12] = self.d12
            panel_config.data_gpio_nums[13] = self.d13
            panel_config.data_gpio_nums[14] = self.d13
            panel_config.data_gpio_nums[15] = self.d15

        panel_config.clk_src = espidf.LCD_CLK_SRC.PLL160M
        panel_config.psram_trans_align = 64
        panel_config.hsync_gpio_num = self.hsync
        panel_config.vsync_gpio_num = self.vsync
        panel_config.de_gpio_num = self.de
        panel_config.pclk_gpio_num = self.pclk
        panel_config.disp_gpio_num = espidf.GPIO_NUM.NC
        panel_config.flags.disp_active_low = 0
        panel_config.flags.relax_on_idle = 0
        panel_config.flags.fb_in_psram = 1
    
        panel_config.timings.pclk_hz = self.speed
        panel_config.timings.h_res = self.width
        panel_config.timings.v_res = self.height
        panel_config.timings.hsync_back_porch = self.hsync_back_porch
        panel_config.timings.hsync_front_porch = self.hsync_front_porch
        panel_config.timings.hsync_pulse_width = self.hsync_pulse_width
        panel_config.timings.vsync_back_porch = self.vsync_back_porch
        panel_config.timings.vsync_front_porch = self.vsync_front_porch
        panel_config.timings.vsync_pulse_width = self.vsync_pulse_width
        panel_config.timings.flags.hsync_idle_low = int(not self.hsync_polarity)
        panel_config.timings.flags.vsync_idle_low = int(not self.vsync_polarity)
        panel_config.timings.flags.de_idle_high = 0
        panel_config.timings.flags.pclk_active_neg = self.pclk_active_neg
        panel_config.timings.flags.pclk_idle_high = 0

        ret = espidf.esp_lcd_new_rgb_panel(panel_config, self.panel_handle)
        if ret != espidf.ESP.OK:
            raise RuntimeError("Failed creating display panel")
    
        ret = esp_lcd_panel_reset(self.panel_handle)
        if ret != espidf.ESP.OK:
            raise RuntimeError("Failed resetting display panel")
    
        ret = esp_lcd_panel_init(self.panel_handle)
        if ret != espidf.ESP.OK:
            raise RuntimeError("Failed initializing display panel")
    
        color = 0xFFFF
        ret = espidf.esp_lcd_panel_draw_bitmap(
            self.panel_handle, 0, 0, 1, 1, color
        )
        if ret != espidf.ESP.OK:
            raise RuntimeError("Failed rendering to display panel")
    
        self.rgb_panel = espidf.esp_rgb_panel_container(self.panel_handle)
        
        self.disp_draw_buf.init(
            self.rgb_panel.fb, None, self.width * self.height
        )
        self.disp_drv.register()
    
    def flush_cb(self, disp_drv, area, color_p):
        espidf.esp_lcd_panel_draw_bitmap(
            self.panel_handle, area.x1, area.y1, 
            area.x2 + 1, area.y2 + 1, color_p
        )
        lv.disp_flush_ready(disp_drv)
