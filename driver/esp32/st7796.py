
import espidf as esp
import lvgl as lv
import ili9XXX
from machine import Pin, PWM
import time


class ST7796(ili9XXX.ili9XXX):

    # The st7795 display controller has an internal framebuffer
    # arranged in 320 x 480
    # configuration. Physical displays with pixel sizes less than
    # 320 x 480 must supply a start_x and
    # start_y argument to indicate where the physical display begins
    # relative to the start of the
    # display controllers internal framebuffer.

    # this display driver supports RGB565 and also RGB666. RGB666 is going to
    # use twice as much memory as the RGB565. It is also going to slow down the
    # frame rate by 1/3, This is becasue of the extra byte of data that needs
    # to get sent. To use RGB666 the color depth MUST be set to 32.
    # so when compiling
    # make sure to have LV_COLOR_DEPTH=32 set in LVFLAGS when you call make.
    # For RGB565 you need to have LV_COLOR_DEPTH=16

    # the reason why we use a 32 bit color depth is because of how the data gets
    # written. The entire 8 bits for each byte gets sent. The controller simply
    # ignores the lowest 2 bits in the byte to make it a 6 bit color channel
    # We just have to tell lvgl that we want to use

    def __init__(
        self,
        miso=-1,
        mosi=19,
        clk=18,
        cs=13,
        dc=12,
        rst=4,
        power=-1,
        backlight=15,
        backlight_on=0,
        backlight_pwm=False,
        power_on=0,
        spihost=esp.HSPI_HOST,
        spimode=0,
        mhz=80,
        hybrid=True,
        width=320,
        height=480,
        start_x=0,
        start_y=0,
        rot=ili9XXX.PORTRAIT,
        invert=False,
        double_buffer=False,
        half_duplex=True,
        asynchronous=False,
        initialize=True
    ):

        if rot in (ili9XXX.LANDSCAPE, ili9XXX.REVERSE_LANDSCAPE):
            width, height = height, width

        if lv.color_t.__SIZE__ == 4:
            pixel_format = 0x07  # 262K-Colors
        elif lv.color_t.__SIZE__ == 2:
            pixel_format = 0x05  # 65K-Colors  55??

        else:
            raise RuntimeError(
                'ST7796 micropython driver '
                'requires defining LV_COLOR_DEPTH=32 or LV_COLOR_DEPTH=16'
            )

        self.display_name = 'ST7796'
        self._thread_running = False
        self._new_backlight = 0

        self.init_cmds = [
            {'cmd': 0x01, 'delay': 120},  # SWRESET
            {'cmd': 0x11, 'delay': 120},  # SLPOUT
            {'cmd': 0xF0, 'data': bytes([0xC3])},  # CSCON  Enable extension command 2 partI
            {'cmd': 0xF0, 'data': bytes([0x96])},  # CSCON  Enable extension command 2 partII
            {'cmd': 0x36, 'data': bytes([self.madctl(ili9XXX.COLOR_MODE_BGR, rot, ili9XXX.ORIENTATION_TABLE)])},  # MADCTL
            # Interface_Pixel_Format
            {'cmd': 0x3A, 'data': bytes([pixel_format])},
            {'cmd': 0xB4, 'data': bytes([0x01])},  # INVTR  1-dot inversion
            {'cmd': 0xB6, 'data': bytes([
                0x80,  # Bypass
                0x02,  # Source Output Scan from S1 to S960, Gate Output scan from G1 to G480, scan cycle=2
                0x3B  # LCD Drive Line=8*(59+1)
            ])},  # DFC
            {'cmd': 0xE8, 'data': bytes([
                0x40,
                0x8A,
                0x00,
                0x00,
                0x29,  # Source eqaulizing period time= 22.5 us
                0x19,  # Timing for "Gate start"=25 (Tclk)
                0xA5,  # Timing for "Gate End"=37 (Tclk), Gate driver EQ function ON
                0x33
            ])},  # DOCA
            {'cmd': 0xC1, 'data': bytes([0x06])},  # PWR2 VAP(GVDD)=3.85+( vcom+vcom offset), VAN(GVCL)=-3.85+( vcom+vcom offset)
            {'cmd': 0xC2, 'data': bytes([0xA7])},  # PWR3 Source driving current level=low, Gamma driving current level=High
            {'cmd': 0xC5, 'data': bytes([0x18]), 'delay':120},  # VCMPCTL VCOM=0.9
            {'cmd': 0xE0, 'data': bytes([
                0xF0, 0x09, 0x0b, 0x06, 0x04, 0x15, 0x2F,
                0x54, 0x42, 0x3C, 0x17, 0x14, 0x18, 0x1B
            ])},  # PGC
            {'cmd': 0xE1, 'data': bytes([
                0xE0, 0x09, 0x0B, 0x06, 0x04, 0x03, 0x2B,
                0x43, 0x42, 0x3B, 0x16, 0x14, 0x17, 0x1B
            ]), 'delay':120},  # NGC
            {'cmd': 0xF0, 'data': bytes([0x3C])},  # CSCON  Disable extension command 2 partI
            {'cmd': 0xF0, 'data': bytes([0x69]), 'delay':120},  # CSCON Disable extension command 2 partII
            {'cmd': 0x29}  # DISPON
        ]

        super().__init__(
            miso=miso,
            mosi=mosi,
            clk=clk,
            cs=cs,
            dc=dc,
            rst=rst,
            power=power,
            backlight=backlight,
            backlight_on=backlight_on,
            power_on=power_on,
            spihost=spihost,
            spimode=spimode,
            mhz=mhz,
            hybrid=hybrid,
            width=width,
            height=height,
            start_x=start_x,
            start_y=start_y,
            invert=invert,
            factor=4,
            double_buffer=double_buffer,
            half_duplex=half_duplex,
            display_type=ili9XXX.DISPLAY_TYPE_ST7789,
            asynchronous=asynchronous,
            initialize=initialize,
            color_format=lv.COLOR_FORMAT.NATIVE_REVERSE
        )

        if backlight != -1:
            self._back_pin = Pin(backlight, Pin.OUT)

            if backlight_pwm:
                self._backlight_pwm = PWM(self._back_pin, freq=25000, duty_u16=0)
            else:
                self._backlight_pwm = None
        else:
            self._back_pin = None
            self._backlight_pwm = None

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

    def power_down(self):
        import time
        if self.power != -1:
            esp.gpio_set_level(self.power, 1 - self.power_on)

        self.backlight_level = 0

        self.send_cmd(0x10)  # sleep
        time.sleep_ms(120)



