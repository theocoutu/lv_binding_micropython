import lvgl as lv
import espidf as _espidf  # NOQA
import gc
from micropython import const


PORTRAIT = const(-1)
LANDSCAPE = const(-2)
REVERSE_PORTRAIT = const(-3)
REVERSE_LANDSCAPE = const(-4)


class touch_t:

    def __init__(self, last_x, last_y, current_state):
        self.last_x = last_x
        self.last_y = last_y
        self.current_state = current_state


class TouchBase(object):

    def __init__(self, init=True):
        if not lv.is_initialized():
            lv.init()

        disp = lv.disp_t.__cast__(None)  # NOQA
        width = disp.get_hor_res()
        height = disp.get_ver_res()

        if 0 in (width, height):
            raise RuntimeError(
                'The display needs to be initilized before the touch driver'
            )

        # -1 coordinates to designate it was never touched
        self._touch_input = touch_t(-1, -1, lv.INDEV_STATE.RELEASED)
        self.touch_count = 0
        self.touch_cycles = 0
        self.start_time_ptr = _espidf.C_Pointer()
        self.end_time_ptr = _espidf.C_Pointer()
        self._swap_xy = False
        self._invert_x = False
        self._invert_y = False
        self._rotation = PORTRAIT
        self.height = height
        self.width = width

        if init:
            self.init()

        indev_drv = lv.indev_drv_t()
        indev_drv.init()  # NOQA
        indev_drv.type = lv.INDEV_TYPE.POINTER
        indev_drv.read_cb = self.read
        indev_drv.register()  # NOQA
        gc.collect()

    @property
    def rotation(self):
        return self._rotation

    @rotation.setter
    def rotation(self, value):
        if value == PORTRAIT:
            self._swap_xy = False
            self._invert_x = False
            self._invert_y = False
        elif value == LANDSCAPE:
            self._swap_xy = True
            self._invert_x = False
            self._invert_y = False
        elif value == REVERSE_PORTRAIT:
            self._swap_xy = False
            self._invert_x = True
            self._invert_y = True
        elif value == REVERSE_LANDSCAPE:
            self._swap_xy = True
            self._invert_x = True
            self._invert_y = True
        else:
            raise RuntimeError('rotation is not supported')

    def init(self):
        pass

    def _get_coords(self):
        """
        This method MUST be overridden.

        It MUST return either`None` or the touch coordinates x, y
        LVGL does not support multitouch at this point in time and it also
        does not support pressure or z axis touches so only x, y is to be
        returned. Do not alter the x, y based on screen orientation.
        It is handled internally and the only thing that needs to be done to
        handle screen orientation changes is passing the rotation
        to the rotation property.
        """
        raise NotImplementedError

    def read(self, _, data):
        _espidf.get_ccount(self.start_time_ptr)
        coords = self._get_coords()
        _espidf.get_ccount(self.end_time_ptr)

        if self.end_time_ptr.int_val > self.start_time_ptr.int_val:
            self.touch_cycles += (
                self.end_time_ptr.int_val - self.start_time_ptr.int_val
            )
            self.touch_count += 1

        if coords is None:
            if self._touch_input.current_state != lv.INDEV_STATE.RELEASED:
                self._touch_input.current_state = lv.INDEV_STATE.RELEASED

            data.point.x = self._touch_input.last_x
            data.point.y = self._touch_input.last_y
            data.state = self._touch_input.current_state
        else:
            x, y = coords

            if self._swap_xy:
                x, y = y, x

            if self._invert_x:
                x = self.width - x

            if self._invert_y:
                y = self.height - y

            if y < 0:
                y += abs(y)

            self._touch_input.current_state = lv.INDEV_STATE.PRESSED
            self._touch_input.last_x = x
            self._touch_input.last_y = y

            data.state = lv.INDEV_STATE.PRESSED
            data.point.x = x
            data.point.y = y

        return False
