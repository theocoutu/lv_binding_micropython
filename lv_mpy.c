
/*
 * Auto-Generated file, DO NOT EDIT!
 *
 * Command line:
 * gen_mpy.py -X anim -X group -I pycparser/utils/fake_libc_include ../lv_objx/lv_arc.h ../lv_objx/lv_bar.h ../lv_objx/lv_btn.h ../lv_objx/lv_btnm.h ../lv_objx/lv_calendar.h ../lv_objx/lv_cb.h ../lv_objx/lv_chart.h ../lv_objx/lv_cont.h ../lv_objx/lv_ddlist.h ../lv_objx/lv_gauge.h ../lv_objx/lv_imgbtn.h ../lv_objx/lv_img.h ../lv_objx/lv_kb.h ../lv_objx/lv_label.h ../lv_objx/lv_led.h ../lv_objx/lv_line.h ../lv_objx/lv_list.h ../lv_objx/lv_lmeter.h ../lv_objx/lv_mbox.h ../lv_objx/lv_objx_templ.h ../lv_objx/lv_page.h ../lv_objx/lv_preload.h ../lv_objx/lv_roller.h ../lv_objx/lv_slider.h ../lv_objx/lv_sw.h ../lv_objx/lv_tabview.h ../lv_objx/lv_ta.h ../lv_objx/lv_win.h ../lv_themes/lv_theme.h
 *
 * Preprocessing command:
 * gcc -E -std=c99 -I pycparser/utils/fake_libc_include -include ../lv_objx/lv_arc.h -include ../lv_objx/lv_bar.h -include ../lv_objx/lv_btn.h -include ../lv_objx/lv_btnm.h -include ../lv_objx/lv_calendar.h -include ../lv_objx/lv_cb.h -include ../lv_objx/lv_chart.h -include ../lv_objx/lv_cont.h -include ../lv_objx/lv_ddlist.h -include ../lv_objx/lv_gauge.h -include ../lv_objx/lv_imgbtn.h -include ../lv_objx/lv_img.h -include ../lv_objx/lv_kb.h -include ../lv_objx/lv_label.h -include ../lv_objx/lv_led.h -include ../lv_objx/lv_line.h -include ../lv_objx/lv_list.h -include ../lv_objx/lv_lmeter.h -include ../lv_objx/lv_mbox.h -include ../lv_objx/lv_objx_templ.h -include ../lv_objx/lv_page.h -include ../lv_objx/lv_preload.h -include ../lv_objx/lv_roller.h -include ../lv_objx/lv_slider.h -include ../lv_objx/lv_sw.h -include ../lv_objx/lv_tabview.h -include ../lv_objx/lv_ta.h -include ../lv_objx/lv_win.h -include ../lv_themes/lv_theme.h ../lv_objx/lv_arc.h
 *
 * Generating Objects: obj(None), arc(obj), cont(obj), btn(cont), label(obj), bar(obj), btnm(obj), cb(btn), line(obj), chart(obj), page(cont), ddlist(page), lmeter(obj), gauge(lmeter), img(obj), kb(btnm), led(obj), list(page), mbox(cont), preload(arc), roller(ddlist), slider(bar), sw(slider), win(obj), tabview(obj), ta(page)
 */

/*
 * Mpy includes
 */

#include <stdlib.h>
#include <string.h>
#include "py/obj.h"
#include "py/runtime.h"

/*
 * lvgl includes
 */

#include "../lv_objx/lv_arc.h"
#include "../lv_objx/lv_bar.h"
#include "../lv_objx/lv_btn.h"
#include "../lv_objx/lv_btnm.h"
#include "../lv_objx/lv_calendar.h"
#include "../lv_objx/lv_cb.h"
#include "../lv_objx/lv_chart.h"
#include "../lv_objx/lv_cont.h"
#include "../lv_objx/lv_ddlist.h"
#include "../lv_objx/lv_gauge.h"
#include "../lv_objx/lv_imgbtn.h"
#include "../lv_objx/lv_img.h"
#include "../lv_objx/lv_kb.h"
#include "../lv_objx/lv_label.h"
#include "../lv_objx/lv_led.h"
#include "../lv_objx/lv_line.h"
#include "../lv_objx/lv_list.h"
#include "../lv_objx/lv_lmeter.h"
#include "../lv_objx/lv_mbox.h"
#include "../lv_objx/lv_objx_templ.h"
#include "../lv_objx/lv_page.h"
#include "../lv_objx/lv_preload.h"
#include "../lv_objx/lv_roller.h"
#include "../lv_objx/lv_slider.h"
#include "../lv_objx/lv_sw.h"
#include "../lv_objx/lv_tabview.h"
#include "../lv_objx/lv_ta.h"
#include "../lv_objx/lv_win.h"
#include "../lv_themes/lv_theme.h"


/*
 * Helper functions
 */

typedef lv_obj_t* (*lv_create)(lv_obj_t * par, const lv_obj_t * copy);

typedef struct mp_lv_obj_t {
    mp_obj_base_t base;
    lv_obj_t *lv_obj;
    mp_obj_t *action;
} mp_lv_obj_t;

STATIC inline lv_obj_t *mp_to_lv(mp_obj_t *mp_obj)
{
    if (mp_obj == NULL || mp_obj == mp_const_none) return NULL;
    mp_lv_obj_t *mp_lv_obj = MP_OBJ_TO_PTR(mp_obj);
    return mp_lv_obj->lv_obj;
}

STATIC inline mp_obj_t *mp_to_lv_action(mp_obj_t *mp_obj)
{
    if (mp_obj == NULL || mp_obj == mp_const_none) return NULL;
    mp_lv_obj_t *mp_lv_obj = MP_OBJ_TO_PTR(mp_obj);
    return mp_lv_obj->action;
}

STATIC inline void set_action(mp_obj_t *mp_obj, mp_obj_t *action)
{
    if (mp_obj == NULL || mp_obj == mp_const_none) return;
    mp_lv_obj_t *mp_lv_obj = MP_OBJ_TO_PTR(mp_obj);
    mp_lv_obj->action = action;
}

STATIC inline const mp_obj_type_t *get_BaseObj_type();

STATIC inline mp_obj_t *lv_to_mp(lv_obj_t *lv_obj)
{
    mp_lv_obj_t *self = lv_obj_get_free_ptr(lv_obj);
    if (!self) 
    {
        self = m_new_obj(mp_lv_obj_t);
        *self = (mp_lv_obj_t){
            .base = {get_BaseObj_type()},
            .lv_obj = lv_obj,
            .action = NULL
        };
    }
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t make_new(
    lv_create create,
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 1, 2, false);
    mp_lv_obj_t *self = m_new_obj(mp_lv_obj_t);
    lv_obj_t *parent = mp_to_lv(args[0]);
    lv_obj_t *copy = n_args > 1? mp_to_lv(args[1]): NULL;
    *self = (mp_lv_obj_t){
        .base = {type}, 
        .lv_obj = create(parent, copy),
        .action = NULL
    };
    lv_obj_set_free_ptr(self->lv_obj, self);
    return MP_OBJ_FROM_PTR(self);
}

STATIC inline mp_obj_t convert_to_bool(bool b)
{
    return b? mp_const_true: mp_const_false;
}

STATIC inline mp_obj_t convert_to_str(const char *str)
{
    return mp_obj_new_str(str, strlen(str));
}

STATIC void field_not_found(qstr struct_name, qstr field_name)
{
    nlr_raise(
        mp_obj_new_exception_msg_varg(
            &mp_type_SyntaxError, 
            "Cannot access field %s. Field does not exist in struct %s!", 
            qstr_str(field_name), 
            qstr_str(struct_name)));
}

typedef struct mp_lv_struct_t
{
    mp_obj_base_t base;
    bool allocated;
    void *data;
} mp_lv_struct_t;

STATIC inline size_t get_lv_struct_size(const mp_obj_type_t *type)
{
    mp_obj_t size_obj = mp_obj_dict_get(type->locals_dict, MP_OBJ_NEW_QSTR(MP_QSTR_SIZE));
    return (size_t)mp_obj_int_get_checked(size_obj);
}

STATIC mp_obj_t make_new_lv_struct(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    size_t size = get_lv_struct_size(type);
    mp_arg_check_num(n_args, n_kw, 0, 1, false);
    mp_lv_struct_t *self = m_new_obj(mp_lv_struct_t);
    lv_obj_t *copy = n_args > 0? mp_to_lv(args[0]): NULL;
    if (!MP_OBJ_IS_OBJ(copy)) nlr_raise(
            mp_obj_new_exception_msg(
                &mp_type_SyntaxError, "Copy argument is not an object!"));

    *self = (mp_lv_struct_t){
        .base = {type}, 
        .allocated = true,
        .data = malloc(size)
    };
    if (copy) {
        mp_lv_struct_t *other = MP_OBJ_TO_PTR(copy);
        if (self->base.type != other->base.type) nlr_raise(
            mp_obj_new_exception_msg(
                &mp_type_SyntaxError, "Incompatible lv_struct!"));
        memcpy(self->data, other->data, size);
    }
    return MP_OBJ_FROM_PTR(self);
}

// Reference an existing lv struct (or part of it)

STATIC mp_obj_t lv_to_mp_struct(const mp_obj_type_t *type, void *lv_struct)
{
    mp_lv_struct_t *self = m_new_obj(mp_lv_struct_t);
    *self = (mp_lv_struct_t){
        .base = {type},
        .allocated = false,
        .data = lv_struct
    };
    return MP_OBJ_FROM_PTR(self);
}

STATIC mp_obj_t delete_lv_struct(mp_obj_t self_in) 
{
    mp_lv_struct_t *self = self_in;
    if (self->allocated) free(self->data);
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_1(delete_lv_struct_obj, delete_lv_struct);

STATIC void mp_blob_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl Blob");
}

STATIC const mp_obj_type_t mp_blob_type = {
    { &mp_type_type },
    .name = MP_QSTR_Blob,
    .print = mp_blob_print,
    //.make_new = make_new_blob,
    //.attr = mp_blob_attr,
    //.locals_dict = (mp_obj_dict_t*)&mp_blob_locals_dict,
};

STATIC void* mp_to_ptr(mp_obj_t self_in)
{
    mp_lv_struct_t *self = self_in;
    if ((!MP_OBJ_IS_OBJ(self_in)) || self->base.type != &mp_blob_type){
        nlr_raise(
            mp_obj_new_exception_msg(
                &mp_type_SyntaxError, "Incompatible type!"));
    }
    return self->data;
}

STATIC inline mp_obj_t ptr_to_mp(void *data)
{
    return lv_to_mp_struct(&mp_blob_type, data);
}



    
/*
 * lvgl LV_TA_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_TA_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SB), MP_ROM_PTR(MP_ROM_INT(LV_TA_STYLE_SB)) },
    { MP_ROM_QSTR(MP_QSTR_CURSOR), MP_ROM_PTR(MP_ROM_INT(LV_TA_STYLE_CURSOR)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_TA_STYLE_BG)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_TA_STYLE_locals_dict, LV_TA_STYLE_locals_dict_table);

STATIC void LV_TA_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_TA_STYLE");
}



STATIC const mp_obj_type_t mp_LV_TA_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_TA_STYLE,
    .print = LV_TA_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_TA_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_BORDER object definitions
 */

STATIC const mp_rom_map_elem_t LV_BORDER_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_TOP), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_TOP)) },
    { MP_ROM_QSTR(MP_QSTR_FULL), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_FULL)) },
    { MP_ROM_QSTR(MP_QSTR_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_BORDER_BOTTOM)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BORDER_locals_dict, LV_BORDER_locals_dict_table);

STATIC void LV_BORDER_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BORDER");
}



STATIC const mp_obj_type_t mp_LV_BORDER_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BORDER,
    .print = LV_BORDER_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BORDER_locals_dict,
};
    

    
/*
 * lvgl LV_TABVIEW_BTNS_POS object definitions
 */

STATIC const mp_rom_map_elem_t LV_TABVIEW_BTNS_POS_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_BTNS_POS_BOTTOM)) },
    { MP_ROM_QSTR(MP_QSTR_TOP), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_BTNS_POS_TOP)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_TABVIEW_BTNS_POS_locals_dict, LV_TABVIEW_BTNS_POS_locals_dict_table);

STATIC void LV_TABVIEW_BTNS_POS_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_TABVIEW_BTNS_POS");
}



STATIC const mp_obj_type_t mp_LV_TABVIEW_BTNS_POS_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_TABVIEW_BTNS_POS,
    .print = LV_TABVIEW_BTNS_POS_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_TABVIEW_BTNS_POS_locals_dict,
};
    

    
/*
 * lvgl LV_DDLIST_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_DDLIST_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SB), MP_ROM_PTR(MP_ROM_INT(LV_DDLIST_STYLE_SB)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_DDLIST_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_SEL), MP_ROM_PTR(MP_ROM_INT(LV_DDLIST_STYLE_SEL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_DDLIST_STYLE_locals_dict, LV_DDLIST_STYLE_locals_dict_table);

STATIC void LV_DDLIST_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_DDLIST_STYLE");
}



STATIC const mp_obj_type_t mp_LV_DDLIST_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_DDLIST_STYLE,
    .print = LV_DDLIST_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_DDLIST_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_ANIM object definitions
 */

STATIC const mp_rom_map_elem_t LV_ANIM_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_GROW_V), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_GROW_V)) },
    { MP_ROM_QSTR(MP_QSTR_FLOAT_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_FLOAT_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_FLOAT_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_FLOAT_BOTTOM)) },
    { MP_ROM_QSTR(MP_QSTR_FLOAT_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_FLOAT_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_GROW_H), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_GROW_H)) },
    { MP_ROM_QSTR(MP_QSTR_FLOAT_TOP), MP_ROM_PTR(MP_ROM_INT(LV_ANIM_FLOAT_TOP)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_ANIM_locals_dict, LV_ANIM_locals_dict_table);

STATIC void LV_ANIM_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_ANIM");
}



STATIC const mp_obj_type_t mp_LV_ANIM_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_ANIM,
    .print = LV_ANIM_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_ANIM_locals_dict,
};
    

    
/*
 * lvgl LV_LAYOUT object definitions
 */

STATIC const mp_rom_map_elem_t LV_LAYOUT_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_COL_R), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_COL_R)) },
    { MP_ROM_QSTR(MP_QSTR_ROW_B), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_ROW_B)) },
    { MP_ROM_QSTR(MP_QSTR_GRID), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_GRID)) },
    { MP_ROM_QSTR(MP_QSTR_ROW_M), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_ROW_M)) },
    { MP_ROM_QSTR(MP_QSTR_PRETTY), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_PRETTY)) },
    { MP_ROM_QSTR(MP_QSTR_COL_L), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_COL_L)) },
    { MP_ROM_QSTR(MP_QSTR_COL_M), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_COL_M)) },
    { MP_ROM_QSTR(MP_QSTR_OFF), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_OFF)) },
    { MP_ROM_QSTR(MP_QSTR_CENTER), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_CENTER)) },
    { MP_ROM_QSTR(MP_QSTR_ROW_T), MP_ROM_PTR(MP_ROM_INT(LV_LAYOUT_ROW_T)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_LAYOUT_locals_dict, LV_LAYOUT_locals_dict_table);

STATIC void LV_LAYOUT_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_LAYOUT");
}



STATIC const mp_obj_type_t mp_LV_LAYOUT_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_LAYOUT,
    .print = LV_LAYOUT_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_LAYOUT_locals_dict,
};
    

    
/*
 * lvgl LV_IMG_CF object definitions
 */

STATIC const mp_rom_map_elem_t LV_IMG_CF_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_ALPHA_2BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_ALPHA_2BIT)) },
    { MP_ROM_QSTR(MP_QSTR_ALPHA_8BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_ALPHA_8BIT)) },
    { MP_ROM_QSTR(MP_QSTR_INDEXED_4BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_INDEXED_4BIT)) },
    { MP_ROM_QSTR(MP_QSTR_ALPHA_4BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_ALPHA_4BIT)) },
    { MP_ROM_QSTR(MP_QSTR_INDEXED_8BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_INDEXED_8BIT)) },
    { MP_ROM_QSTR(MP_QSTR_TRUE_COLOR), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_TRUE_COLOR)) },
    { MP_ROM_QSTR(MP_QSTR_TRUE_COLOR_ALPHA), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_TRUE_COLOR_ALPHA)) },
    { MP_ROM_QSTR(MP_QSTR_ALPHA_1BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_ALPHA_1BIT)) },
    { MP_ROM_QSTR(MP_QSTR_TRUE_COLOR_CHROMA_KEYED), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_TRUE_COLOR_CHROMA_KEYED)) },
    { MP_ROM_QSTR(MP_QSTR_INDEXED_1BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_INDEXED_1BIT)) },
    { MP_ROM_QSTR(MP_QSTR_INDEXED_2BIT), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_INDEXED_2BIT)) },
    { MP_ROM_QSTR(MP_QSTR_RAW_ALPHA), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_RAW_ALPHA)) },
    { MP_ROM_QSTR(MP_QSTR_RAW), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_RAW)) },
    { MP_ROM_QSTR(MP_QSTR_UNKOWN), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_UNKOWN)) },
    { MP_ROM_QSTR(MP_QSTR_RAW_CHROMA_KEYED), MP_ROM_PTR(MP_ROM_INT(LV_IMG_CF_RAW_CHROMA_KEYED)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_IMG_CF_locals_dict, LV_IMG_CF_locals_dict_table);

STATIC void LV_IMG_CF_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_IMG_CF");
}



STATIC const mp_obj_type_t mp_LV_IMG_CF_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_IMG_CF,
    .print = LV_IMG_CF_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_IMG_CF_locals_dict,
};
    

    
/*
 * lvgl LV_INDEV_TYPE object definitions
 */

STATIC const mp_rom_map_elem_t LV_INDEV_TYPE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_POINTER), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_TYPE_POINTER)) },
    { MP_ROM_QSTR(MP_QSTR_BUTTON), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_TYPE_BUTTON)) },
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_TYPE_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_ENCODER), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_TYPE_ENCODER)) },
    { MP_ROM_QSTR(MP_QSTR_KEYPAD), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_TYPE_KEYPAD)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_INDEV_TYPE_locals_dict, LV_INDEV_TYPE_locals_dict_table);

STATIC void LV_INDEV_TYPE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_INDEV_TYPE");
}



STATIC const mp_obj_type_t mp_LV_INDEV_TYPE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_INDEV_TYPE,
    .print = LV_INDEV_TYPE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_INDEV_TYPE_locals_dict,
};
    

    
/*
 * lvgl LV_FS_RES object definitions
 */

STATIC const mp_rom_map_elem_t LV_FS_RES_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_TOUT), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_TOUT)) },
    { MP_ROM_QSTR(MP_QSTR_DENIED), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_DENIED)) },
    { MP_ROM_QSTR(MP_QSTR_UNKNOWN), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_UNKNOWN)) },
    { MP_ROM_QSTR(MP_QSTR_FS_ERR), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_FS_ERR)) },
    { MP_ROM_QSTR(MP_QSTR_LOCKED), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_LOCKED)) },
    { MP_ROM_QSTR(MP_QSTR_NOT_EX), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_NOT_EX)) },
    { MP_ROM_QSTR(MP_QSTR_HW_ERR), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_HW_ERR)) },
    { MP_ROM_QSTR(MP_QSTR_INV_PARAM), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_INV_PARAM)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_OF_MEM), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_OUT_OF_MEM)) },
    { MP_ROM_QSTR(MP_QSTR_NOT_IMP), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_NOT_IMP)) },
    { MP_ROM_QSTR(MP_QSTR_FULL), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_FULL)) },
    { MP_ROM_QSTR(MP_QSTR_OK), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_OK)) },
    { MP_ROM_QSTR(MP_QSTR_BUSY), MP_ROM_PTR(MP_ROM_INT(LV_FS_RES_BUSY)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_FS_RES_locals_dict, LV_FS_RES_locals_dict_table);

STATIC void LV_FS_RES_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_FS_RES");
}



STATIC const mp_obj_type_t mp_LV_FS_RES_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_FS_RES,
    .print = LV_FS_RES_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_FS_RES_locals_dict,
};
    

    
/*
 * lvgl LV_WIN_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_WIN_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_CONTENT_SCRL), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_CONTENT_SCRL)) },
    { MP_ROM_QSTR(MP_QSTR_SB), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_SB)) },
    { MP_ROM_QSTR(MP_QSTR_CONTENT_BG), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_CONTENT_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_HEADER), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_HEADER)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_BTN_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_WIN_STYLE_BTN_REL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_WIN_STYLE_locals_dict, LV_WIN_STYLE_locals_dict_table);

STATIC void LV_WIN_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_WIN_STYLE");
}



STATIC const mp_obj_type_t mp_LV_WIN_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_WIN_STYLE,
    .print = LV_WIN_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_WIN_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_SB_MODE object definitions
 */

STATIC const mp_rom_map_elem_t LV_SB_MODE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_HIDE), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_HIDE)) },
    { MP_ROM_QSTR(MP_QSTR_UNHIDE), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_UNHIDE)) },
    { MP_ROM_QSTR(MP_QSTR_ON), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_ON)) },
    { MP_ROM_QSTR(MP_QSTR_DRAG), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_DRAG)) },
    { MP_ROM_QSTR(MP_QSTR_OFF), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_OFF)) },
    { MP_ROM_QSTR(MP_QSTR_AUTO), MP_ROM_PTR(MP_ROM_INT(LV_SB_MODE_AUTO)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_SB_MODE_locals_dict, LV_SB_MODE_locals_dict_table);

STATIC void LV_SB_MODE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_SB_MODE");
}



STATIC const mp_obj_type_t mp_LV_SB_MODE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_SB_MODE,
    .print = LV_SB_MODE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_SB_MODE_locals_dict,
};
    

    
/*
 * lvgl LV_SIGNAL object definitions
 */

STATIC const mp_rom_map_elem_t LV_SIGNAL_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_PRESS_LOST), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_PRESS_LOST)) },
    { MP_ROM_QSTR(MP_QSTR_DRAG_BEGIN), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_DRAG_BEGIN)) },
    { MP_ROM_QSTR(MP_QSTR_DEFOCUS), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_DEFOCUS)) },
    { MP_ROM_QSTR(MP_QSTR_RELEASED), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_RELEASED)) },
    { MP_ROM_QSTR(MP_QSTR_STYLE_CHG), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_STYLE_CHG)) },
    { MP_ROM_QSTR(MP_QSTR_GET_EDITABLE), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_GET_EDITABLE)) },
    { MP_ROM_QSTR(MP_QSTR_CHILD_CHG), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_CHILD_CHG)) },
    { MP_ROM_QSTR(MP_QSTR_CLEANUP), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_CLEANUP)) },
    { MP_ROM_QSTR(MP_QSTR_DRAG_END), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_DRAG_END)) },
    { MP_ROM_QSTR(MP_QSTR_FOCUS), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_FOCUS)) },
    { MP_ROM_QSTR(MP_QSTR_PRESSING), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_PRESSING)) },
    { MP_ROM_QSTR(MP_QSTR_LONG_PRESS), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_LONG_PRESS)) },
    { MP_ROM_QSTR(MP_QSTR_PRESSED), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_PRESSED)) },
    { MP_ROM_QSTR(MP_QSTR_LONG_PRESS_REP), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_LONG_PRESS_REP)) },
    { MP_ROM_QSTR(MP_QSTR_CORD_CHG), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_CORD_CHG)) },
    { MP_ROM_QSTR(MP_QSTR_GET_TYPE), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_GET_TYPE)) },
    { MP_ROM_QSTR(MP_QSTR_REFR_EXT_SIZE), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_REFR_EXT_SIZE)) },
    { MP_ROM_QSTR(MP_QSTR_CONTROLL), MP_ROM_PTR(MP_ROM_INT(LV_SIGNAL_CONTROLL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_SIGNAL_locals_dict, LV_SIGNAL_locals_dict_table);

STATIC void LV_SIGNAL_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_SIGNAL");
}



STATIC const mp_obj_type_t mp_LV_SIGNAL_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_SIGNAL,
    .print = LV_SIGNAL_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_SIGNAL_locals_dict,
};
    

    
/*
 * lvgl LV_BTN_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_BTN_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STYLE_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STYLE_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STYLE_REL)) },
    { MP_ROM_QSTR(MP_QSTR_INA), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STYLE_INA)) },
    { MP_ROM_QSTR(MP_QSTR_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STYLE_PR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BTN_STYLE_locals_dict, LV_BTN_STYLE_locals_dict_table);

STATIC void LV_BTN_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BTN_STYLE");
}



STATIC const mp_obj_type_t mp_LV_BTN_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BTN_STYLE,
    .print = LV_BTN_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BTN_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_KB_MODE object definitions
 */

STATIC const mp_rom_map_elem_t LV_KB_MODE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_TEXT), MP_ROM_PTR(MP_ROM_INT(LV_KB_MODE_TEXT)) },
    { MP_ROM_QSTR(MP_QSTR_NUM), MP_ROM_PTR(MP_ROM_INT(LV_KB_MODE_NUM)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_KB_MODE_locals_dict, LV_KB_MODE_locals_dict_table);

STATIC void LV_KB_MODE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_KB_MODE");
}



STATIC const mp_obj_type_t mp_LV_KB_MODE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_KB_MODE,
    .print = LV_KB_MODE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_KB_MODE_locals_dict,
};
    

    
/*
 * lvgl LV_BAR_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_BAR_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_INDIC), MP_ROM_PTR(MP_ROM_INT(LV_BAR_STYLE_INDIC)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_BAR_STYLE_BG)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BAR_STYLE_locals_dict, LV_BAR_STYLE_locals_dict_table);

STATIC void LV_BAR_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BAR_STYLE");
}



STATIC const mp_obj_type_t mp_LV_BAR_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BAR_STYLE,
    .print = LV_BAR_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BAR_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_ROLLER_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_ROLLER_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_ROLLER_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_SEL), MP_ROM_PTR(MP_ROM_INT(LV_ROLLER_STYLE_SEL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_ROLLER_STYLE_locals_dict, LV_ROLLER_STYLE_locals_dict_table);

STATIC void LV_ROLLER_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_ROLLER_STYLE");
}



STATIC const mp_obj_type_t mp_LV_ROLLER_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_ROLLER_STYLE,
    .print = LV_ROLLER_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_ROLLER_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_SW_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_SW_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_KNOB_ON), MP_ROM_PTR(MP_ROM_INT(LV_SW_STYLE_KNOB_ON)) },
    { MP_ROM_QSTR(MP_QSTR_KNOB_OFF), MP_ROM_PTR(MP_ROM_INT(LV_SW_STYLE_KNOB_OFF)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_SW_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_INDIC), MP_ROM_PTR(MP_ROM_INT(LV_SW_STYLE_INDIC)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_SW_STYLE_locals_dict, LV_SW_STYLE_locals_dict_table);

STATIC void LV_SW_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_SW_STYLE");
}



STATIC const mp_obj_type_t mp_LV_SW_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_SW_STYLE,
    .print = LV_SW_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_SW_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_IMG_SRC object definitions
 */

STATIC const mp_rom_map_elem_t LV_IMG_SRC_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SYMBOL), MP_ROM_PTR(MP_ROM_INT(LV_IMG_SRC_SYMBOL)) },
    { MP_ROM_QSTR(MP_QSTR_FILE), MP_ROM_PTR(MP_ROM_INT(LV_IMG_SRC_FILE)) },
    { MP_ROM_QSTR(MP_QSTR_UNKNOWN), MP_ROM_PTR(MP_ROM_INT(LV_IMG_SRC_UNKNOWN)) },
    { MP_ROM_QSTR(MP_QSTR_VARIABLE), MP_ROM_PTR(MP_ROM_INT(LV_IMG_SRC_VARIABLE)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_IMG_SRC_locals_dict, LV_IMG_SRC_locals_dict_table);

STATIC void LV_IMG_SRC_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_IMG_SRC");
}



STATIC const mp_obj_type_t mp_LV_IMG_SRC_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_IMG_SRC,
    .print = LV_IMG_SRC_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_IMG_SRC_locals_dict,
};
    

    
/*
 * lvgl LV_PRELOAD_TYPE_SPINNING object definitions
 */

STATIC const mp_rom_map_elem_t LV_PRELOAD_TYPE_SPINNING_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_ARC), MP_ROM_PTR(MP_ROM_INT(LV_PRELOAD_TYPE_SPINNING_ARC)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_PRELOAD_TYPE_SPINNING_locals_dict, LV_PRELOAD_TYPE_SPINNING_locals_dict_table);

STATIC void LV_PRELOAD_TYPE_SPINNING_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_PRELOAD_TYPE_SPINNING");
}



STATIC const mp_obj_type_t mp_LV_PRELOAD_TYPE_SPINNING_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_PRELOAD_TYPE_SPINNING,
    .print = LV_PRELOAD_TYPE_SPINNING_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_PRELOAD_TYPE_SPINNING_locals_dict,
};
    

    
/*
 * lvgl LV_ALIGN object definitions
 */

STATIC const mp_rom_map_elem_t LV_ALIGN_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_OUT_LEFT_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_LEFT_BOTTOM)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_TOP_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_TOP_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_IN_RIGHT_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_RIGHT_MID)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_RIGHT_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_RIGHT_MID)) },
    { MP_ROM_QSTR(MP_QSTR_IN_BOTTOM_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_BOTTOM_MID)) },
    { MP_ROM_QSTR(MP_QSTR_CENTER), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_CENTER)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_RIGHT_TOP), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_RIGHT_TOP)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_BOTTOM_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_BOTTOM_MID)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_LEFT_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_LEFT_MID)) },
    { MP_ROM_QSTR(MP_QSTR_IN_TOP_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_TOP_MID)) },
    { MP_ROM_QSTR(MP_QSTR_IN_BOTTOM_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_BOTTOM_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_IN_TOP_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_TOP_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_TOP_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_TOP_MID)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_BOTTOM_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_BOTTOM_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_RIGHT_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_RIGHT_BOTTOM)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_LEFT_TOP), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_LEFT_TOP)) },
    { MP_ROM_QSTR(MP_QSTR_IN_BOTTOM_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_BOTTOM_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_TOP_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_TOP_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_OUT_BOTTOM_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_OUT_BOTTOM_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_IN_TOP_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_TOP_LEFT)) },
    { MP_ROM_QSTR(MP_QSTR_IN_LEFT_MID), MP_ROM_PTR(MP_ROM_INT(LV_ALIGN_IN_LEFT_MID)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_ALIGN_locals_dict, LV_ALIGN_locals_dict_table);

STATIC void LV_ALIGN_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_ALIGN");
}



STATIC const mp_obj_type_t mp_LV_ALIGN_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_ALIGN,
    .print = LV_ALIGN_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_ALIGN_locals_dict,
};
    

    
/*
 * lvgl LV_INDEV_STATE object definitions
 */

STATIC const mp_rom_map_elem_t LV_INDEV_STATE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_PR), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_STATE_PR)) },
    { MP_ROM_QSTR(MP_QSTR_REL), MP_ROM_PTR(MP_ROM_INT(LV_INDEV_STATE_REL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_INDEV_STATE_locals_dict, LV_INDEV_STATE_locals_dict_table);

STATIC void LV_INDEV_STATE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_INDEV_STATE");
}



STATIC const mp_obj_type_t mp_LV_INDEV_STATE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_INDEV_STATE,
    .print = LV_INDEV_STATE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_INDEV_STATE_locals_dict,
};
    

    
/*
 * lvgl LV_CURSOR object definitions
 */

STATIC const mp_rom_map_elem_t LV_CURSOR_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_UNDERLINE), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_UNDERLINE)) },
    { MP_ROM_QSTR(MP_QSTR_BLOCK), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_BLOCK)) },
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_HIDDEN), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_HIDDEN)) },
    { MP_ROM_QSTR(MP_QSTR_OUTLINE), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_OUTLINE)) },
    { MP_ROM_QSTR(MP_QSTR_LINE), MP_ROM_PTR(MP_ROM_INT(LV_CURSOR_LINE)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_CURSOR_locals_dict, LV_CURSOR_locals_dict_table);

STATIC void LV_CURSOR_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_CURSOR");
}



STATIC const mp_obj_type_t mp_LV_CURSOR_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_CURSOR,
    .print = LV_CURSOR_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_CURSOR_locals_dict,
};
    

    
/*
 * lvgl LV_LIST_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_LIST_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_SCRL), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_SCRL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_INA), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BTN_INA)) },
    { MP_ROM_QSTR(MP_QSTR_SB), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_SB)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BTN_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BTN_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BTN_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_LIST_STYLE_BTN_TGL_REL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_LIST_STYLE_locals_dict, LV_LIST_STYLE_locals_dict_table);

STATIC void LV_LIST_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_LIST_STYLE");
}



STATIC const mp_obj_type_t mp_LV_LIST_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_LIST_STYLE,
    .print = LV_LIST_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_LIST_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_SHADOW object definitions
 */

STATIC const mp_rom_map_elem_t LV_SHADOW_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BOTTOM), MP_ROM_PTR(MP_ROM_INT(LV_SHADOW_BOTTOM)) },
    { MP_ROM_QSTR(MP_QSTR_FULL), MP_ROM_PTR(MP_ROM_INT(LV_SHADOW_FULL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_SHADOW_locals_dict, LV_SHADOW_locals_dict_table);

STATIC void LV_SHADOW_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_SHADOW");
}



STATIC const mp_obj_type_t mp_LV_SHADOW_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_SHADOW,
    .print = LV_SHADOW_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_SHADOW_locals_dict,
};
    

    
/*
 * lvgl LV_TABVIEW_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_TABVIEW_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BTN_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BTN_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_BG), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BTN_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BTN_REL)) },
    { MP_ROM_QSTR(MP_QSTR_INDIC), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_INDIC)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_TABVIEW_STYLE_BTN_PR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_TABVIEW_STYLE_locals_dict, LV_TABVIEW_STYLE_locals_dict_table);

STATIC void LV_TABVIEW_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_TABVIEW_STYLE");
}



STATIC const mp_obj_type_t mp_LV_TABVIEW_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_TABVIEW_STYLE,
    .print = LV_TABVIEW_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_TABVIEW_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_MBOX_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_MBOX_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_BG), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_INA), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_INA)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_MBOX_STYLE_BTN_REL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_MBOX_STYLE_locals_dict, LV_MBOX_STYLE_locals_dict_table);

STATIC void LV_MBOX_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_MBOX_STYLE");
}



STATIC const mp_obj_type_t mp_LV_MBOX_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_MBOX_STYLE,
    .print = LV_MBOX_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_MBOX_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_TXT_FLAG object definitions
 */

STATIC const mp_rom_map_elem_t LV_TXT_FLAG_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_CENTER), MP_ROM_PTR(MP_ROM_INT(LV_TXT_FLAG_CENTER)) },
    { MP_ROM_QSTR(MP_QSTR_RECOLOR), MP_ROM_PTR(MP_ROM_INT(LV_TXT_FLAG_RECOLOR)) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_TXT_FLAG_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_TXT_FLAG_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_EXPAND), MP_ROM_PTR(MP_ROM_INT(LV_TXT_FLAG_EXPAND)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_TXT_FLAG_locals_dict, LV_TXT_FLAG_locals_dict_table);

STATIC void LV_TXT_FLAG_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_TXT_FLAG");
}



STATIC const mp_obj_type_t mp_LV_TXT_FLAG_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_TXT_FLAG,
    .print = LV_TXT_FLAG_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_TXT_FLAG_locals_dict,
};
    

    
/*
 * lvgl LV_BTN_ACTION object definitions
 */

STATIC const mp_rom_map_elem_t LV_BTN_ACTION_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_NUM), MP_ROM_PTR(MP_ROM_INT(LV_BTN_ACTION_NUM)) },
    { MP_ROM_QSTR(MP_QSTR_LONG_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_ACTION_LONG_PR)) },
    { MP_ROM_QSTR(MP_QSTR_LONG_PR_REPEAT), MP_ROM_PTR(MP_ROM_INT(LV_BTN_ACTION_LONG_PR_REPEAT)) },
    { MP_ROM_QSTR(MP_QSTR_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_ACTION_PR)) },
    { MP_ROM_QSTR(MP_QSTR_CLICK), MP_ROM_PTR(MP_ROM_INT(LV_BTN_ACTION_CLICK)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BTN_ACTION_locals_dict, LV_BTN_ACTION_locals_dict_table);

STATIC void LV_BTN_ACTION_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BTN_ACTION");
}



STATIC const mp_obj_type_t mp_LV_BTN_ACTION_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BTN_ACTION,
    .print = LV_BTN_ACTION_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BTN_ACTION_locals_dict,
};
    

    
/*
 * lvgl LV_RES object definitions
 */

STATIC const mp_rom_map_elem_t LV_RES_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_OK), MP_ROM_PTR(MP_ROM_INT(LV_RES_OK)) },
    { MP_ROM_QSTR(MP_QSTR_INV), MP_ROM_PTR(MP_ROM_INT(LV_RES_INV)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_RES_locals_dict, LV_RES_locals_dict_table);

STATIC void LV_RES_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_RES");
}



STATIC const mp_obj_type_t mp_LV_RES_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_RES,
    .print = LV_RES_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_RES_locals_dict,
};
    

    
/*
 * lvgl LV_FS_MODE object definitions
 */

STATIC const mp_rom_map_elem_t LV_FS_MODE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_RD), MP_ROM_PTR(MP_ROM_INT(LV_FS_MODE_RD)) },
    { MP_ROM_QSTR(MP_QSTR_WR), MP_ROM_PTR(MP_ROM_INT(LV_FS_MODE_WR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_FS_MODE_locals_dict, LV_FS_MODE_locals_dict_table);

STATIC void LV_FS_MODE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_FS_MODE");
}



STATIC const mp_obj_type_t mp_LV_FS_MODE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_FS_MODE,
    .print = LV_FS_MODE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_FS_MODE_locals_dict,
};
    

    
/*
 * lvgl LV_BTNM_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_BTNM_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BTN_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_INA), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BTN_INA)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BTN_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BTN_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTNM_STYLE_BTN_PR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BTNM_STYLE_locals_dict, LV_BTNM_STYLE_locals_dict_table);

STATIC void LV_BTNM_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BTNM_STYLE");
}



STATIC const mp_obj_type_t mp_LV_BTNM_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BTNM_STYLE,
    .print = LV_BTNM_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BTNM_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_PAGE_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_PAGE_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SB), MP_ROM_PTR(MP_ROM_INT(LV_PAGE_STYLE_SB)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_PAGE_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_SCRL), MP_ROM_PTR(MP_ROM_INT(LV_PAGE_STYLE_SCRL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_PAGE_STYLE_locals_dict, LV_PAGE_STYLE_locals_dict_table);

STATIC void LV_PAGE_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_PAGE_STYLE");
}



STATIC const mp_obj_type_t mp_LV_PAGE_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_PAGE_STYLE,
    .print = LV_PAGE_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_PAGE_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_TXT_CMD_STATE object definitions
 */

STATIC const mp_rom_map_elem_t LV_TXT_CMD_STATE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_IN), MP_ROM_PTR(MP_ROM_INT(LV_TXT_CMD_STATE_IN)) },
    { MP_ROM_QSTR(MP_QSTR_PAR), MP_ROM_PTR(MP_ROM_INT(LV_TXT_CMD_STATE_PAR)) },
    { MP_ROM_QSTR(MP_QSTR_WAIT), MP_ROM_PTR(MP_ROM_INT(LV_TXT_CMD_STATE_WAIT)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_TXT_CMD_STATE_locals_dict, LV_TXT_CMD_STATE_locals_dict_table);

STATIC void LV_TXT_CMD_STATE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_TXT_CMD_STATE");
}



STATIC const mp_obj_type_t mp_LV_TXT_CMD_STATE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_TXT_CMD_STATE,
    .print = LV_TXT_CMD_STATE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_TXT_CMD_STATE_locals_dict,
};
    

    
/*
 * lvgl LV_LABEL_LONG object definitions
 */

STATIC const mp_rom_map_elem_t LV_LABEL_LONG_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_DOT), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_DOT)) },
    { MP_ROM_QSTR(MP_QSTR_CROP), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_CROP)) },
    { MP_ROM_QSTR(MP_QSTR_BREAK), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_BREAK)) },
    { MP_ROM_QSTR(MP_QSTR_SCROLL), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_SCROLL)) },
    { MP_ROM_QSTR(MP_QSTR_EXPAND), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_EXPAND)) },
    { MP_ROM_QSTR(MP_QSTR_ROLL), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_LONG_ROLL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_LABEL_LONG_locals_dict, LV_LABEL_LONG_locals_dict_table);

STATIC void LV_LABEL_LONG_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_LABEL_LONG");
}



STATIC const mp_obj_type_t mp_LV_LABEL_LONG_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_LABEL_LONG,
    .print = LV_LABEL_LONG_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_LABEL_LONG_locals_dict,
};
    

    
/*
 * lvgl LV_PROTECT object definitions
 */

STATIC const mp_rom_map_elem_t LV_PROTECT_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_NONE), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_NONE)) },
    { MP_ROM_QSTR(MP_QSTR_PARENT), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_PARENT)) },
    { MP_ROM_QSTR(MP_QSTR_PRESS_LOST), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_PRESS_LOST)) },
    { MP_ROM_QSTR(MP_QSTR_FOLLOW), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_FOLLOW)) },
    { MP_ROM_QSTR(MP_QSTR_POS), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_POS)) },
    { MP_ROM_QSTR(MP_QSTR_CLICK_FOCUS), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_CLICK_FOCUS)) },
    { MP_ROM_QSTR(MP_QSTR_CHILD_CHG), MP_ROM_PTR(MP_ROM_INT(LV_PROTECT_CHILD_CHG)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_PROTECT_locals_dict, LV_PROTECT_locals_dict_table);

STATIC void LV_PROTECT_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_PROTECT");
}



STATIC const mp_obj_type_t mp_LV_PROTECT_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_PROTECT,
    .print = LV_PROTECT_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_PROTECT_locals_dict,
};
    

    
/*
 * lvgl LV_BTN_STATE object definitions
 */

STATIC const mp_rom_map_elem_t LV_BTN_STATE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_NUM), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_NUM)) },
    { MP_ROM_QSTR(MP_QSTR_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_PR), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_PR)) },
    { MP_ROM_QSTR(MP_QSTR_INA), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_INA)) },
    { MP_ROM_QSTR(MP_QSTR_REL), MP_ROM_PTR(MP_ROM_INT(LV_BTN_STATE_REL)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_BTN_STATE_locals_dict, LV_BTN_STATE_locals_dict_table);

STATIC void LV_BTN_STATE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_BTN_STATE");
}



STATIC const mp_obj_type_t mp_LV_BTN_STATE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_BTN_STATE,
    .print = LV_BTN_STATE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_BTN_STATE_locals_dict,
};
    

    
/*
 * lvgl LV_PRELOAD_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_PRELOAD_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_MAIN), MP_ROM_PTR(MP_ROM_INT(LV_PRELOAD_STYLE_MAIN)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_PRELOAD_STYLE_locals_dict, LV_PRELOAD_STYLE_locals_dict_table);

STATIC void LV_PRELOAD_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_PRELOAD_STYLE");
}



STATIC const mp_obj_type_t mp_LV_PRELOAD_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_PRELOAD_STYLE,
    .print = LV_PRELOAD_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_PRELOAD_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_LABEL_ALIGN object definitions
 */

STATIC const mp_rom_map_elem_t LV_LABEL_ALIGN_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_CENTER), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_ALIGN_CENTER)) },
    { MP_ROM_QSTR(MP_QSTR_RIGHT), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_ALIGN_RIGHT)) },
    { MP_ROM_QSTR(MP_QSTR_LEFT), MP_ROM_PTR(MP_ROM_INT(LV_LABEL_ALIGN_LEFT)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_LABEL_ALIGN_locals_dict, LV_LABEL_ALIGN_locals_dict_table);

STATIC void LV_LABEL_ALIGN_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_LABEL_ALIGN");
}



STATIC const mp_obj_type_t mp_LV_LABEL_ALIGN_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_LABEL_ALIGN,
    .print = LV_LABEL_ALIGN_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_LABEL_ALIGN_locals_dict,
};
    

    
/*
 * lvgl LV_CHART_TYPE object definitions
 */

STATIC const mp_rom_map_elem_t LV_CHART_TYPE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_LINE), MP_ROM_PTR(MP_ROM_INT(LV_CHART_TYPE_LINE)) },
    { MP_ROM_QSTR(MP_QSTR_POINT), MP_ROM_PTR(MP_ROM_INT(LV_CHART_TYPE_POINT)) },
    { MP_ROM_QSTR(MP_QSTR_COLUMN), MP_ROM_PTR(MP_ROM_INT(LV_CHART_TYPE_COLUMN)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_CHART_TYPE_locals_dict, LV_CHART_TYPE_locals_dict_table);

STATIC void LV_CHART_TYPE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_CHART_TYPE");
}



STATIC const mp_obj_type_t mp_LV_CHART_TYPE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_CHART_TYPE,
    .print = LV_CHART_TYPE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_CHART_TYPE_locals_dict,
};
    

    
/*
 * lvgl LV_DESIGN object definitions
 */

STATIC const mp_rom_map_elem_t LV_DESIGN_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_COVER_CHK), MP_ROM_PTR(MP_ROM_INT(LV_DESIGN_COVER_CHK)) },
    { MP_ROM_QSTR(MP_QSTR_DRAW_POST), MP_ROM_PTR(MP_ROM_INT(LV_DESIGN_DRAW_POST)) },
    { MP_ROM_QSTR(MP_QSTR_DRAW_MAIN), MP_ROM_PTR(MP_ROM_INT(LV_DESIGN_DRAW_MAIN)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_DESIGN_locals_dict, LV_DESIGN_locals_dict_table);

STATIC void LV_DESIGN_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_DESIGN");
}



STATIC const mp_obj_type_t mp_LV_DESIGN_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_DESIGN,
    .print = LV_DESIGN_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_DESIGN_locals_dict,
};
    

    
/*
 * lvgl LV_SLIDER_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_SLIDER_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_SLIDER_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_INDIC), MP_ROM_PTR(MP_ROM_INT(LV_SLIDER_STYLE_INDIC)) },
    { MP_ROM_QSTR(MP_QSTR_KNOB), MP_ROM_PTR(MP_ROM_INT(LV_SLIDER_STYLE_KNOB)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_SLIDER_STYLE_locals_dict, LV_SLIDER_STYLE_locals_dict_table);

STATIC void LV_SLIDER_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_SLIDER_STYLE");
}



STATIC const mp_obj_type_t mp_LV_SLIDER_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_SLIDER_STYLE,
    .print = LV_SLIDER_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_SLIDER_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_CB_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_CB_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BOX_INA), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BOX_INA)) },
    { MP_ROM_QSTR(MP_QSTR_BOX_REL), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BOX_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BOX_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BOX_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BOX_PR), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BOX_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BOX_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_CB_STYLE_BOX_TGL_PR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_CB_STYLE_locals_dict, LV_CB_STYLE_locals_dict_table);

STATIC void LV_CB_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_CB_STYLE");
}



STATIC const mp_obj_type_t mp_LV_CB_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_CB_STYLE,
    .print = LV_CB_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_CB_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_ARC_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_ARC_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_MAIN), MP_ROM_PTR(MP_ROM_INT(LV_ARC_STYLE_MAIN)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_ARC_STYLE_locals_dict, LV_ARC_STYLE_locals_dict_table);

STATIC void LV_ARC_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_ARC_STYLE");
}



STATIC const mp_obj_type_t mp_LV_ARC_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_ARC_STYLE,
    .print = LV_ARC_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_ARC_STYLE_locals_dict,
};
    

    
/*
 * lvgl LV_KB_STYLE object definitions
 */

STATIC const mp_rom_map_elem_t LV_KB_STYLE_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_REL), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BTN_TGL_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BG), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BG)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_INA), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BTN_INA)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_REL), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BTN_REL)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_TGL_PR), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BTN_TGL_PR)) },
    { MP_ROM_QSTR(MP_QSTR_BTN_PR), MP_ROM_PTR(MP_ROM_INT(LV_KB_STYLE_BTN_PR)) }
};

STATIC MP_DEFINE_CONST_DICT(LV_KB_STYLE_locals_dict, LV_KB_STYLE_locals_dict_table);

STATIC void LV_KB_STYLE_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl LV_KB_STYLE");
}



STATIC const mp_obj_type_t mp_LV_KB_STYLE_type = {
    { &mp_type_type },
    .name = MP_QSTR_LV_KB_STYLE,
    .print = LV_KB_STYLE_print,
    
    .locals_dict = (mp_obj_dict_t*)&LV_KB_STYLE_locals_dict,
};
    

/*
 * Function NOT generated:
 * Callback: Can only handle callbaks that return lv_res_t or void
 * int32_t lv_anim_path_t(const struct _lv_anim_t *)
 */
    

/*
 * Callback function lv_anim_fp_t
 * void lv_anim_fp_t(void *, int32_t)
 */

STATIC void lv_anim_fp_t_callback(void* arg0, int32_t arg1)
{
    mp_obj_t args[2];
    args[0] = ptr_to_mp((void*)arg0);
    args[1] = mp_obj_new_int(arg1);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(2, args);
    mp_sched_schedule(action, arg_list);
    return;
}


/*
 * Callback function lv_anim_cb_t
 * void lv_anim_cb_t(void *)
 */

STATIC void lv_anim_cb_t_callback(void* arg0)
{
    mp_obj_t args[1];
    args[0] = ptr_to_mp((void*)arg0);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(1, args);
    mp_sched_schedule(action, arg_list);
    return;
}


/*
 * Function NOT generated:
 * Callback: Can only handle callbaks that return lv_res_t or void
 * bool lv_design_func_t(struct _lv_obj_t *obj, const lv_area_t *mask_p, lv_design_mode_t mode)
 */
    

/*
 * Callback function lv_signal_func_t
 * lv_res_t lv_signal_func_t(struct _lv_obj_t *obj, lv_signal_t sign, void *param)
 */

STATIC lv_res_t lv_signal_func_t_callback(struct _lv_obj_t* arg0, lv_signal_t arg1, void* arg2)
{
    mp_obj_t args[3];
    args[0] = lv_to_mp((void*)arg0);
    args[1] = mp_obj_new_int_from_uint(arg1);
    args[2] = ptr_to_mp((void*)arg2);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(3, args);
    bool schedule_result = mp_sched_schedule(action, arg_list);
    return schedule_result? LV_RES_OK: LV_RES_INV;
}


/*
 * Callback function lv_action_t
 * lv_res_t lv_action_t(struct _lv_obj_t *obj)
 */

STATIC lv_res_t lv_action_t_callback(struct _lv_obj_t* arg0)
{
    mp_obj_t args[1];
    args[0] = lv_to_mp((void*)arg0);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(1, args);
    bool schedule_result = mp_sched_schedule(action, arg_list);
    return schedule_result? LV_RES_OK: LV_RES_INV;
}

typedef typeof( ((lv_style_t*)(0))->body ) lv_style_body_t;

/*
 * Struct lv_color16_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_color16_t_type();

STATIC inline lv_color16_t* mp_write_ptr_lv_color16_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_color16_t*)self->data;
}

#define mp_write_lv_color16_t(struct_obj) *mp_write_ptr_lv_color16_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_color16_t(lv_color16_t *field)
{
    return lv_to_mp_struct(get_mp_lv_color16_t_type(), field);
}

#define mp_read_lv_color16_t(field) mp_read_ptr_lv_color16_t(&field)

STATIC void mp_lv_color16_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_color16_t *data = (lv_color16_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_blue: dest[0] = mp_obj_new_int_from_uint(data->blue); break; // converting from uint16_t;
            case MP_QSTR_green: dest[0] = mp_obj_new_int_from_uint(data->green); break; // converting from uint16_t;
            case MP_QSTR_red: dest[0] = mp_obj_new_int_from_uint(data->red); break; // converting from uint16_t;
            case MP_QSTR_full: dest[0] = mp_obj_new_int_from_uint(data->full); break; // converting from uint16_t;
            default: field_not_found(MP_QSTR_lv_color16_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_blue: data->blue = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_green: data->green = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_red: data->red = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_full: data->full = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                default: field_not_found(MP_QSTR_lv_color16_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_color16_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_color16_t");
}

STATIC const mp_rom_map_elem_t mp_lv_color16_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_color16_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_color16_t_locals_dict, mp_lv_color16_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_color16_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_color16_t,
    .print = mp_lv_color16_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_color16_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_color16_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_color16_t_type()
{
    return &mp_lv_color16_t_type;
}
    
typedef typeof( ((lv_style_body_t*)(0))->border ) lv_style_body_border_t;

/*
 * Struct lv_style_body_border_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_border_t_type();

STATIC inline lv_style_body_border_t* mp_write_ptr_lv_style_body_border_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_body_border_t*)self->data;
}

#define mp_write_lv_style_body_border_t(struct_obj) *mp_write_ptr_lv_style_body_border_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_body_border_t(lv_style_body_border_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_body_border_t_type(), field);
}

#define mp_read_lv_style_body_border_t(field) mp_read_ptr_lv_style_body_border_t(&field)

STATIC void mp_lv_style_body_border_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_body_border_t *data = (lv_style_body_border_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            case MP_QSTR_width: dest[0] = mp_obj_new_int(data->width); break; // converting from lv_coord_t;
            case MP_QSTR_part: dest[0] = mp_obj_new_int_from_uint(data->part); break; // converting from lv_border_part_t;
            case MP_QSTR_opa: dest[0] = mp_obj_new_int_from_uint(data->opa); break; // converting from lv_opa_t;
            default: field_not_found(MP_QSTR_lv_style_body_border_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_width: data->width = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_part: data->part = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_border_part_t;
                case MP_QSTR_opa: data->opa = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                default: field_not_found(MP_QSTR_lv_style_body_border_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_body_border_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_body_border_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_body_border_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_body_border_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_body_border_t_locals_dict, mp_lv_style_body_border_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_body_border_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_body_border_t,
    .print = mp_lv_style_body_border_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_body_border_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_body_border_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_border_t_type()
{
    return &mp_lv_style_body_border_t_type;
}
    
typedef typeof( ((lv_style_body_t*)(0))->shadow ) lv_style_body_shadow_t;

/*
 * Struct lv_style_body_shadow_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_shadow_t_type();

STATIC inline lv_style_body_shadow_t* mp_write_ptr_lv_style_body_shadow_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_body_shadow_t*)self->data;
}

#define mp_write_lv_style_body_shadow_t(struct_obj) *mp_write_ptr_lv_style_body_shadow_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_body_shadow_t(lv_style_body_shadow_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_body_shadow_t_type(), field);
}

#define mp_read_lv_style_body_shadow_t(field) mp_read_ptr_lv_style_body_shadow_t(&field)

STATIC void mp_lv_style_body_shadow_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_body_shadow_t *data = (lv_style_body_shadow_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            case MP_QSTR_width: dest[0] = mp_obj_new_int(data->width); break; // converting from lv_coord_t;
            case MP_QSTR_type: dest[0] = mp_obj_new_int_from_uint(data->type); break; // converting from lv_shadow_type_t;
            default: field_not_found(MP_QSTR_lv_style_body_shadow_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_width: data->width = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_type: data->type = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_shadow_type_t;
                default: field_not_found(MP_QSTR_lv_style_body_shadow_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_body_shadow_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_body_shadow_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_body_shadow_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_body_shadow_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_body_shadow_t_locals_dict, mp_lv_style_body_shadow_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_body_shadow_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_body_shadow_t,
    .print = mp_lv_style_body_shadow_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_body_shadow_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_body_shadow_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_shadow_t_type()
{
    return &mp_lv_style_body_shadow_t_type;
}
    
typedef typeof( ((lv_style_body_t*)(0))->padding ) lv_style_body_padding_t;

/*
 * Struct lv_style_body_padding_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_padding_t_type();

STATIC inline lv_style_body_padding_t* mp_write_ptr_lv_style_body_padding_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_body_padding_t*)self->data;
}

#define mp_write_lv_style_body_padding_t(struct_obj) *mp_write_ptr_lv_style_body_padding_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_body_padding_t(lv_style_body_padding_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_body_padding_t_type(), field);
}

#define mp_read_lv_style_body_padding_t(field) mp_read_ptr_lv_style_body_padding_t(&field)

STATIC void mp_lv_style_body_padding_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_body_padding_t *data = (lv_style_body_padding_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_ver: dest[0] = mp_obj_new_int(data->ver); break; // converting from lv_coord_t;
            case MP_QSTR_hor: dest[0] = mp_obj_new_int(data->hor); break; // converting from lv_coord_t;
            case MP_QSTR_inner: dest[0] = mp_obj_new_int(data->inner); break; // converting from lv_coord_t;
            default: field_not_found(MP_QSTR_lv_style_body_padding_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_ver: data->ver = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_hor: data->hor = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_inner: data->inner = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                default: field_not_found(MP_QSTR_lv_style_body_padding_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_body_padding_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_body_padding_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_body_padding_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_body_padding_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_body_padding_t_locals_dict, mp_lv_style_body_padding_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_body_padding_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_body_padding_t,
    .print = mp_lv_style_body_padding_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_body_padding_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_body_padding_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_padding_t_type()
{
    return &mp_lv_style_body_padding_t_type;
}
    

/*
 * Struct lv_style_body_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_t_type();

STATIC inline lv_style_body_t* mp_write_ptr_lv_style_body_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_body_t*)self->data;
}

#define mp_write_lv_style_body_t(struct_obj) *mp_write_ptr_lv_style_body_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_body_t(lv_style_body_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_body_t_type(), field);
}

#define mp_read_lv_style_body_t(field) mp_read_ptr_lv_style_body_t(&field)

STATIC void mp_lv_style_body_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_body_t *data = (lv_style_body_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_main_color: dest[0] = mp_read_lv_color16_t(data->main_color); break; // converting from lv_color_t;
            case MP_QSTR_grad_color: dest[0] = mp_read_lv_color16_t(data->grad_color); break; // converting from lv_color_t;
            case MP_QSTR_radius: dest[0] = mp_obj_new_int(data->radius); break; // converting from lv_coord_t;
            case MP_QSTR_opa: dest[0] = mp_obj_new_int_from_uint(data->opa); break; // converting from lv_opa_t;
            case MP_QSTR_border: dest[0] = mp_read_lv_style_body_border_t(data->border); break; // converting from lv_style_body_border_t;
            case MP_QSTR_shadow: dest[0] = mp_read_lv_style_body_shadow_t(data->shadow); break; // converting from lv_style_body_shadow_t;
            case MP_QSTR_padding: dest[0] = mp_read_lv_style_body_padding_t(data->padding); break; // converting from lv_style_body_padding_t;
            case MP_QSTR_empty: dest[0] = mp_obj_new_int_from_uint(data->empty); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_style_body_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_main_color: data->main_color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_grad_color: data->grad_color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_radius: data->radius = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_opa: data->opa = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                case MP_QSTR_border: data->border = mp_write_lv_style_body_border_t(dest[1]); break; // converting to lv_style_body_border_t;
                case MP_QSTR_shadow: data->shadow = mp_write_lv_style_body_shadow_t(dest[1]); break; // converting to lv_style_body_shadow_t;
                case MP_QSTR_padding: data->padding = mp_write_lv_style_body_padding_t(dest[1]); break; // converting to lv_style_body_padding_t;
                case MP_QSTR_empty: data->empty = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_style_body_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_body_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_body_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_body_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_body_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_body_t_locals_dict, mp_lv_style_body_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_body_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_body_t,
    .print = mp_lv_style_body_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_body_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_body_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_body_t_type()
{
    return &mp_lv_style_body_t_type;
}
    
typedef typeof( ((lv_style_t*)(0))->text ) lv_style_text_t;

/*
 * Struct lv_style_text_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_text_t_type();

STATIC inline lv_style_text_t* mp_write_ptr_lv_style_text_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_text_t*)self->data;
}

#define mp_write_lv_style_text_t(struct_obj) *mp_write_ptr_lv_style_text_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_text_t(lv_style_text_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_text_t_type(), field);
}

#define mp_read_lv_style_text_t(field) mp_read_ptr_lv_style_text_t(&field)

STATIC void mp_lv_style_text_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_text_t *data = (lv_style_text_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            case MP_QSTR_font: dest[0] = ptr_to_mp((void*)data->font); break; // converting from lv_font_t*;
            case MP_QSTR_letter_space: dest[0] = mp_obj_new_int(data->letter_space); break; // converting from lv_coord_t;
            case MP_QSTR_line_space: dest[0] = mp_obj_new_int(data->line_space); break; // converting from lv_coord_t;
            case MP_QSTR_opa: dest[0] = mp_obj_new_int_from_uint(data->opa); break; // converting from lv_opa_t;
            default: field_not_found(MP_QSTR_lv_style_text_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_font: data->font = (void*)mp_to_ptr(dest[1]); break; // converting to lv_font_t*;
                case MP_QSTR_letter_space: data->letter_space = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_line_space: data->line_space = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_opa: data->opa = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                default: field_not_found(MP_QSTR_lv_style_text_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_text_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_text_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_text_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_text_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_text_t_locals_dict, mp_lv_style_text_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_text_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_text_t,
    .print = mp_lv_style_text_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_text_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_text_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_text_t_type()
{
    return &mp_lv_style_text_t_type;
}
    
typedef typeof( ((lv_style_t*)(0))->image ) lv_style_image_t;

/*
 * Struct lv_style_image_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_image_t_type();

STATIC inline lv_style_image_t* mp_write_ptr_lv_style_image_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_image_t*)self->data;
}

#define mp_write_lv_style_image_t(struct_obj) *mp_write_ptr_lv_style_image_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_image_t(lv_style_image_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_image_t_type(), field);
}

#define mp_read_lv_style_image_t(field) mp_read_ptr_lv_style_image_t(&field)

STATIC void mp_lv_style_image_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_image_t *data = (lv_style_image_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            case MP_QSTR_intense: dest[0] = mp_obj_new_int_from_uint(data->intense); break; // converting from lv_opa_t;
            case MP_QSTR_opa: dest[0] = mp_obj_new_int_from_uint(data->opa); break; // converting from lv_opa_t;
            default: field_not_found(MP_QSTR_lv_style_image_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_intense: data->intense = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                case MP_QSTR_opa: data->opa = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                default: field_not_found(MP_QSTR_lv_style_image_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_image_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_image_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_image_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_image_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_image_t_locals_dict, mp_lv_style_image_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_image_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_image_t,
    .print = mp_lv_style_image_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_image_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_image_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_image_t_type()
{
    return &mp_lv_style_image_t_type;
}
    
typedef typeof( ((lv_style_t*)(0))->line ) lv_style_line_t;

/*
 * Struct lv_style_line_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_line_t_type();

STATIC inline lv_style_line_t* mp_write_ptr_lv_style_line_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_line_t*)self->data;
}

#define mp_write_lv_style_line_t(struct_obj) *mp_write_ptr_lv_style_line_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_line_t(lv_style_line_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_line_t_type(), field);
}

#define mp_read_lv_style_line_t(field) mp_read_ptr_lv_style_line_t(&field)

STATIC void mp_lv_style_line_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_line_t *data = (lv_style_line_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            case MP_QSTR_width: dest[0] = mp_obj_new_int(data->width); break; // converting from lv_coord_t;
            case MP_QSTR_opa: dest[0] = mp_obj_new_int_from_uint(data->opa); break; // converting from lv_opa_t;
            case MP_QSTR_rounded: dest[0] = mp_obj_new_int_from_uint(data->rounded); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_style_line_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                case MP_QSTR_width: data->width = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_opa: data->opa = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_opa_t;
                case MP_QSTR_rounded: data->rounded = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_style_line_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_line_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_line_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_line_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_line_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_line_t_locals_dict, mp_lv_style_line_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_line_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_line_t,
    .print = mp_lv_style_line_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_line_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_line_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_line_t_type()
{
    return &mp_lv_style_line_t_type;
}
    

/*
 * Struct lv_style_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_t_type();

STATIC inline lv_style_t* mp_write_ptr_lv_style_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_t*)self->data;
}

#define mp_write_lv_style_t(struct_obj) *mp_write_ptr_lv_style_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_t(lv_style_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_t_type(), field);
}

#define mp_read_lv_style_t(field) mp_read_ptr_lv_style_t(&field)

STATIC void mp_lv_style_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_t *data = (lv_style_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_glass: dest[0] = mp_obj_new_int_from_uint(data->glass); break; // converting from uint8_t;
            case MP_QSTR_body: dest[0] = mp_read_lv_style_body_t(data->body); break; // converting from lv_style_body_t;
            case MP_QSTR_text: dest[0] = mp_read_lv_style_text_t(data->text); break; // converting from lv_style_text_t;
            case MP_QSTR_image: dest[0] = mp_read_lv_style_image_t(data->image); break; // converting from lv_style_image_t;
            case MP_QSTR_line: dest[0] = mp_read_lv_style_line_t(data->line); break; // converting from lv_style_line_t;
            default: field_not_found(MP_QSTR_lv_style_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_glass: data->glass = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_body: data->body = mp_write_lv_style_body_t(dest[1]); break; // converting to lv_style_body_t;
                case MP_QSTR_text: data->text = mp_write_lv_style_text_t(dest[1]); break; // converting to lv_style_text_t;
                case MP_QSTR_image: data->image = mp_write_lv_style_image_t(dest[1]); break; // converting to lv_style_image_t;
                case MP_QSTR_line: data->line = mp_write_lv_style_line_t(dest[1]); break; // converting to lv_style_line_t;
                default: field_not_found(MP_QSTR_lv_style_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_t_locals_dict, mp_lv_style_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_t,
    .print = mp_lv_style_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_t_type()
{
    return &mp_lv_style_t_type;
}
    

/*
 * Callback function lv_group_style_mod_func_t
 * void lv_group_style_mod_func_t(lv_style_t *)
 */

STATIC void lv_group_style_mod_func_t_callback(lv_style_t* arg0)
{
    mp_obj_t args[1];
    args[0] = mp_read_ptr_lv_style_t((void*)arg0);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(1, args);
    mp_sched_schedule(action, arg_list);
    return;
}


/*
 * Function NOT generated:
 * Callback: Missing conversion to struct _lv_group_t*
 * void lv_group_focus_cb_t(struct _lv_group_t *)
 */
    

/*
 * Callback function lv_btnm_action_t
 * lv_res_t lv_btnm_action_t(lv_obj_t *, const char *txt)
 */

STATIC lv_res_t lv_btnm_action_t_callback(lv_obj_t* arg0, const char* arg1)
{
    mp_obj_t args[2];
    args[0] = lv_to_mp((void*)arg0);
    args[1] = convert_to_str((void*)arg1);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(2, args);
    bool schedule_result = mp_sched_schedule(action, arg_list);
    return schedule_result? LV_RES_OK: LV_RES_INV;
}


/*
 * Struct lv_img_header_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_img_header_t_type();

STATIC inline lv_img_header_t* mp_write_ptr_lv_img_header_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_img_header_t*)self->data;
}

#define mp_write_lv_img_header_t(struct_obj) *mp_write_ptr_lv_img_header_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_img_header_t(lv_img_header_t *field)
{
    return lv_to_mp_struct(get_mp_lv_img_header_t_type(), field);
}

#define mp_read_lv_img_header_t(field) mp_read_ptr_lv_img_header_t(&field)

STATIC void mp_lv_img_header_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_img_header_t *data = (lv_img_header_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_cf: dest[0] = mp_obj_new_int_from_uint(data->cf); break; // converting from uint32_t;
            case MP_QSTR_always_zero: dest[0] = mp_obj_new_int_from_uint(data->always_zero); break; // converting from uint32_t;
            case MP_QSTR_reserved: dest[0] = mp_obj_new_int_from_uint(data->reserved); break; // converting from uint32_t;
            case MP_QSTR_w: dest[0] = mp_obj_new_int_from_uint(data->w); break; // converting from uint32_t;
            case MP_QSTR_h: dest[0] = mp_obj_new_int_from_uint(data->h); break; // converting from uint32_t;
            default: field_not_found(MP_QSTR_lv_img_header_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_cf: data->cf = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_always_zero: data->always_zero = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_reserved: data->reserved = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_w: data->w = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_h: data->h = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                default: field_not_found(MP_QSTR_lv_img_header_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_img_header_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_img_header_t");
}

STATIC const mp_rom_map_elem_t mp_lv_img_header_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_img_header_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_img_header_t_locals_dict, mp_lv_img_header_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_img_header_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_img_header_t,
    .print = mp_lv_img_header_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_img_header_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_img_header_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_img_header_t_type()
{
    return &mp_lv_img_header_t_type;
}
    

/*
 * Callback function lv_img_decoder_info_f_t
 * lv_res_t lv_img_decoder_info_f_t(const void *src, lv_img_header_t *header)
 */

STATIC lv_res_t lv_img_decoder_info_f_t_callback(const void* arg0, lv_img_header_t* arg1)
{
    mp_obj_t args[2];
    args[0] = ptr_to_mp((void*)arg0);
    args[1] = mp_read_ptr_lv_img_header_t((void*)arg1);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(2, args);
    bool schedule_result = mp_sched_schedule(action, arg_list);
    return schedule_result? LV_RES_OK: LV_RES_INV;
}


/*
 * Function NOT generated:
 * Callback: Can only handle callbaks that return lv_res_t or void
 * const uint8_t *lv_img_decoder_open_f_t(const void *src, const lv_style_t *style)
 */
    

/*
 * Function NOT generated:
 * Callback: Missing conversion to uint8_t*
 * lv_res_t lv_img_decoder_read_line_f_t(lv_coord_t x, lv_coord_t y, lv_coord_t len, uint8_t *buf)
 */
    

/*
 * Function NOT generated:
 * Callback: Missing conversion to void
 * void lv_img_decoder_close_f_t(void)
 */
    

/*
 * Callback function lv_tabview_action_t
 * lv_res_t lv_tabview_action_t(lv_obj_t *, uint16_t)
 */

STATIC lv_res_t lv_tabview_action_t_callback(lv_obj_t* arg0, uint16_t arg1)
{
    mp_obj_t args[2];
    args[0] = lv_to_mp((void*)arg0);
    args[1] = mp_obj_new_int_from_uint(arg1);
    mp_obj_t action = mp_to_lv_action(args[0]);
    mp_obj_t arg_list = mp_obj_new_list(2, args);
    bool schedule_result = mp_sched_schedule(action, arg_list);
    return schedule_result? LV_RES_OK: LV_RES_INV;
}


/*
 * lvgl extension definition for:
 * lv_res_t lv_obj_del(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_del(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_res_t res = lv_obj_del(obj);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_del_obj, 1, 1, mp_lv_obj_del);

 

/*
 * lvgl extension definition for:
 * void lv_obj_clean(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_clean(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_clean(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_clean_obj, 1, 1, mp_lv_obj_clean);

 

/*
 * lvgl extension definition for:
 * void lv_obj_invalidate(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_invalidate(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_invalidate(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_invalidate_obj, 1, 1, mp_lv_obj_invalidate);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_parent(lv_obj_t *obj, lv_obj_t *parent)
 */
 
STATIC mp_obj_t mp_lv_obj_set_parent(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_t *parent = mp_to_lv(args[1]);
    lv_obj_set_parent(obj, parent);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_parent_obj, 2, 2, mp_lv_obj_set_parent);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_pos(lv_obj_t *obj, lv_coord_t x, lv_coord_t y)
 */
 
STATIC mp_obj_t mp_lv_obj_set_pos(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t x = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_obj_set_pos(obj, x, y);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_pos_obj, 3, 3, mp_lv_obj_set_pos);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_x(lv_obj_t *obj, lv_coord_t x)
 */
 
STATIC mp_obj_t mp_lv_obj_set_x(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t x = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_x(obj, x);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_x_obj, 2, 2, mp_lv_obj_set_x);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_y(lv_obj_t *obj, lv_coord_t y)
 */
 
STATIC mp_obj_t mp_lv_obj_set_y(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_y(obj, y);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_y_obj, 2, 2, mp_lv_obj_set_y);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_size(lv_obj_t *obj, lv_coord_t w, lv_coord_t h)
 */
 
STATIC mp_obj_t mp_lv_obj_set_size(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t w = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t h = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_obj_set_size(obj, w, h);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_size_obj, 3, 3, mp_lv_obj_set_size);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_width(lv_obj_t *obj, lv_coord_t w)
 */
 
STATIC mp_obj_t mp_lv_obj_set_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t w = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_width(obj, w);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_width_obj, 2, 2, mp_lv_obj_set_width);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_height(lv_obj_t *obj, lv_coord_t h)
 */
 
STATIC mp_obj_t mp_lv_obj_set_height(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t h = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_height(obj, h);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_height_obj, 2, 2, mp_lv_obj_set_height);

 

/*
 * lvgl extension definition for:
 * void lv_obj_align(lv_obj_t *obj, const lv_obj_t *base, lv_align_t align, lv_coord_t x_mod, lv_coord_t y_mod)
 */
 
STATIC mp_obj_t mp_lv_obj_align(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    const lv_obj_t *base = mp_to_lv(args[1]);
    lv_align_t align = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_coord_t x_mod = (int16_t)mp_obj_int_get_checked(args[3]);
    lv_coord_t y_mod = (int16_t)mp_obj_int_get_checked(args[4]);
    lv_obj_align(obj, base, align, x_mod, y_mod);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_align_obj, 5, 5, mp_lv_obj_align);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_style(lv_obj_t *obj, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_obj_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_obj_set_style(obj, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_style_obj, 2, 2, mp_lv_obj_set_style);

 

/*
 * lvgl extension definition for:
 * void lv_obj_refresh_style(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_refresh_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_refresh_style(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_refresh_style_obj, 1, 1, mp_lv_obj_refresh_style);

 

/*
 * lvgl extension definition for:
 * void lv_obj_report_style_mod(lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_obj_report_style_mod(size_t n_args, const mp_obj_t *args)
{
    lv_style_t *style = mp_write_ptr_lv_style_t(args[0]);
    lv_obj_report_style_mod(style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_report_style_mod_obj, 1, 1, mp_lv_obj_report_style_mod);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_hidden(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_hidden(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_hidden(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_hidden_obj, 2, 2, mp_lv_obj_set_hidden);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_click(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_click(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_click(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_click_obj, 2, 2, mp_lv_obj_set_click);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_top(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_top(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_top(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_top_obj, 2, 2, mp_lv_obj_set_top);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_drag(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_drag(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_drag(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_drag_obj, 2, 2, mp_lv_obj_set_drag);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_drag_throw(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_drag_throw(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_drag_throw(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_drag_throw_obj, 2, 2, mp_lv_obj_set_drag_throw);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_drag_parent(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_drag_parent(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_drag_parent(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_drag_parent_obj, 2, 2, mp_lv_obj_set_drag_parent);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_opa_scale_enable(lv_obj_t *obj, bool en)
 */
 
STATIC mp_obj_t mp_lv_obj_set_opa_scale_enable(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_obj_set_opa_scale_enable(obj, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_opa_scale_enable_obj, 2, 2, mp_lv_obj_set_opa_scale_enable);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_opa_scale(lv_obj_t *obj, lv_opa_t opa_scale)
 */
 
STATIC mp_obj_t mp_lv_obj_set_opa_scale(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_opa_t opa_scale = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_opa_scale(obj, opa_scale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_opa_scale_obj, 2, 2, mp_lv_obj_set_opa_scale);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_protect(lv_obj_t *obj, uint8_t prot)
 */
 
STATIC mp_obj_t mp_lv_obj_set_protect(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    uint8_t prot = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_protect(obj, prot);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_protect_obj, 2, 2, mp_lv_obj_set_protect);

 

/*
 * lvgl extension definition for:
 * void lv_obj_clear_protect(lv_obj_t *obj, uint8_t prot)
 */
 
STATIC mp_obj_t mp_lv_obj_clear_protect(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    uint8_t prot = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_obj_clear_protect(obj, prot);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_clear_protect_obj, 2, 2, mp_lv_obj_clear_protect);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_signal_func(lv_obj_t *obj, lv_signal_func_t fp)
 */
 
STATIC mp_obj_t mp_lv_obj_set_signal_func(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_signal_func_t fp = &lv_signal_func_t_callback;
    lv_obj_set_signal_func(obj, fp);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_signal_func_obj, 2, 2, mp_lv_obj_set_signal_func);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_design_func(lv_obj_t *obj, lv_design_func_t fp)
 */
 
STATIC mp_obj_t mp_lv_obj_set_design_func(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_design_func_t fp = mp_to_ptr(args[1]);
    lv_obj_set_design_func(obj, fp);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_design_func_obj, 2, 2, mp_lv_obj_set_design_func);

 

/*
 * lvgl extension definition for:
 * void *lv_obj_allocate_ext_attr(lv_obj_t *obj, uint16_t ext_size)
 */
 
STATIC mp_obj_t mp_lv_obj_allocate_ext_attr(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    uint16_t ext_size = (uint16_t)mp_obj_int_get_checked(args[1]);
    void* res = lv_obj_allocate_ext_attr(obj, ext_size);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_allocate_ext_attr_obj, 2, 2, mp_lv_obj_allocate_ext_attr);

 

/*
 * lvgl extension definition for:
 * void lv_obj_refresh_ext_size(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_refresh_ext_size(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_refresh_ext_size(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_refresh_ext_size_obj, 1, 1, mp_lv_obj_refresh_ext_size);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_free_num(lv_obj_t *obj, uint32_t free_num)
 */
 
STATIC mp_obj_t mp_lv_obj_set_free_num(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    uint32_t free_num = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_obj_set_free_num(obj, free_num);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_free_num_obj, 2, 2, mp_lv_obj_set_free_num);

 

/*
 * lvgl extension definition for:
 * void lv_obj_set_free_ptr(lv_obj_t *obj, void *free_p)
 */
 
STATIC mp_obj_t mp_lv_obj_set_free_ptr(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    void *free_p = mp_to_ptr(args[1]);
    lv_obj_set_free_ptr(obj, free_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_set_free_ptr_obj, 2, 2, mp_lv_obj_set_free_ptr);

 

/*
 * Function NOT generated:
 * Missing conversion to void cb(lv_obj_t *)*
 * void lv_obj_animate(lv_obj_t *obj, lv_anim_builtin_t type, uint16_t time, uint16_t delay, void (*cb)(lv_obj_t *))
 */
    

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_obj_get_screen(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_screen(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_t* res = lv_obj_get_screen(obj);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_screen_obj, 1, 1, mp_lv_obj_get_screen);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_obj_get_parent(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_parent(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_obj_t* res = lv_obj_get_parent(obj);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_parent_obj, 1, 1, mp_lv_obj_get_parent);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_obj_get_child(const lv_obj_t *obj, const lv_obj_t *child)
 */
 
STATIC mp_obj_t mp_lv_obj_get_child(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    const lv_obj_t *child = mp_to_lv(args[1]);
    lv_obj_t* res = lv_obj_get_child(obj, child);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_child_obj, 2, 2, mp_lv_obj_get_child);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_obj_get_child_back(const lv_obj_t *obj, const lv_obj_t *child)
 */
 
STATIC mp_obj_t mp_lv_obj_get_child_back(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    const lv_obj_t *child = mp_to_lv(args[1]);
    lv_obj_t* res = lv_obj_get_child_back(obj, child);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_child_back_obj, 2, 2, mp_lv_obj_get_child_back);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_obj_count_children(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_count_children(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    uint16_t res = lv_obj_count_children(obj);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_count_children_obj, 1, 1, mp_lv_obj_count_children);

 

/*
 * Struct lv_area_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_area_t_type();

STATIC inline lv_area_t* mp_write_ptr_lv_area_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_area_t*)self->data;
}

#define mp_write_lv_area_t(struct_obj) *mp_write_ptr_lv_area_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_area_t(lv_area_t *field)
{
    return lv_to_mp_struct(get_mp_lv_area_t_type(), field);
}

#define mp_read_lv_area_t(field) mp_read_ptr_lv_area_t(&field)

STATIC void mp_lv_area_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_area_t *data = (lv_area_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_x1: dest[0] = mp_obj_new_int(data->x1); break; // converting from lv_coord_t;
            case MP_QSTR_y1: dest[0] = mp_obj_new_int(data->y1); break; // converting from lv_coord_t;
            case MP_QSTR_x2: dest[0] = mp_obj_new_int(data->x2); break; // converting from lv_coord_t;
            case MP_QSTR_y2: dest[0] = mp_obj_new_int(data->y2); break; // converting from lv_coord_t;
            default: field_not_found(MP_QSTR_lv_area_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_x1: data->x1 = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_y1: data->y1 = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_x2: data->x2 = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_y2: data->y2 = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                default: field_not_found(MP_QSTR_lv_area_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_area_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_area_t");
}

STATIC const mp_rom_map_elem_t mp_lv_area_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_area_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_area_t_locals_dict, mp_lv_area_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_area_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_area_t,
    .print = mp_lv_area_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_area_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_area_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_area_t_type()
{
    return &mp_lv_area_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_obj_get_coords(const lv_obj_t *obj, lv_area_t *cords_p)
 */
 
STATIC mp_obj_t mp_lv_obj_get_coords(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_area_t *cords_p = mp_write_ptr_lv_area_t(args[1]);
    lv_obj_get_coords(obj, cords_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_coords_obj, 2, 2, mp_lv_obj_get_coords);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_obj_get_x(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_x(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t res = lv_obj_get_x(obj);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_x_obj, 1, 1, mp_lv_obj_get_x);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_obj_get_y(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_y(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t res = lv_obj_get_y(obj);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_y_obj, 1, 1, mp_lv_obj_get_y);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_obj_get_width(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_width(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t res = lv_obj_get_width(obj);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_width_obj, 1, 1, mp_lv_obj_get_width);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_obj_get_height(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_height(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t res = lv_obj_get_height(obj);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_height_obj, 1, 1, mp_lv_obj_get_height);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_obj_get_ext_size(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_ext_size(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_coord_t res = lv_obj_get_ext_size(obj);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_ext_size_obj, 1, 1, mp_lv_obj_get_ext_size);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_obj_get_style(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_style_t* res = lv_obj_get_style(obj);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_style_obj, 1, 1, mp_lv_obj_get_style);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_hidden(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_hidden(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_hidden(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_hidden_obj, 1, 1, mp_lv_obj_get_hidden);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_click(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_click(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_click(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_click_obj, 1, 1, mp_lv_obj_get_click);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_top(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_top(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_top(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_top_obj, 1, 1, mp_lv_obj_get_top);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_drag(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_drag(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_drag(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_drag_obj, 1, 1, mp_lv_obj_get_drag);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_drag_throw(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_drag_throw(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_drag_throw(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_drag_throw_obj, 1, 1, mp_lv_obj_get_drag_throw);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_get_drag_parent(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_drag_parent(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_get_drag_parent(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_drag_parent_obj, 1, 1, mp_lv_obj_get_drag_parent);

 

/*
 * lvgl extension definition for:
 * lv_opa_t lv_obj_get_opa_scale(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_opa_scale(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_opa_t res = lv_obj_get_opa_scale(obj);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_opa_scale_obj, 1, 1, mp_lv_obj_get_opa_scale);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_obj_get_protect(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_protect(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    uint8_t res = lv_obj_get_protect(obj);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_protect_obj, 1, 1, mp_lv_obj_get_protect);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_is_protected(const lv_obj_t *obj, uint8_t prot)
 */
 
STATIC mp_obj_t mp_lv_obj_is_protected(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    uint8_t prot = (uint8_t)mp_obj_int_get_checked(args[1]);
    bool res = lv_obj_is_protected(obj, prot);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_is_protected_obj, 2, 2, mp_lv_obj_is_protected);

 

/*
 * lvgl extension definition for:
 * lv_signal_func_t lv_obj_get_signal_func(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_signal_func(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_signal_func_t res = lv_obj_get_signal_func(obj);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_signal_func_obj, 1, 1, mp_lv_obj_get_signal_func);

 

/*
 * lvgl extension definition for:
 * lv_design_func_t lv_obj_get_design_func(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_design_func(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    lv_design_func_t res = lv_obj_get_design_func(obj);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_design_func_obj, 1, 1, mp_lv_obj_get_design_func);

 

/*
 * lvgl extension definition for:
 * void *lv_obj_get_ext_attr(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_ext_attr(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    void* res = lv_obj_get_ext_attr(obj);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_ext_attr_obj, 1, 1, mp_lv_obj_get_ext_attr);

 

/*
 * Function NOT generated:
 * Missing conversion to char8 when generating struct lv_obj_type_t.type
 * void lv_obj_get_type(lv_obj_t *obj, lv_obj_type_t *buf)
 */
    

/*
 * lvgl extension definition for:
 * uint32_t lv_obj_get_free_num(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_free_num(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    uint32_t res = lv_obj_get_free_num(obj);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_free_num_obj, 1, 1, mp_lv_obj_get_free_num);

 

/*
 * lvgl extension definition for:
 * void *lv_obj_get_free_ptr(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_free_ptr(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    void* res = lv_obj_get_free_ptr(obj);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_free_ptr_obj, 1, 1, mp_lv_obj_get_free_ptr);

 

/*
 * lvgl extension definition for:
 * void *lv_obj_get_group(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_get_group(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    void* res = lv_obj_get_group(obj);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_get_group_obj, 1, 1, mp_lv_obj_get_group);

 

/*
 * lvgl extension definition for:
 * bool lv_obj_is_focused(const lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_obj_is_focused(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *obj = mp_to_lv(args[0]);
    bool res = lv_obj_is_focused(obj);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_obj_is_focused_obj, 1, 1, mp_lv_obj_is_focused);

 

    
/*
 * lvgl obj object definitions
 */

STATIC const mp_rom_map_elem_t obj_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(obj_locals_dict, obj_locals_dict_table);

STATIC void obj_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl obj");
}


STATIC mp_obj_t obj_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_obj_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_obj_type = {
    { &mp_type_type },
    .name = MP_QSTR_obj,
    .print = obj_print,
    .make_new = obj_make_new,
    .locals_dict = (mp_obj_dict_t*)&obj_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * void lv_arc_set_angles(lv_obj_t *arc, uint16_t start, uint16_t end)
 */
 
STATIC mp_obj_t mp_lv_arc_set_angles(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *arc = mp_to_lv(args[0]);
    uint16_t start = (uint16_t)mp_obj_int_get_checked(args[1]);
    uint16_t end = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_arc_set_angles(arc, start, end);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_arc_set_angles_obj, 3, 3, mp_lv_arc_set_angles);

 

/*
 * lvgl extension definition for:
 * void lv_arc_set_style(lv_obj_t *arc, lv_arc_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_arc_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *arc = mp_to_lv(args[0]);
    lv_arc_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_arc_set_style(arc, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_arc_set_style_obj, 3, 3, mp_lv_arc_set_style);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_arc_get_angle_start(lv_obj_t *arc)
 */
 
STATIC mp_obj_t mp_lv_arc_get_angle_start(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *arc = mp_to_lv(args[0]);
    uint16_t res = lv_arc_get_angle_start(arc);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_arc_get_angle_start_obj, 1, 1, mp_lv_arc_get_angle_start);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_arc_get_angle_end(lv_obj_t *arc)
 */
 
STATIC mp_obj_t mp_lv_arc_get_angle_end(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *arc = mp_to_lv(args[0]);
    uint16_t res = lv_arc_get_angle_end(arc);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_arc_get_angle_end_obj, 1, 1, mp_lv_arc_get_angle_end);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_arc_get_style(const lv_obj_t *arc, lv_arc_style_t type)
 */
 
STATIC mp_obj_t mp_lv_arc_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *arc = mp_to_lv(args[0]);
    lv_arc_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_arc_get_style(arc, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_arc_get_style_obj, 2, 2, mp_lv_arc_get_style);

 

    
/*
 * lvgl arc object definitions
 */

STATIC const mp_rom_map_elem_t arc_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_angles), MP_ROM_PTR(&mp_lv_arc_set_angles_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_arc_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle_start), MP_ROM_PTR(&mp_lv_arc_get_angle_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle_end), MP_ROM_PTR(&mp_lv_arc_get_angle_end_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_arc_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_ARC_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(arc_locals_dict, arc_locals_dict_table);

STATIC void arc_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl arc");
}


STATIC mp_obj_t arc_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_arc_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_arc_type = {
    { &mp_type_type },
    .name = MP_QSTR_arc,
    .print = arc_print,
    .make_new = arc_make_new,
    .locals_dict = (mp_obj_dict_t*)&arc_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_cont_set_style(lv_obj_t *cont, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_cont_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cont = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_cont_set_style(cont, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_set_style_obj, 2, 2, mp_lv_cont_set_style);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_cont_get_style(const lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cont = mp_to_lv(args[0]);
    lv_style_t* res = lv_cont_get_style(cont);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_style_obj, 1, 1, mp_lv_cont_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_cont_set_layout(lv_obj_t *cont, lv_layout_t layout)
 */
 
STATIC mp_obj_t mp_lv_cont_set_layout(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cont = mp_to_lv(args[0]);
    lv_layout_t layout = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_cont_set_layout(cont, layout);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_set_layout_obj, 2, 2, mp_lv_cont_set_layout);

 

/*
 * lvgl extension definition for:
 * void lv_cont_set_fit(lv_obj_t *cont, bool hor_en, bool ver_en)
 */
 
STATIC mp_obj_t mp_lv_cont_set_fit(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cont = mp_to_lv(args[0]);
    bool hor_en = mp_obj_is_true(args[1]);
    bool ver_en = mp_obj_is_true(args[2]);
    lv_cont_set_fit(cont, hor_en, ver_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_set_fit_obj, 3, 3, mp_lv_cont_set_fit);

 

/*
 * lvgl extension definition for:
 * lv_layout_t lv_cont_get_layout(const lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_layout(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cont = mp_to_lv(args[0]);
    lv_layout_t res = lv_cont_get_layout(cont);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_layout_obj, 1, 1, mp_lv_cont_get_layout);

 

/*
 * lvgl extension definition for:
 * bool lv_cont_get_hor_fit(const lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_hor_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cont = mp_to_lv(args[0]);
    bool res = lv_cont_get_hor_fit(cont);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_hor_fit_obj, 1, 1, mp_lv_cont_get_hor_fit);

 

/*
 * lvgl extension definition for:
 * bool lv_cont_get_ver_fit(const lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_ver_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cont = mp_to_lv(args[0]);
    bool res = lv_cont_get_ver_fit(cont);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_ver_fit_obj, 1, 1, mp_lv_cont_get_ver_fit);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_cont_get_fit_width(lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_fit_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cont = mp_to_lv(args[0]);
    lv_coord_t res = lv_cont_get_fit_width(cont);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_fit_width_obj, 1, 1, mp_lv_cont_get_fit_width);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_cont_get_fit_height(lv_obj_t *cont)
 */
 
STATIC mp_obj_t mp_lv_cont_get_fit_height(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cont = mp_to_lv(args[0]);
    lv_coord_t res = lv_cont_get_fit_height(cont);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cont_get_fit_height_obj, 1, 1, mp_lv_cont_get_fit_height);

 

    
/*
 * lvgl cont object definitions
 */

STATIC const mp_rom_map_elem_t cont_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(cont_locals_dict, cont_locals_dict_table);

STATIC void cont_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl cont");
}


STATIC mp_obj_t cont_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_cont_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_cont_type = {
    { &mp_type_type },
    .name = MP_QSTR_cont,
    .print = cont_print,
    .make_new = cont_make_new,
    .locals_dict = (mp_obj_dict_t*)&cont_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_btn_set_layout(lv_obj_t *btn, lv_layout_t layout)
 */
 
STATIC mp_obj_t mp_lv_btn_set_layout(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_layout_t layout = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_btn_set_layout(btn, layout);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_layout_obj, 2, 2, mp_lv_btn_set_layout);

 

/*
 * lvgl extension definition for:
 * inline static void lv_btn_set_fit(lv_obj_t *btn, bool hor_en, bool ver_en)
 */
 
STATIC mp_obj_t mp_lv_btn_set_fit(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    bool hor_en = mp_obj_is_true(args[1]);
    bool ver_en = mp_obj_is_true(args[2]);
    lv_btn_set_fit(btn, hor_en, ver_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_fit_obj, 3, 3, mp_lv_btn_set_fit);

 

/*
 * lvgl extension definition for:
 * inline static lv_layout_t lv_btn_get_layout(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_layout(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_layout_t res = lv_btn_get_layout(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_layout_obj, 1, 1, mp_lv_btn_get_layout);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_btn_get_hor_fit(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_hor_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    bool res = lv_btn_get_hor_fit(btn);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_hor_fit_obj, 1, 1, mp_lv_btn_get_hor_fit);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_btn_get_ver_fit(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_ver_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    bool res = lv_btn_get_ver_fit(btn);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_ver_fit_obj, 1, 1, mp_lv_btn_get_ver_fit);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_toggle(lv_obj_t *btn, bool tgl)
 */
 
STATIC mp_obj_t mp_lv_btn_set_toggle(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    bool tgl = mp_obj_is_true(args[1]);
    lv_btn_set_toggle(btn, tgl);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_toggle_obj, 2, 2, mp_lv_btn_set_toggle);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_state(lv_obj_t *btn, lv_btn_state_t state)
 */
 
STATIC mp_obj_t mp_lv_btn_set_state(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_state_t state = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_btn_set_state(btn, state);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_state_obj, 2, 2, mp_lv_btn_set_state);

 

/*
 * lvgl extension definition for:
 * void lv_btn_toggle(lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_toggle(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_toggle(btn);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_toggle_obj, 1, 1, mp_lv_btn_toggle);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_action(lv_obj_t *btn, lv_btn_action_t type, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_btn_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_action_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    set_action(args[0], args[2]);
    lv_action_t action = &lv_action_t_callback;
    lv_btn_set_action(btn, type, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_action_obj, 3, 3, mp_lv_btn_set_action);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_ink_in_time(lv_obj_t *btn, uint16_t time)
 */
 
STATIC mp_obj_t mp_lv_btn_set_ink_in_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_btn_set_ink_in_time(btn, time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_ink_in_time_obj, 2, 2, mp_lv_btn_set_ink_in_time);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_ink_wait_time(lv_obj_t *btn, uint16_t time)
 */
 
STATIC mp_obj_t mp_lv_btn_set_ink_wait_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_btn_set_ink_wait_time(btn, time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_ink_wait_time_obj, 2, 2, mp_lv_btn_set_ink_wait_time);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_ink_out_time(lv_obj_t *btn, uint16_t time)
 */
 
STATIC mp_obj_t mp_lv_btn_set_ink_out_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_btn_set_ink_out_time(btn, time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_ink_out_time_obj, 2, 2, mp_lv_btn_set_ink_out_time);

 

/*
 * lvgl extension definition for:
 * void lv_btn_set_style(lv_obj_t *btn, lv_btn_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_btn_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_btn_set_style(btn, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_set_style_obj, 3, 3, mp_lv_btn_set_style);

 

/*
 * lvgl extension definition for:
 * lv_btn_state_t lv_btn_get_state(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_state(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_state_t res = lv_btn_get_state(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_state_obj, 1, 1, mp_lv_btn_get_state);

 

/*
 * lvgl extension definition for:
 * bool lv_btn_get_toggle(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_toggle(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    bool res = lv_btn_get_toggle(btn);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_toggle_obj, 1, 1, mp_lv_btn_get_toggle);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_btn_get_action(const lv_obj_t *btn, lv_btn_action_t type)
 */
 
STATIC mp_obj_t mp_lv_btn_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_action_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_action_t res = lv_btn_get_action(btn, type);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_action_obj, 2, 2, mp_lv_btn_get_action);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_btn_get_ink_in_time(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_ink_in_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t res = lv_btn_get_ink_in_time(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_ink_in_time_obj, 1, 1, mp_lv_btn_get_ink_in_time);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_btn_get_ink_wait_time(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_ink_wait_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t res = lv_btn_get_ink_wait_time(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_ink_wait_time_obj, 1, 1, mp_lv_btn_get_ink_wait_time);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_btn_get_ink_out_time(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_btn_get_ink_out_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    uint16_t res = lv_btn_get_ink_out_time(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_ink_out_time_obj, 1, 1, mp_lv_btn_get_ink_out_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_btn_get_style(const lv_obj_t *btn, lv_btn_style_t type)
 */
 
STATIC mp_obj_t mp_lv_btn_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_btn_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_btn_get_style(btn, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btn_get_style_obj, 2, 2, mp_lv_btn_get_style);

 

    
/*
 * lvgl btn object definitions
 */

STATIC const mp_rom_map_elem_t btn_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_btn_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_btn_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_btn_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_btn_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_btn_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_toggle), MP_ROM_PTR(&mp_lv_btn_set_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_state), MP_ROM_PTR(&mp_lv_btn_set_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_toggle), MP_ROM_PTR(&mp_lv_btn_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_btn_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_in_time), MP_ROM_PTR(&mp_lv_btn_set_ink_in_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_wait_time), MP_ROM_PTR(&mp_lv_btn_set_ink_wait_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_out_time), MP_ROM_PTR(&mp_lv_btn_set_ink_out_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_btn_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&mp_lv_btn_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_toggle), MP_ROM_PTR(&mp_lv_btn_get_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_btn_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_in_time), MP_ROM_PTR(&mp_lv_btn_get_ink_in_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_wait_time), MP_ROM_PTR(&mp_lv_btn_get_ink_wait_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_out_time), MP_ROM_PTR(&mp_lv_btn_get_ink_out_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_btn_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BTN_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_ACTION), MP_ROM_PTR(&mp_LV_BTN_ACTION_type) },
    { MP_ROM_QSTR(MP_QSTR_STATE), MP_ROM_PTR(&mp_LV_BTN_STATE_type) }
};

STATIC MP_DEFINE_CONST_DICT(btn_locals_dict, btn_locals_dict_table);

STATIC void btn_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl btn");
}


STATIC mp_obj_t btn_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_btn_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_btn_type = {
    { &mp_type_type },
    .name = MP_QSTR_btn,
    .print = btn_print,
    .make_new = btn_make_new,
    .locals_dict = (mp_obj_dict_t*)&btn_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_label_set_style(lv_obj_t *label, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_label_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_label_set_style(label, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_style_obj, 2, 2, mp_lv_label_set_style);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_label_get_style(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    lv_style_t* res = lv_label_get_style(label);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_style_obj, 1, 1, mp_lv_label_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_text(lv_obj_t *label, const char *text)
 */
 
STATIC mp_obj_t mp_lv_label_set_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    const char *text = (char*)mp_obj_str_get_str(args[1]);
    lv_label_set_text(label, text);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_text_obj, 2, 2, mp_lv_label_set_text);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_array_text(lv_obj_t *label, const char *array, uint16_t size)
 */
 
STATIC mp_obj_t mp_lv_label_set_array_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    const char *array = (char*)mp_obj_str_get_str(args[1]);
    uint16_t size = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_label_set_array_text(label, array, size);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_array_text_obj, 3, 3, mp_lv_label_set_array_text);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_static_text(lv_obj_t *label, const char *text)
 */
 
STATIC mp_obj_t mp_lv_label_set_static_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    const char *text = (char*)mp_obj_str_get_str(args[1]);
    lv_label_set_static_text(label, text);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_static_text_obj, 2, 2, mp_lv_label_set_static_text);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_long_mode(lv_obj_t *label, lv_label_long_mode_t long_mode)
 */
 
STATIC mp_obj_t mp_lv_label_set_long_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    lv_label_long_mode_t long_mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_label_set_long_mode(label, long_mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_long_mode_obj, 2, 2, mp_lv_label_set_long_mode);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_align(lv_obj_t *label, lv_label_align_t align)
 */
 
STATIC mp_obj_t mp_lv_label_set_align(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    lv_label_align_t align = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_label_set_align(label, align);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_align_obj, 2, 2, mp_lv_label_set_align);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_recolor(lv_obj_t *label, bool recolor_en)
 */
 
STATIC mp_obj_t mp_lv_label_set_recolor(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    bool recolor_en = mp_obj_is_true(args[1]);
    lv_label_set_recolor(label, recolor_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_recolor_obj, 2, 2, mp_lv_label_set_recolor);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_body_draw(lv_obj_t *label, bool body_en)
 */
 
STATIC mp_obj_t mp_lv_label_set_body_draw(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    bool body_en = mp_obj_is_true(args[1]);
    lv_label_set_body_draw(label, body_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_body_draw_obj, 2, 2, mp_lv_label_set_body_draw);

 

/*
 * lvgl extension definition for:
 * void lv_label_set_anim_speed(lv_obj_t *label, uint16_t anim_speed)
 */
 
STATIC mp_obj_t mp_lv_label_set_anim_speed(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    uint16_t anim_speed = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_label_set_anim_speed(label, anim_speed);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_set_anim_speed_obj, 2, 2, mp_lv_label_set_anim_speed);

 

/*
 * lvgl extension definition for:
 * char *lv_label_get_text(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_text(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    char* res = lv_label_get_text(label);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_text_obj, 1, 1, mp_lv_label_get_text);

 

/*
 * lvgl extension definition for:
 * lv_label_long_mode_t lv_label_get_long_mode(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_long_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    lv_label_long_mode_t res = lv_label_get_long_mode(label);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_long_mode_obj, 1, 1, mp_lv_label_get_long_mode);

 

/*
 * lvgl extension definition for:
 * lv_label_align_t lv_label_get_align(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_align(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    lv_label_align_t res = lv_label_get_align(label);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_align_obj, 1, 1, mp_lv_label_get_align);

 

/*
 * lvgl extension definition for:
 * bool lv_label_get_recolor(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_recolor(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    bool res = lv_label_get_recolor(label);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_recolor_obj, 1, 1, mp_lv_label_get_recolor);

 

/*
 * lvgl extension definition for:
 * bool lv_label_get_body_draw(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_body_draw(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    bool res = lv_label_get_body_draw(label);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_body_draw_obj, 1, 1, mp_lv_label_get_body_draw);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_label_get_anim_speed(const lv_obj_t *label)
 */
 
STATIC mp_obj_t mp_lv_label_get_anim_speed(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    uint16_t res = lv_label_get_anim_speed(label);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_anim_speed_obj, 1, 1, mp_lv_label_get_anim_speed);

 

/*
 * Struct lv_point_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_point_t_type();

STATIC inline lv_point_t* mp_write_ptr_lv_point_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_point_t*)self->data;
}

#define mp_write_lv_point_t(struct_obj) *mp_write_ptr_lv_point_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_point_t(lv_point_t *field)
{
    return lv_to_mp_struct(get_mp_lv_point_t_type(), field);
}

#define mp_read_lv_point_t(field) mp_read_ptr_lv_point_t(&field)

STATIC void mp_lv_point_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_point_t *data = (lv_point_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_x: dest[0] = mp_obj_new_int(data->x); break; // converting from lv_coord_t;
            case MP_QSTR_y: dest[0] = mp_obj_new_int(data->y); break; // converting from lv_coord_t;
            default: field_not_found(MP_QSTR_lv_point_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_x: data->x = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                case MP_QSTR_y: data->y = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_coord_t;
                default: field_not_found(MP_QSTR_lv_point_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_point_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_point_t");
}

STATIC const mp_rom_map_elem_t mp_lv_point_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_point_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_point_t_locals_dict, mp_lv_point_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_point_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_point_t,
    .print = mp_lv_point_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_point_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_point_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_point_t_type()
{
    return &mp_lv_point_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_label_get_letter_pos(const lv_obj_t *label, uint16_t index, lv_point_t *pos)
 */
 
STATIC mp_obj_t mp_lv_label_get_letter_pos(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    uint16_t index = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_point_t *pos = mp_write_ptr_lv_point_t(args[2]);
    lv_label_get_letter_pos(label, index, pos);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_letter_pos_obj, 3, 3, mp_lv_label_get_letter_pos);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_label_get_letter_on(const lv_obj_t *label, lv_point_t *pos)
 */
 
STATIC mp_obj_t mp_lv_label_get_letter_on(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *label = mp_to_lv(args[0]);
    lv_point_t *pos = mp_write_ptr_lv_point_t(args[1]);
    uint16_t res = lv_label_get_letter_on(label, pos);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_get_letter_on_obj, 2, 2, mp_lv_label_get_letter_on);

 

/*
 * lvgl extension definition for:
 * void lv_label_ins_text(lv_obj_t *label, uint32_t pos, const char *txt)
 */
 
STATIC mp_obj_t mp_lv_label_ins_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    uint32_t pos = (uint32_t)mp_obj_int_get_checked(args[1]);
    const char *txt = (char*)mp_obj_str_get_str(args[2]);
    lv_label_ins_text(label, pos, txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_ins_text_obj, 3, 3, mp_lv_label_ins_text);

 

/*
 * lvgl extension definition for:
 * void lv_label_cut_text(lv_obj_t *label, uint32_t pos, uint32_t cnt)
 */
 
STATIC mp_obj_t mp_lv_label_cut_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *label = mp_to_lv(args[0]);
    uint32_t pos = (uint32_t)mp_obj_int_get_checked(args[1]);
    uint32_t cnt = (uint32_t)mp_obj_int_get_checked(args[2]);
    lv_label_cut_text(label, pos, cnt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_label_cut_text_obj, 3, 3, mp_lv_label_cut_text);

 

    
/*
 * lvgl label object definitions
 */

STATIC const mp_rom_map_elem_t label_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_label_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_label_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_text), MP_ROM_PTR(&mp_lv_label_set_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_array_text), MP_ROM_PTR(&mp_lv_label_set_array_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_static_text), MP_ROM_PTR(&mp_lv_label_set_static_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_long_mode), MP_ROM_PTR(&mp_lv_label_set_long_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_align), MP_ROM_PTR(&mp_lv_label_set_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_recolor), MP_ROM_PTR(&mp_lv_label_set_recolor_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_body_draw), MP_ROM_PTR(&mp_lv_label_set_body_draw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_speed), MP_ROM_PTR(&mp_lv_label_set_anim_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_text), MP_ROM_PTR(&mp_lv_label_get_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_long_mode), MP_ROM_PTR(&mp_lv_label_get_long_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_align), MP_ROM_PTR(&mp_lv_label_get_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_recolor), MP_ROM_PTR(&mp_lv_label_get_recolor_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_body_draw), MP_ROM_PTR(&mp_lv_label_get_body_draw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_speed), MP_ROM_PTR(&mp_lv_label_get_anim_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_letter_pos), MP_ROM_PTR(&mp_lv_label_get_letter_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_letter_on), MP_ROM_PTR(&mp_lv_label_get_letter_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_ins_text), MP_ROM_PTR(&mp_lv_label_ins_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_cut_text), MP_ROM_PTR(&mp_lv_label_cut_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_LONG), MP_ROM_PTR(&mp_LV_LABEL_LONG_type) },
    { MP_ROM_QSTR(MP_QSTR_ALIGN), MP_ROM_PTR(&mp_LV_LABEL_ALIGN_type) }
};

STATIC MP_DEFINE_CONST_DICT(label_locals_dict, label_locals_dict_table);

STATIC void label_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl label");
}


STATIC mp_obj_t label_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_label_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_label_type = {
    { &mp_type_type },
    .name = MP_QSTR_label,
    .print = label_print,
    .make_new = label_make_new,
    .locals_dict = (mp_obj_dict_t*)&label_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * void lv_bar_set_value(lv_obj_t *bar, int16_t value)
 */
 
STATIC mp_obj_t mp_lv_bar_set_value(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_bar_set_value(bar, value);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_set_value_obj, 2, 2, mp_lv_bar_set_value);

 

/*
 * lvgl extension definition for:
 * void lv_bar_set_value_anim(lv_obj_t *bar, int16_t value, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_bar_set_value_anim(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_bar_set_value_anim(bar, value, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_set_value_anim_obj, 3, 3, mp_lv_bar_set_value_anim);

 

/*
 * lvgl extension definition for:
 * void lv_bar_set_range(lv_obj_t *bar, int16_t min, int16_t max)
 */
 
STATIC mp_obj_t mp_lv_bar_set_range(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t min = (int16_t)mp_obj_int_get_checked(args[1]);
    int16_t max = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_bar_set_range(bar, min, max);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_set_range_obj, 3, 3, mp_lv_bar_set_range);

 

/*
 * lvgl extension definition for:
 * void lv_bar_set_style(lv_obj_t *bar, lv_bar_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_bar_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *bar = mp_to_lv(args[0]);
    lv_bar_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_bar_set_style(bar, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_set_style_obj, 3, 3, mp_lv_bar_set_style);

 

/*
 * lvgl extension definition for:
 * int16_t lv_bar_get_value(const lv_obj_t *bar)
 */
 
STATIC mp_obj_t mp_lv_bar_get_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t res = lv_bar_get_value(bar);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_get_value_obj, 1, 1, mp_lv_bar_get_value);

 

/*
 * lvgl extension definition for:
 * int16_t lv_bar_get_min_value(const lv_obj_t *bar)
 */
 
STATIC mp_obj_t mp_lv_bar_get_min_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t res = lv_bar_get_min_value(bar);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_get_min_value_obj, 1, 1, mp_lv_bar_get_min_value);

 

/*
 * lvgl extension definition for:
 * int16_t lv_bar_get_max_value(const lv_obj_t *bar)
 */
 
STATIC mp_obj_t mp_lv_bar_get_max_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *bar = mp_to_lv(args[0]);
    int16_t res = lv_bar_get_max_value(bar);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_get_max_value_obj, 1, 1, mp_lv_bar_get_max_value);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_bar_get_style(const lv_obj_t *bar, lv_bar_style_t type)
 */
 
STATIC mp_obj_t mp_lv_bar_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *bar = mp_to_lv(args[0]);
    lv_bar_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_bar_get_style(bar, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_bar_get_style_obj, 2, 2, mp_lv_bar_get_style);

 

    
/*
 * lvgl bar object definitions
 */

STATIC const mp_rom_map_elem_t bar_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_bar_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value_anim), MP_ROM_PTR(&mp_lv_bar_set_value_anim_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_bar_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_bar_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_bar_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_bar_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_bar_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_bar_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BAR_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(bar_locals_dict, bar_locals_dict_table);

STATIC void bar_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl bar");
}


STATIC mp_obj_t bar_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_bar_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_bar_type = {
    { &mp_type_type },
    .name = MP_QSTR_bar,
    .print = bar_print,
    .make_new = bar_make_new,
    .locals_dict = (mp_obj_dict_t*)&bar_locals_dict,
};
    

/*
 * Function NOT generated:
 * Missing conversion to char**
 * void lv_btnm_set_map(lv_obj_t *btnm, const char **map)
 */
    

/*
 * lvgl extension definition for:
 * void lv_btnm_set_action(lv_obj_t *btnm, lv_btnm_action_t action)
 */
 
STATIC mp_obj_t mp_lv_btnm_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btnm = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_btnm_action_t action = &lv_btnm_action_t_callback;
    lv_btnm_set_action(btnm, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_set_action_obj, 2, 2, mp_lv_btnm_set_action);

 

/*
 * lvgl extension definition for:
 * void lv_btnm_set_toggle(lv_obj_t *btnm, bool en, uint16_t id)
 */
 
STATIC mp_obj_t mp_lv_btnm_set_toggle(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btnm = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    uint16_t id = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_btnm_set_toggle(btnm, en, id);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_set_toggle_obj, 3, 3, mp_lv_btnm_set_toggle);

 

/*
 * lvgl extension definition for:
 * void lv_btnm_set_style(lv_obj_t *btnm, lv_btnm_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_btnm_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btnm = mp_to_lv(args[0]);
    lv_btnm_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_btnm_set_style(btnm, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_set_style_obj, 3, 3, mp_lv_btnm_set_style);

 

/*
 * Function NOT generated:
 * Missing convertion from const char**
 * const char **lv_btnm_get_map(const lv_obj_t *btnm)
 */
    

/*
 * lvgl extension definition for:
 * lv_btnm_action_t lv_btnm_get_action(const lv_obj_t *btnm)
 */
 
STATIC mp_obj_t mp_lv_btnm_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btnm = mp_to_lv(args[0]);
    lv_btnm_action_t res = lv_btnm_get_action(btnm);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_get_action_obj, 1, 1, mp_lv_btnm_get_action);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_btnm_get_toggled(const lv_obj_t *btnm)
 */
 
STATIC mp_obj_t mp_lv_btnm_get_toggled(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btnm = mp_to_lv(args[0]);
    uint16_t res = lv_btnm_get_toggled(btnm);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_get_toggled_obj, 1, 1, mp_lv_btnm_get_toggled);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_btnm_get_style(const lv_obj_t *btnm, lv_btnm_style_t type)
 */
 
STATIC mp_obj_t mp_lv_btnm_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btnm = mp_to_lv(args[0]);
    lv_btnm_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_btnm_get_style(btnm, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_btnm_get_style_obj, 2, 2, mp_lv_btnm_get_style);

 

    
/*
 * lvgl btnm object definitions
 */

STATIC const mp_rom_map_elem_t btnm_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_btnm_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_toggle), MP_ROM_PTR(&mp_lv_btnm_set_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_btnm_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_btnm_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_toggled), MP_ROM_PTR(&mp_lv_btnm_get_toggled_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_btnm_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BTNM_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(btnm_locals_dict, btnm_locals_dict_table);

STATIC void btnm_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl btnm");
}


STATIC mp_obj_t btnm_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_btnm_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_btnm_type = {
    { &mp_type_type },
    .name = MP_QSTR_btnm,
    .print = btnm_print,
    .make_new = btnm_make_new,
    .locals_dict = (mp_obj_dict_t*)&btnm_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_cb_set_checked(lv_obj_t *cb, bool checked)
 */
 
STATIC mp_obj_t mp_lv_cb_set_checked(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cb = mp_to_lv(args[0]);
    bool checked = mp_obj_is_true(args[1]);
    lv_cb_set_checked(cb, checked);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_set_checked_obj, 2, 2, mp_lv_cb_set_checked);

 

/*
 * lvgl extension definition for:
 * inline static void lv_cb_set_inactive(lv_obj_t *cb)
 */
 
STATIC mp_obj_t mp_lv_cb_set_inactive(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cb = mp_to_lv(args[0]);
    lv_cb_set_inactive(cb);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_set_inactive_obj, 1, 1, mp_lv_cb_set_inactive);

 

/*
 * lvgl extension definition for:
 * inline static void lv_cb_set_action(lv_obj_t *cb, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_cb_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cb = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_cb_set_action(cb, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_set_action_obj, 2, 2, mp_lv_cb_set_action);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_cb_is_checked(const lv_obj_t *cb)
 */
 
STATIC mp_obj_t mp_lv_cb_is_checked(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cb = mp_to_lv(args[0]);
    bool res = lv_cb_is_checked(cb);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_is_checked_obj, 1, 1, mp_lv_cb_is_checked);

 

/*
 * lvgl extension definition for:
 * inline static lv_action_t lv_cb_get_action(const lv_obj_t *cb)
 */
 
STATIC mp_obj_t mp_lv_cb_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cb = mp_to_lv(args[0]);
    lv_action_t res = lv_cb_get_action(cb);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_get_action_obj, 1, 1, mp_lv_cb_get_action);

 

/*
 * lvgl extension definition for:
 * void lv_cb_set_text(lv_obj_t *cb, const char *txt)
 */
 
STATIC mp_obj_t mp_lv_cb_set_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cb = mp_to_lv(args[0]);
    const char *txt = (char*)mp_obj_str_get_str(args[1]);
    lv_cb_set_text(cb, txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_set_text_obj, 2, 2, mp_lv_cb_set_text);

 

/*
 * lvgl extension definition for:
 * void lv_cb_set_style(lv_obj_t *cb, lv_cb_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_cb_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *cb = mp_to_lv(args[0]);
    lv_cb_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_cb_set_style(cb, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_set_style_obj, 3, 3, mp_lv_cb_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_cb_get_text(const lv_obj_t *cb)
 */
 
STATIC mp_obj_t mp_lv_cb_get_text(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cb = mp_to_lv(args[0]);
    const char* res = lv_cb_get_text(cb);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_get_text_obj, 1, 1, mp_lv_cb_get_text);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_cb_get_style(const lv_obj_t *cb, lv_cb_style_t type)
 */
 
STATIC mp_obj_t mp_lv_cb_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *cb = mp_to_lv(args[0]);
    lv_cb_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_cb_get_style(cb, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_cb_get_style_obj, 2, 2, mp_lv_cb_get_style);

 

    
/*
 * lvgl cb object definitions
 */

STATIC const mp_rom_map_elem_t cb_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_checked), MP_ROM_PTR(&mp_lv_cb_set_checked_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_inactive), MP_ROM_PTR(&mp_lv_cb_set_inactive_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_cb_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_checked), MP_ROM_PTR(&mp_lv_cb_is_checked_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_cb_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_text), MP_ROM_PTR(&mp_lv_cb_set_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cb_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_text), MP_ROM_PTR(&mp_lv_cb_get_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cb_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_btn_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_btn_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_btn_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_btn_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_btn_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_toggle), MP_ROM_PTR(&mp_lv_btn_set_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_state), MP_ROM_PTR(&mp_lv_btn_set_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_toggle), MP_ROM_PTR(&mp_lv_btn_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_btn_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_in_time), MP_ROM_PTR(&mp_lv_btn_set_ink_in_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_wait_time), MP_ROM_PTR(&mp_lv_btn_set_ink_wait_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ink_out_time), MP_ROM_PTR(&mp_lv_btn_set_ink_out_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_btn_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&mp_lv_btn_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_toggle), MP_ROM_PTR(&mp_lv_btn_get_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_btn_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_in_time), MP_ROM_PTR(&mp_lv_btn_get_ink_in_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_wait_time), MP_ROM_PTR(&mp_lv_btn_get_ink_wait_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ink_out_time), MP_ROM_PTR(&mp_lv_btn_get_ink_out_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_btn_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BTN_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_ACTION), MP_ROM_PTR(&mp_LV_BTN_ACTION_type) },
    { MP_ROM_QSTR(MP_QSTR_STATE), MP_ROM_PTR(&mp_LV_BTN_STATE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_CB_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(cb_locals_dict, cb_locals_dict_table);

STATIC void cb_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl cb");
}


STATIC mp_obj_t cb_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_cb_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_cb_type = {
    { &mp_type_type },
    .name = MP_QSTR_cb,
    .print = cb_print,
    .make_new = cb_make_new,
    .locals_dict = (mp_obj_dict_t*)&cb_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_line_set_style(lv_obj_t *line, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_line_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *line = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_line_set_style(line, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_set_style_obj, 2, 2, mp_lv_line_set_style);

 

/*
 * lvgl extension definition for:
 * inline static void lv_line_set_upscale(lv_obj_t *line, bool upcale)
 */
 
STATIC mp_obj_t mp_lv_line_set_upscale(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *line = mp_to_lv(args[0]);
    bool upcale = mp_obj_is_true(args[1]);
    lv_line_set_upscale(line, upcale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_set_upscale_obj, 2, 2, mp_lv_line_set_upscale);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_line_get_style(const lv_obj_t *line)
 */
 
STATIC mp_obj_t mp_lv_line_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *line = mp_to_lv(args[0]);
    lv_style_t* res = lv_line_get_style(line);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_get_style_obj, 1, 1, mp_lv_line_get_style);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_line_get_upscale(const lv_obj_t *line)
 */
 
STATIC mp_obj_t mp_lv_line_get_upscale(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *line = mp_to_lv(args[0]);
    bool res = lv_line_get_upscale(line);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_get_upscale_obj, 1, 1, mp_lv_line_get_upscale);

 

/*
 * lvgl extension definition for:
 * void lv_line_set_points(lv_obj_t *line, const lv_point_t *point_a, uint16_t point_num)
 */
 
STATIC mp_obj_t mp_lv_line_set_points(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *line = mp_to_lv(args[0]);
    const lv_point_t *point_a = mp_write_ptr_lv_point_t(args[1]);
    uint16_t point_num = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_line_set_points(line, point_a, point_num);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_set_points_obj, 3, 3, mp_lv_line_set_points);

 

/*
 * lvgl extension definition for:
 * void lv_line_set_auto_size(lv_obj_t *line, bool autosize_en)
 */
 
STATIC mp_obj_t mp_lv_line_set_auto_size(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *line = mp_to_lv(args[0]);
    bool autosize_en = mp_obj_is_true(args[1]);
    lv_line_set_auto_size(line, autosize_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_set_auto_size_obj, 2, 2, mp_lv_line_set_auto_size);

 

/*
 * lvgl extension definition for:
 * void lv_line_set_y_invert(lv_obj_t *line, bool yinv_en)
 */
 
STATIC mp_obj_t mp_lv_line_set_y_invert(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *line = mp_to_lv(args[0]);
    bool yinv_en = mp_obj_is_true(args[1]);
    lv_line_set_y_invert(line, yinv_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_set_y_invert_obj, 2, 2, mp_lv_line_set_y_invert);

 

/*
 * lvgl extension definition for:
 * bool lv_line_get_auto_size(const lv_obj_t *line)
 */
 
STATIC mp_obj_t mp_lv_line_get_auto_size(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *line = mp_to_lv(args[0]);
    bool res = lv_line_get_auto_size(line);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_get_auto_size_obj, 1, 1, mp_lv_line_get_auto_size);

 

/*
 * lvgl extension definition for:
 * bool lv_line_get_y_inv(const lv_obj_t *line)
 */
 
STATIC mp_obj_t mp_lv_line_get_y_inv(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *line = mp_to_lv(args[0]);
    bool res = lv_line_get_y_inv(line);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_line_get_y_inv_obj, 1, 1, mp_lv_line_get_y_inv);

 

    
/*
 * lvgl line object definitions
 */

STATIC const mp_rom_map_elem_t line_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_line_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_upscale), MP_ROM_PTR(&mp_lv_line_set_upscale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_line_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_upscale), MP_ROM_PTR(&mp_lv_line_get_upscale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_points), MP_ROM_PTR(&mp_lv_line_set_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_auto_size), MP_ROM_PTR(&mp_lv_line_set_auto_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y_invert), MP_ROM_PTR(&mp_lv_line_set_y_invert_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_auto_size), MP_ROM_PTR(&mp_lv_line_get_auto_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y_inv), MP_ROM_PTR(&mp_lv_line_get_y_inv_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(line_locals_dict, line_locals_dict_table);

STATIC void line_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl line");
}


STATIC mp_obj_t line_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_line_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_line_type = {
    { &mp_type_type },
    .name = MP_QSTR_line,
    .print = line_print,
    .make_new = line_make_new,
    .locals_dict = (mp_obj_dict_t*)&line_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_chart_set_style(lv_obj_t *chart, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_chart_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_chart_set_style(chart, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_style_obj, 2, 2, mp_lv_chart_set_style);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_chart_get_style(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    lv_style_t* res = lv_chart_get_style(chart);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_style_obj, 1, 1, mp_lv_chart_get_style);

 

/*
 * Struct lv_chart_series_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_chart_series_t_type();

STATIC inline lv_chart_series_t* mp_write_ptr_lv_chart_series_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_chart_series_t*)self->data;
}

#define mp_write_lv_chart_series_t(struct_obj) *mp_write_ptr_lv_chart_series_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_chart_series_t(lv_chart_series_t *field)
{
    return lv_to_mp_struct(get_mp_lv_chart_series_t_type(), field);
}

#define mp_read_lv_chart_series_t(field) mp_read_ptr_lv_chart_series_t(&field)

STATIC void mp_lv_chart_series_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_chart_series_t *data = (lv_chart_series_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_points: dest[0] = ptr_to_mp((void*)data->points); break; // converting from lv_coord_t*;
            case MP_QSTR_color: dest[0] = mp_read_lv_color16_t(data->color); break; // converting from lv_color_t;
            default: field_not_found(MP_QSTR_lv_chart_series_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_points: data->points = (void*)mp_to_ptr(dest[1]); break; // converting to lv_coord_t*;
                case MP_QSTR_color: data->color = mp_write_lv_color16_t(dest[1]); break; // converting to lv_color_t;
                default: field_not_found(MP_QSTR_lv_chart_series_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_chart_series_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_chart_series_t");
}

STATIC const mp_rom_map_elem_t mp_lv_chart_series_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_chart_series_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_chart_series_t_locals_dict, mp_lv_chart_series_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_chart_series_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_chart_series_t,
    .print = mp_lv_chart_series_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_chart_series_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_chart_series_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_chart_series_t_type()
{
    return &mp_lv_chart_series_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_chart_series_t *lv_chart_add_series(lv_obj_t *chart, lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_chart_add_series(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_color_t color = mp_write_lv_color16_t(args[1]);
    lv_chart_series_t* res = lv_chart_add_series(chart, color);
    return mp_read_ptr_lv_chart_series_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_add_series_obj, 2, 2, mp_lv_chart_add_series);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_div_line_count(lv_obj_t *chart, uint8_t hdiv, uint8_t vdiv)
 */
 
STATIC mp_obj_t mp_lv_chart_set_div_line_count(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    uint8_t hdiv = (uint8_t)mp_obj_int_get_checked(args[1]);
    uint8_t vdiv = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_chart_set_div_line_count(chart, hdiv, vdiv);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_div_line_count_obj, 3, 3, mp_lv_chart_set_div_line_count);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_range(lv_obj_t *chart, lv_coord_t ymin, lv_coord_t ymax)
 */
 
STATIC mp_obj_t mp_lv_chart_set_range(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_coord_t ymin = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t ymax = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_chart_set_range(chart, ymin, ymax);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_range_obj, 3, 3, mp_lv_chart_set_range);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_type(lv_obj_t *chart, lv_chart_type_t type)
 */
 
STATIC mp_obj_t mp_lv_chart_set_type(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_type_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_chart_set_type(chart, type);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_type_obj, 2, 2, mp_lv_chart_set_type);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_point_count(lv_obj_t *chart, uint16_t point_cnt)
 */
 
STATIC mp_obj_t mp_lv_chart_set_point_count(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    uint16_t point_cnt = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_chart_set_point_count(chart, point_cnt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_point_count_obj, 2, 2, mp_lv_chart_set_point_count);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_series_opa(lv_obj_t *chart, lv_opa_t opa)
 */
 
STATIC mp_obj_t mp_lv_chart_set_series_opa(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_opa_t opa = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_chart_set_series_opa(chart, opa);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_series_opa_obj, 2, 2, mp_lv_chart_set_series_opa);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_series_width(lv_obj_t *chart, lv_coord_t width)
 */
 
STATIC mp_obj_t mp_lv_chart_set_series_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_coord_t width = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_chart_set_series_width(chart, width);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_series_width_obj, 2, 2, mp_lv_chart_set_series_width);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_series_darking(lv_obj_t *chart, lv_opa_t dark_eff)
 */
 
STATIC mp_obj_t mp_lv_chart_set_series_darking(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_opa_t dark_eff = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_chart_set_series_darking(chart, dark_eff);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_series_darking_obj, 2, 2, mp_lv_chart_set_series_darking);

 

/*
 * lvgl extension definition for:
 * void lv_chart_init_points(lv_obj_t *chart, lv_chart_series_t *ser, lv_coord_t y)
 */
 
STATIC mp_obj_t mp_lv_chart_init_points(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_series_t *ser = mp_write_ptr_lv_chart_series_t(args[1]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_chart_init_points(chart, ser, y);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_init_points_obj, 3, 3, mp_lv_chart_init_points);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_points(lv_obj_t *chart, lv_chart_series_t *ser, lv_coord_t *y_array)
 */
 
STATIC mp_obj_t mp_lv_chart_set_points(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_series_t *ser = mp_write_ptr_lv_chart_series_t(args[1]);
    lv_coord_t *y_array = mp_to_ptr(args[2]);
    lv_chart_set_points(chart, ser, y_array);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_points_obj, 3, 3, mp_lv_chart_set_points);

 

/*
 * lvgl extension definition for:
 * void lv_chart_set_next(lv_obj_t *chart, lv_chart_series_t *ser, lv_coord_t y)
 */
 
STATIC mp_obj_t mp_lv_chart_set_next(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_series_t *ser = mp_write_ptr_lv_chart_series_t(args[1]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_chart_set_next(chart, ser, y);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_set_next_obj, 3, 3, mp_lv_chart_set_next);

 

/*
 * lvgl extension definition for:
 * lv_chart_type_t lv_chart_get_type(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_type(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_type_t res = lv_chart_get_type(chart);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_type_obj, 1, 1, mp_lv_chart_get_type);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_chart_get_point_cnt(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_point_cnt(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    uint16_t res = lv_chart_get_point_cnt(chart);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_point_cnt_obj, 1, 1, mp_lv_chart_get_point_cnt);

 

/*
 * lvgl extension definition for:
 * lv_opa_t lv_chart_get_series_opa(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_series_opa(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    lv_opa_t res = lv_chart_get_series_opa(chart);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_series_opa_obj, 1, 1, mp_lv_chart_get_series_opa);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_chart_get_series_width(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_series_width(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    lv_coord_t res = lv_chart_get_series_width(chart);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_series_width_obj, 1, 1, mp_lv_chart_get_series_width);

 

/*
 * lvgl extension definition for:
 * lv_opa_t lv_chart_get_series_darking(const lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_get_series_darking(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *chart = mp_to_lv(args[0]);
    lv_opa_t res = lv_chart_get_series_darking(chart);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_get_series_darking_obj, 1, 1, mp_lv_chart_get_series_darking);

 

/*
 * lvgl extension definition for:
 * void lv_chart_refresh(lv_obj_t *chart)
 */
 
STATIC mp_obj_t mp_lv_chart_refresh(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *chart = mp_to_lv(args[0]);
    lv_chart_refresh(chart);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_chart_refresh_obj, 1, 1, mp_lv_chart_refresh);

 

    
/*
 * lvgl chart object definitions
 */

STATIC const mp_rom_map_elem_t chart_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_chart_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_chart_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_series), MP_ROM_PTR(&mp_lv_chart_add_series_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_div_line_count), MP_ROM_PTR(&mp_lv_chart_set_div_line_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_chart_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_type), MP_ROM_PTR(&mp_lv_chart_set_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_point_count), MP_ROM_PTR(&mp_lv_chart_set_point_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_series_opa), MP_ROM_PTR(&mp_lv_chart_set_series_opa_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_series_width), MP_ROM_PTR(&mp_lv_chart_set_series_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_series_darking), MP_ROM_PTR(&mp_lv_chart_set_series_darking_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_points), MP_ROM_PTR(&mp_lv_chart_init_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_points), MP_ROM_PTR(&mp_lv_chart_set_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_next), MP_ROM_PTR(&mp_lv_chart_set_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_type), MP_ROM_PTR(&mp_lv_chart_get_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_point_cnt), MP_ROM_PTR(&mp_lv_chart_get_point_cnt_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_series_opa), MP_ROM_PTR(&mp_lv_chart_get_series_opa_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_series_width), MP_ROM_PTR(&mp_lv_chart_get_series_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_series_darking), MP_ROM_PTR(&mp_lv_chart_get_series_darking_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh), MP_ROM_PTR(&mp_lv_chart_refresh_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_TYPE), MP_ROM_PTR(&mp_LV_CHART_TYPE_type) }
};

STATIC MP_DEFINE_CONST_DICT(chart_locals_dict, chart_locals_dict_table);

STATIC void chart_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl chart");
}


STATIC mp_obj_t chart_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_chart_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_chart_type = {
    { &mp_type_type },
    .name = MP_QSTR_chart,
    .print = chart_print,
    .make_new = chart_make_new,
    .locals_dict = (mp_obj_dict_t*)&chart_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_page_set_scrl_fit(lv_obj_t *page, bool hor_en, bool ver_en)
 */
 
STATIC mp_obj_t mp_lv_page_set_scrl_fit(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    bool hor_en = mp_obj_is_true(args[1]);
    bool ver_en = mp_obj_is_true(args[2]);
    lv_page_set_scrl_fit(page, hor_en, ver_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_scrl_fit_obj, 3, 3, mp_lv_page_set_scrl_fit);

 

/*
 * lvgl extension definition for:
 * inline static void lv_page_set_scrl_width(lv_obj_t *page, lv_coord_t w)
 */
 
STATIC mp_obj_t mp_lv_page_set_scrl_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t w = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_page_set_scrl_width(page, w);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_scrl_width_obj, 2, 2, mp_lv_page_set_scrl_width);

 

/*
 * lvgl extension definition for:
 * inline static void lv_page_set_scrl_height(lv_obj_t *page, lv_coord_t h)
 */
 
STATIC mp_obj_t mp_lv_page_set_scrl_height(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t h = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_page_set_scrl_height(page, h);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_scrl_height_obj, 2, 2, mp_lv_page_set_scrl_height);

 

/*
 * lvgl extension definition for:
 * inline static void lv_page_set_scrl_layout(lv_obj_t *page, lv_layout_t layout)
 */
 
STATIC mp_obj_t mp_lv_page_set_scrl_layout(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_layout_t layout = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_page_set_scrl_layout(page, layout);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_scrl_layout_obj, 2, 2, mp_lv_page_set_scrl_layout);

 

/*
 * lvgl extension definition for:
 * inline static lv_coord_t lv_page_get_scrl_width(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl_width(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t res = lv_page_get_scrl_width(page);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_width_obj, 1, 1, mp_lv_page_get_scrl_width);

 

/*
 * lvgl extension definition for:
 * inline static lv_coord_t lv_page_get_scrl_height(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl_height(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t res = lv_page_get_scrl_height(page);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_height_obj, 1, 1, mp_lv_page_get_scrl_height);

 

/*
 * lvgl extension definition for:
 * inline static lv_layout_t lv_page_get_scrl_layout(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl_layout(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_layout_t res = lv_page_get_scrl_layout(page);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_layout_obj, 1, 1, mp_lv_page_get_scrl_layout);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_page_get_scrl_hor_fit(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl_hor_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    bool res = lv_page_get_scrl_hor_fit(page);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_hor_fit_obj, 1, 1, mp_lv_page_get_scrl_hor_fit);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_page_get_scrl_fit_ver(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl_fit_ver(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    bool res = lv_page_get_scrl_fit_ver(page);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_fit_ver_obj, 1, 1, mp_lv_page_get_scrl_fit_ver);

 

/*
 * lvgl extension definition for:
 * void lv_page_clean(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_page_clean(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_page_clean(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_clean_obj, 1, 1, mp_lv_page_clean);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_page_get_pr_action(lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_pr_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_action_t res = lv_page_get_pr_action(page);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_pr_action_obj, 1, 1, mp_lv_page_get_pr_action);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_page_get_rel_action(lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_rel_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_action_t res = lv_page_get_rel_action(page);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_rel_action_obj, 1, 1, mp_lv_page_get_rel_action);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_page_get_scrl(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_scrl(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_obj_t* res = lv_page_get_scrl(page);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_scrl_obj, 1, 1, mp_lv_page_get_scrl);

 

/*
 * lvgl extension definition for:
 * void lv_page_set_rel_action(lv_obj_t *page, lv_action_t rel_action)
 */
 
STATIC mp_obj_t mp_lv_page_set_rel_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t rel_action = &lv_action_t_callback;
    lv_page_set_rel_action(page, rel_action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_rel_action_obj, 2, 2, mp_lv_page_set_rel_action);

 

/*
 * lvgl extension definition for:
 * void lv_page_set_pr_action(lv_obj_t *page, lv_action_t pr_action)
 */
 
STATIC mp_obj_t mp_lv_page_set_pr_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t pr_action = &lv_action_t_callback;
    lv_page_set_pr_action(page, pr_action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_pr_action_obj, 2, 2, mp_lv_page_set_pr_action);

 

/*
 * lvgl extension definition for:
 * void lv_page_set_sb_mode(lv_obj_t *page, lv_sb_mode_t sb_mode)
 */
 
STATIC mp_obj_t mp_lv_page_set_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_sb_mode_t sb_mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_page_set_sb_mode(page, sb_mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_sb_mode_obj, 2, 2, mp_lv_page_set_sb_mode);

 

/*
 * lvgl extension definition for:
 * void lv_page_set_arrow_scroll(lv_obj_t *page, bool en)
 */
 
STATIC mp_obj_t mp_lv_page_set_arrow_scroll(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_page_set_arrow_scroll(page, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_arrow_scroll_obj, 2, 2, mp_lv_page_set_arrow_scroll);

 

/*
 * lvgl extension definition for:
 * void lv_page_set_style(lv_obj_t *page, lv_page_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_page_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_page_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_page_set_style(page, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_set_style_obj, 3, 3, mp_lv_page_set_style);

 

/*
 * lvgl extension definition for:
 * lv_sb_mode_t lv_page_get_sb_mode(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_sb_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_sb_mode_t res = lv_page_get_sb_mode(page);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_sb_mode_obj, 1, 1, mp_lv_page_get_sb_mode);

 

/*
 * lvgl extension definition for:
 * bool lv_page_get_arrow_scroll(const lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_arrow_scroll(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    bool res = lv_page_get_arrow_scroll(page);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_arrow_scroll_obj, 1, 1, mp_lv_page_get_arrow_scroll);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_page_get_fit_width(lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_fit_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t res = lv_page_get_fit_width(page);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_fit_width_obj, 1, 1, mp_lv_page_get_fit_width);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_page_get_fit_height(lv_obj_t *page)
 */
 
STATIC mp_obj_t mp_lv_page_get_fit_height(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t res = lv_page_get_fit_height(page);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_fit_height_obj, 1, 1, mp_lv_page_get_fit_height);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_page_get_style(const lv_obj_t *page, lv_page_style_t type)
 */
 
STATIC mp_obj_t mp_lv_page_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *page = mp_to_lv(args[0]);
    lv_page_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_page_get_style(page, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_get_style_obj, 2, 2, mp_lv_page_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_page_glue_obj(lv_obj_t *obj, bool glue)
 */
 
STATIC mp_obj_t mp_lv_page_glue_obj(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    bool glue = mp_obj_is_true(args[1]);
    lv_page_glue_obj(obj, glue);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_glue_obj_obj, 2, 2, mp_lv_page_glue_obj);

 

/*
 * lvgl extension definition for:
 * void lv_page_focus(lv_obj_t *page, const lv_obj_t *obj, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_page_focus(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    const lv_obj_t *obj = mp_to_lv(args[1]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_page_focus(page, obj, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_focus_obj, 3, 3, mp_lv_page_focus);

 

/*
 * lvgl extension definition for:
 * void lv_page_scroll_hor(lv_obj_t *page, lv_coord_t dist)
 */
 
STATIC mp_obj_t mp_lv_page_scroll_hor(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t dist = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_page_scroll_hor(page, dist);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_scroll_hor_obj, 2, 2, mp_lv_page_scroll_hor);

 

/*
 * lvgl extension definition for:
 * void lv_page_scroll_ver(lv_obj_t *page, lv_coord_t dist)
 */
 
STATIC mp_obj_t mp_lv_page_scroll_ver(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *page = mp_to_lv(args[0]);
    lv_coord_t dist = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_page_scroll_ver(page, dist);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_page_scroll_ver_obj, 2, 2, mp_lv_page_scroll_ver);

 

    
/*
 * lvgl page object definitions
 */

STATIC const mp_rom_map_elem_t page_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_scrl_fit), MP_ROM_PTR(&mp_lv_page_set_scrl_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_width), MP_ROM_PTR(&mp_lv_page_set_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_height), MP_ROM_PTR(&mp_lv_page_set_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_layout), MP_ROM_PTR(&mp_lv_page_set_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_width), MP_ROM_PTR(&mp_lv_page_get_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_height), MP_ROM_PTR(&mp_lv_page_get_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_layout), MP_ROM_PTR(&mp_lv_page_get_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_hor_fit), MP_ROM_PTR(&mp_lv_page_get_scrl_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_fit_ver), MP_ROM_PTR(&mp_lv_page_get_scrl_fit_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_page_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pr_action), MP_ROM_PTR(&mp_lv_page_get_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rel_action), MP_ROM_PTR(&mp_lv_page_get_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl), MP_ROM_PTR(&mp_lv_page_get_scrl_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rel_action), MP_ROM_PTR(&mp_lv_page_set_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pr_action), MP_ROM_PTR(&mp_lv_page_set_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_page_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arrow_scroll), MP_ROM_PTR(&mp_lv_page_set_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_page_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_page_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arrow_scroll), MP_ROM_PTR(&mp_lv_page_get_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_page_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_page_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_page_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_glue_obj), MP_ROM_PTR(&mp_lv_page_glue_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_page_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_page_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_page_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PAGE_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(page_locals_dict, page_locals_dict_table);

STATIC void page_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl page");
}


STATIC mp_obj_t page_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_page_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_page_type = {
    { &mp_type_type },
    .name = MP_QSTR_page,
    .print = page_print,
    .make_new = page_make_new,
    .locals_dict = (mp_obj_dict_t*)&page_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_ddlist_set_sb_mode(lv_obj_t *ddlist, lv_sb_mode_t mode)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_sb_mode_t mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_ddlist_set_sb_mode(ddlist, mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_sb_mode_obj, 2, 2, mp_lv_ddlist_set_sb_mode);

 

/*
 * lvgl extension definition for:
 * inline static lv_sb_mode_t lv_ddlist_get_sb_mode(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_sb_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_sb_mode_t res = lv_ddlist_get_sb_mode(ddlist);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_sb_mode_obj, 1, 1, mp_lv_ddlist_get_sb_mode);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_options(lv_obj_t *ddlist, const char *options)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_options(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    const char *options = (char*)mp_obj_str_get_str(args[1]);
    lv_ddlist_set_options(ddlist, options);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_options_obj, 2, 2, mp_lv_ddlist_set_options);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_selected(lv_obj_t *ddlist, uint16_t sel_opt)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_selected(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    uint16_t sel_opt = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_ddlist_set_selected(ddlist, sel_opt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_selected_obj, 2, 2, mp_lv_ddlist_set_selected);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_action(lv_obj_t *ddlist, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_ddlist_set_action(ddlist, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_action_obj, 2, 2, mp_lv_ddlist_set_action);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_fix_height(lv_obj_t *ddlist, lv_coord_t h)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_fix_height(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_coord_t h = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_ddlist_set_fix_height(ddlist, h);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_fix_height_obj, 2, 2, mp_lv_ddlist_set_fix_height);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_hor_fit(lv_obj_t *ddlist, bool fit_en)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_hor_fit(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    bool fit_en = mp_obj_is_true(args[1]);
    lv_ddlist_set_hor_fit(ddlist, fit_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_hor_fit_obj, 2, 2, mp_lv_ddlist_set_hor_fit);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_anim_time(lv_obj_t *ddlist, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_anim_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_ddlist_set_anim_time(ddlist, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_anim_time_obj, 2, 2, mp_lv_ddlist_set_anim_time);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_set_style(lv_obj_t *ddlist, lv_ddlist_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_ddlist_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_ddlist_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_ddlist_set_style(ddlist, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_set_style_obj, 3, 3, mp_lv_ddlist_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_ddlist_get_options(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_options(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    const char* res = lv_ddlist_get_options(ddlist);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_options_obj, 1, 1, mp_lv_ddlist_get_options);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_ddlist_get_selected(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_selected(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    uint16_t res = lv_ddlist_get_selected(ddlist);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_selected_obj, 1, 1, mp_lv_ddlist_get_selected);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_get_selected_str(const lv_obj_t *ddlist, char *buf)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_selected_str(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    char *buf = (char*)mp_obj_str_get_str(args[1]);
    lv_ddlist_get_selected_str(ddlist, buf);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_selected_str_obj, 2, 2, mp_lv_ddlist_get_selected_str);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_ddlist_get_action(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_action_t res = lv_ddlist_get_action(ddlist);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_action_obj, 1, 1, mp_lv_ddlist_get_action);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_ddlist_get_fix_height(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_fix_height(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_coord_t res = lv_ddlist_get_fix_height(ddlist);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_fix_height_obj, 1, 1, mp_lv_ddlist_get_fix_height);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_ddlist_get_anim_time(const lv_obj_t *ddlist)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_anim_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    uint16_t res = lv_ddlist_get_anim_time(ddlist);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_anim_time_obj, 1, 1, mp_lv_ddlist_get_anim_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_ddlist_get_style(const lv_obj_t *ddlist, lv_ddlist_style_t type)
 */
 
STATIC mp_obj_t mp_lv_ddlist_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ddlist = mp_to_lv(args[0]);
    lv_ddlist_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_ddlist_get_style(ddlist, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_get_style_obj, 2, 2, mp_lv_ddlist_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_open(lv_obj_t *ddlist, bool anim)
 */
 
STATIC mp_obj_t mp_lv_ddlist_open(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    bool anim = mp_obj_is_true(args[1]);
    lv_ddlist_open(ddlist, anim);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_open_obj, 2, 2, mp_lv_ddlist_open);

 

/*
 * lvgl extension definition for:
 * void lv_ddlist_close(lv_obj_t *ddlist, bool anim)
 */
 
STATIC mp_obj_t mp_lv_ddlist_close(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ddlist = mp_to_lv(args[0]);
    bool anim = mp_obj_is_true(args[1]);
    lv_ddlist_close(ddlist, anim);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ddlist_close_obj, 2, 2, mp_lv_ddlist_close);

 

    
/*
 * lvgl ddlist object definitions
 */

STATIC const mp_rom_map_elem_t ddlist_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_ddlist_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_ddlist_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_options), MP_ROM_PTR(&mp_lv_ddlist_set_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_selected), MP_ROM_PTR(&mp_lv_ddlist_set_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_ddlist_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fix_height), MP_ROM_PTR(&mp_lv_ddlist_set_fix_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hor_fit), MP_ROM_PTR(&mp_lv_ddlist_set_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_ddlist_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_ddlist_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_options), MP_ROM_PTR(&mp_lv_ddlist_get_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected), MP_ROM_PTR(&mp_lv_ddlist_get_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected_str), MP_ROM_PTR(&mp_lv_ddlist_get_selected_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_ddlist_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fix_height), MP_ROM_PTR(&mp_lv_ddlist_get_fix_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_ddlist_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_ddlist_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&mp_lv_ddlist_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_lv_ddlist_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_fit), MP_ROM_PTR(&mp_lv_page_set_scrl_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_width), MP_ROM_PTR(&mp_lv_page_set_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_height), MP_ROM_PTR(&mp_lv_page_set_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_layout), MP_ROM_PTR(&mp_lv_page_set_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_width), MP_ROM_PTR(&mp_lv_page_get_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_height), MP_ROM_PTR(&mp_lv_page_get_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_layout), MP_ROM_PTR(&mp_lv_page_get_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_hor_fit), MP_ROM_PTR(&mp_lv_page_get_scrl_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_fit_ver), MP_ROM_PTR(&mp_lv_page_get_scrl_fit_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_page_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pr_action), MP_ROM_PTR(&mp_lv_page_get_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rel_action), MP_ROM_PTR(&mp_lv_page_get_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl), MP_ROM_PTR(&mp_lv_page_get_scrl_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rel_action), MP_ROM_PTR(&mp_lv_page_set_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pr_action), MP_ROM_PTR(&mp_lv_page_set_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_page_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arrow_scroll), MP_ROM_PTR(&mp_lv_page_set_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_page_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_page_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arrow_scroll), MP_ROM_PTR(&mp_lv_page_get_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_page_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_page_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_page_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_glue_obj), MP_ROM_PTR(&mp_lv_page_glue_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_page_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_page_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_page_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PAGE_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_DDLIST_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(ddlist_locals_dict, ddlist_locals_dict_table);

STATIC void ddlist_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl ddlist");
}


STATIC mp_obj_t ddlist_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_ddlist_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_ddlist_type = {
    { &mp_type_type },
    .name = MP_QSTR_ddlist,
    .print = ddlist_print,
    .make_new = ddlist_make_new,
    .locals_dict = (mp_obj_dict_t*)&ddlist_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_lmeter_set_style(lv_obj_t *lmeter, lv_style_t *bg)
 */
 
STATIC mp_obj_t mp_lv_lmeter_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *lmeter = mp_to_lv(args[0]);
    lv_style_t *bg = mp_write_ptr_lv_style_t(args[1]);
    lv_lmeter_set_style(lmeter, bg);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_set_style_obj, 2, 2, mp_lv_lmeter_set_style);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_lmeter_get_style(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    lv_style_t* res = lv_lmeter_get_style(lmeter);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_style_obj, 1, 1, mp_lv_lmeter_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_lmeter_set_value(lv_obj_t *lmeter, int16_t value)
 */
 
STATIC mp_obj_t mp_lv_lmeter_set_value(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_lmeter_set_value(lmeter, value);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_set_value_obj, 2, 2, mp_lv_lmeter_set_value);

 

/*
 * lvgl extension definition for:
 * void lv_lmeter_set_range(lv_obj_t *lmeter, int16_t min, int16_t max)
 */
 
STATIC mp_obj_t mp_lv_lmeter_set_range(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t min = (int16_t)mp_obj_int_get_checked(args[1]);
    int16_t max = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_lmeter_set_range(lmeter, min, max);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_set_range_obj, 3, 3, mp_lv_lmeter_set_range);

 

/*
 * lvgl extension definition for:
 * void lv_lmeter_set_scale(lv_obj_t *lmeter, uint16_t angle, uint8_t line_cnt)
 */
 
STATIC mp_obj_t mp_lv_lmeter_set_scale(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *lmeter = mp_to_lv(args[0]);
    uint16_t angle = (uint16_t)mp_obj_int_get_checked(args[1]);
    uint8_t line_cnt = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_lmeter_set_scale(lmeter, angle, line_cnt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_set_scale_obj, 3, 3, mp_lv_lmeter_set_scale);

 

/*
 * lvgl extension definition for:
 * int16_t lv_lmeter_get_value(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t res = lv_lmeter_get_value(lmeter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_value_obj, 1, 1, mp_lv_lmeter_get_value);

 

/*
 * lvgl extension definition for:
 * int16_t lv_lmeter_get_min_value(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_min_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t res = lv_lmeter_get_min_value(lmeter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_min_value_obj, 1, 1, mp_lv_lmeter_get_min_value);

 

/*
 * lvgl extension definition for:
 * int16_t lv_lmeter_get_max_value(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_max_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t res = lv_lmeter_get_max_value(lmeter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_max_value_obj, 1, 1, mp_lv_lmeter_get_max_value);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_lmeter_get_line_count(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_line_count(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    uint8_t res = lv_lmeter_get_line_count(lmeter);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_line_count_obj, 1, 1, mp_lv_lmeter_get_line_count);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_lmeter_get_scale_angle(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_lmeter_get_scale_angle(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    uint16_t res = lv_lmeter_get_scale_angle(lmeter);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_lmeter_get_scale_angle_obj, 1, 1, mp_lv_lmeter_get_scale_angle);

 

    
/*
 * lvgl lmeter object definitions
 */

STATIC const mp_rom_map_elem_t lmeter_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_lmeter_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_lmeter_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_lmeter_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_lmeter_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scale), MP_ROM_PTR(&mp_lv_lmeter_set_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_lmeter_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_lmeter_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_lmeter_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_line_count), MP_ROM_PTR(&mp_lv_lmeter_get_line_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scale_angle), MP_ROM_PTR(&mp_lv_lmeter_get_scale_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(lmeter_locals_dict, lmeter_locals_dict_table);

STATIC void lmeter_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl lmeter");
}


STATIC mp_obj_t lmeter_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_lmeter_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_lmeter_type = {
    { &mp_type_type },
    .name = MP_QSTR_lmeter,
    .print = lmeter_print,
    .make_new = lmeter_make_new,
    .locals_dict = (mp_obj_dict_t*)&lmeter_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_gauge_set_range(lv_obj_t *gauge, int16_t min, int16_t max)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_range(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    int16_t min = (int16_t)mp_obj_int_get_checked(args[1]);
    int16_t max = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_gauge_set_range(gauge, min, max);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_range_obj, 3, 3, mp_lv_gauge_set_range);

 

/*
 * lvgl extension definition for:
 * inline static void lv_gauge_set_critical_value(lv_obj_t *gauge, int16_t value)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_critical_value(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_gauge_set_critical_value(gauge, value);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_critical_value_obj, 2, 2, mp_lv_gauge_set_critical_value);

 

/*
 * lvgl extension definition for:
 * inline static void lv_gauge_set_style(lv_obj_t *gauge, lv_style_t *bg)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    lv_style_t *bg = mp_write_ptr_lv_style_t(args[1]);
    lv_gauge_set_style(gauge, bg);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_style_obj, 2, 2, mp_lv_gauge_set_style);

 

/*
 * lvgl extension definition for:
 * inline static int16_t lv_gauge_get_min_value(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_min_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t res = lv_gauge_get_min_value(lmeter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_min_value_obj, 1, 1, mp_lv_gauge_get_min_value);

 

/*
 * lvgl extension definition for:
 * inline static int16_t lv_gauge_get_max_value(const lv_obj_t *lmeter)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_max_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *lmeter = mp_to_lv(args[0]);
    int16_t res = lv_gauge_get_max_value(lmeter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_max_value_obj, 1, 1, mp_lv_gauge_get_max_value);

 

/*
 * lvgl extension definition for:
 * inline static int16_t lv_gauge_get_critical_value(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_critical_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    int16_t res = lv_gauge_get_critical_value(gauge);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_critical_value_obj, 1, 1, mp_lv_gauge_get_critical_value);

 

/*
 * lvgl extension definition for:
 * inline static uint8_t lv_gauge_get_line_count(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_line_count(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t res = lv_gauge_get_line_count(gauge);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_line_count_obj, 1, 1, mp_lv_gauge_get_line_count);

 

/*
 * lvgl extension definition for:
 * inline static uint16_t lv_gauge_get_scale_angle(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_scale_angle(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    uint16_t res = lv_gauge_get_scale_angle(gauge);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_scale_angle_obj, 1, 1, mp_lv_gauge_get_scale_angle);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_gauge_get_style(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    lv_style_t* res = lv_gauge_get_style(gauge);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_style_obj, 1, 1, mp_lv_gauge_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_gauge_set_needle_count(lv_obj_t *gauge, uint8_t needle_cnt, const lv_color_t *colors)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_needle_count(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t needle_cnt = (uint8_t)mp_obj_int_get_checked(args[1]);
    const lv_color_t *colors = mp_write_ptr_lv_color16_t(args[2]);
    lv_gauge_set_needle_count(gauge, needle_cnt, colors);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_needle_count_obj, 3, 3, mp_lv_gauge_set_needle_count);

 

/*
 * lvgl extension definition for:
 * void lv_gauge_set_value(lv_obj_t *gauge, uint8_t needle_id, int16_t value)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_value(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t needle_id = (uint8_t)mp_obj_int_get_checked(args[1]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_gauge_set_value(gauge, needle_id, value);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_value_obj, 3, 3, mp_lv_gauge_set_value);

 

/*
 * lvgl extension definition for:
 * void lv_gauge_set_scale(lv_obj_t *gauge, uint16_t angle, uint8_t line_cnt, uint8_t label_cnt)
 */
 
STATIC mp_obj_t mp_lv_gauge_set_scale(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *gauge = mp_to_lv(args[0]);
    uint16_t angle = (uint16_t)mp_obj_int_get_checked(args[1]);
    uint8_t line_cnt = (uint8_t)mp_obj_int_get_checked(args[2]);
    uint8_t label_cnt = (uint8_t)mp_obj_int_get_checked(args[3]);
    lv_gauge_set_scale(gauge, angle, line_cnt, label_cnt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_set_scale_obj, 4, 4, mp_lv_gauge_set_scale);

 

/*
 * lvgl extension definition for:
 * int16_t lv_gauge_get_value(const lv_obj_t *gauge, uint8_t needle)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t needle = (uint8_t)mp_obj_int_get_checked(args[1]);
    int16_t res = lv_gauge_get_value(gauge, needle);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_value_obj, 2, 2, mp_lv_gauge_get_value);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_gauge_get_needle_count(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_needle_count(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t res = lv_gauge_get_needle_count(gauge);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_needle_count_obj, 1, 1, mp_lv_gauge_get_needle_count);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_gauge_get_label_count(const lv_obj_t *gauge)
 */
 
STATIC mp_obj_t mp_lv_gauge_get_label_count(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *gauge = mp_to_lv(args[0]);
    uint8_t res = lv_gauge_get_label_count(gauge);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_gauge_get_label_count_obj, 1, 1, mp_lv_gauge_get_label_count);

 

    
/*
 * lvgl gauge object definitions
 */

STATIC const mp_rom_map_elem_t gauge_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_gauge_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_critical_value), MP_ROM_PTR(&mp_lv_gauge_set_critical_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_gauge_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_gauge_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_gauge_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_critical_value), MP_ROM_PTR(&mp_lv_gauge_get_critical_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_line_count), MP_ROM_PTR(&mp_lv_gauge_get_line_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scale_angle), MP_ROM_PTR(&mp_lv_gauge_get_scale_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_gauge_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_needle_count), MP_ROM_PTR(&mp_lv_gauge_set_needle_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_gauge_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scale), MP_ROM_PTR(&mp_lv_gauge_set_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_gauge_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_needle_count), MP_ROM_PTR(&mp_lv_gauge_get_needle_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_label_count), MP_ROM_PTR(&mp_lv_gauge_get_label_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_lmeter_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_lmeter_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_lmeter_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_lmeter_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scale), MP_ROM_PTR(&mp_lv_lmeter_set_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_lmeter_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_lmeter_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_lmeter_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_line_count), MP_ROM_PTR(&mp_lv_lmeter_get_line_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scale_angle), MP_ROM_PTR(&mp_lv_lmeter_get_scale_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(gauge_locals_dict, gauge_locals_dict_table);

STATIC void gauge_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl gauge");
}


STATIC mp_obj_t gauge_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_gauge_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_gauge_type = {
    { &mp_type_type },
    .name = MP_QSTR_gauge,
    .print = gauge_print,
    .make_new = gauge_make_new,
    .locals_dict = (mp_obj_dict_t*)&gauge_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_img_set_file(lv_obj_t *img, const char *fn)
 */
 
STATIC mp_obj_t mp_lv_img_set_file(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    const char *fn = (char*)mp_obj_str_get_str(args[1]);
    lv_img_set_file(img, fn);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_set_file_obj, 2, 2, mp_lv_img_set_file);

 

/*
 * lvgl extension definition for:
 * inline static void lv_img_set_style(lv_obj_t *img, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_img_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_img_set_style(img, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_set_style_obj, 2, 2, mp_lv_img_set_style);

 

/*
 * lvgl extension definition for:
 * inline static void lv_img_set_upscale(lv_obj_t *img, bool upcale)
 */
 
STATIC mp_obj_t mp_lv_img_set_upscale(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    bool upcale = mp_obj_is_true(args[1]);
    lv_img_set_upscale(img, upcale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_set_upscale_obj, 2, 2, mp_lv_img_set_upscale);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_img_get_style(const lv_obj_t *img)
 */
 
STATIC mp_obj_t mp_lv_img_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *img = mp_to_lv(args[0]);
    lv_style_t* res = lv_img_get_style(img);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_get_style_obj, 1, 1, mp_lv_img_get_style);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_img_get_upscale(const lv_obj_t *img)
 */
 
STATIC mp_obj_t mp_lv_img_get_upscale(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *img = mp_to_lv(args[0]);
    bool res = lv_img_get_upscale(img);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_get_upscale_obj, 1, 1, mp_lv_img_get_upscale);

 

/*
 * lvgl extension definition for:
 * lv_img_src_t lv_img_src_get_type(const void *src)
 */
 
STATIC mp_obj_t mp_lv_img_src_get_type(size_t n_args, const mp_obj_t *args)
{
    const void *src = mp_to_ptr(args[0]);
    lv_img_src_t res = lv_img_src_get_type(src);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_src_get_type_obj, 1, 1, mp_lv_img_src_get_type);

 

/*
 * lvgl extension definition for:
 * void lv_img_decoder_set_custom(lv_img_decoder_info_f_t info_fp, lv_img_decoder_open_f_t open_fp, lv_img_decoder_read_line_f_t read_fp, lv_img_decoder_close_f_t close_fp)
 */
 
STATIC mp_obj_t mp_lv_img_decoder_set_custom(size_t n_args, const mp_obj_t *args)
{
    set_action(args[0], args[0]);
    lv_img_decoder_info_f_t info_fp = &lv_img_decoder_info_f_t_callback;
    lv_img_decoder_open_f_t open_fp = mp_to_ptr(args[1]);
    lv_img_decoder_read_line_f_t read_fp = mp_to_ptr(args[2]);
    lv_img_decoder_close_f_t close_fp = mp_to_ptr(args[3]);
    lv_img_decoder_set_custom(info_fp, open_fp, read_fp, close_fp);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_decoder_set_custom_obj, 4, 4, mp_lv_img_decoder_set_custom);

 

/*
 * lvgl extension definition for:
 * lv_res_t lv_img_dsc_get_info(const char *src, lv_img_header_t *header)
 */
 
STATIC mp_obj_t mp_lv_img_dsc_get_info(size_t n_args, const mp_obj_t *args)
{
    const char *src = (char*)mp_obj_str_get_str(args[0]);
    lv_img_header_t *header = mp_write_ptr_lv_img_header_t(args[1]);
    lv_res_t res = lv_img_dsc_get_info(src, header);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_dsc_get_info_obj, 2, 2, mp_lv_img_dsc_get_info);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_img_color_format_get_px_size(lv_img_cf_t cf)
 */
 
STATIC mp_obj_t mp_lv_img_color_format_get_px_size(size_t n_args, const mp_obj_t *args)
{
    lv_img_cf_t cf = (uint8_t)mp_obj_int_get_checked(args[0]);
    uint8_t res = lv_img_color_format_get_px_size(cf);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_color_format_get_px_size_obj, 1, 1, mp_lv_img_color_format_get_px_size);

 

/*
 * lvgl extension definition for:
 * bool lv_img_color_format_is_chroma_keyed(lv_img_cf_t cf)
 */
 
STATIC mp_obj_t mp_lv_img_color_format_is_chroma_keyed(size_t n_args, const mp_obj_t *args)
{
    lv_img_cf_t cf = (uint8_t)mp_obj_int_get_checked(args[0]);
    bool res = lv_img_color_format_is_chroma_keyed(cf);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_color_format_is_chroma_keyed_obj, 1, 1, mp_lv_img_color_format_is_chroma_keyed);

 

/*
 * lvgl extension definition for:
 * bool lv_img_color_format_has_alpha(lv_img_cf_t cf)
 */
 
STATIC mp_obj_t mp_lv_img_color_format_has_alpha(size_t n_args, const mp_obj_t *args)
{
    lv_img_cf_t cf = (uint8_t)mp_obj_int_get_checked(args[0]);
    bool res = lv_img_color_format_has_alpha(cf);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_color_format_has_alpha_obj, 1, 1, mp_lv_img_color_format_has_alpha);

 

/*
 * lvgl extension definition for:
 * void lv_img_set_src(lv_obj_t *img, const void *src_img)
 */
 
STATIC mp_obj_t mp_lv_img_set_src(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    const void *src_img = mp_to_ptr(args[1]);
    lv_img_set_src(img, src_img);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_set_src_obj, 2, 2, mp_lv_img_set_src);

 

/*
 * lvgl extension definition for:
 * void lv_img_set_auto_size(lv_obj_t *img, bool autosize_en)
 */
 
STATIC mp_obj_t mp_lv_img_set_auto_size(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    bool autosize_en = mp_obj_is_true(args[1]);
    lv_img_set_auto_size(img, autosize_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_set_auto_size_obj, 2, 2, mp_lv_img_set_auto_size);

 

/*
 * lvgl extension definition for:
 * const void *lv_img_get_src(lv_obj_t *img)
 */
 
STATIC mp_obj_t mp_lv_img_get_src(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *img = mp_to_lv(args[0]);
    const void* res = lv_img_get_src(img);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_get_src_obj, 1, 1, mp_lv_img_get_src);

 

/*
 * lvgl extension definition for:
 * const char *lv_img_get_file_name(const lv_obj_t *img)
 */
 
STATIC mp_obj_t mp_lv_img_get_file_name(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *img = mp_to_lv(args[0]);
    const char* res = lv_img_get_file_name(img);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_get_file_name_obj, 1, 1, mp_lv_img_get_file_name);

 

/*
 * lvgl extension definition for:
 * bool lv_img_get_auto_size(const lv_obj_t *img)
 */
 
STATIC mp_obj_t mp_lv_img_get_auto_size(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *img = mp_to_lv(args[0]);
    bool res = lv_img_get_auto_size(img);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_img_get_auto_size_obj, 1, 1, mp_lv_img_get_auto_size);

 

    
/*
 * lvgl img object definitions
 */

STATIC const mp_rom_map_elem_t img_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_file), MP_ROM_PTR(&mp_lv_img_set_file_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_img_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_upscale), MP_ROM_PTR(&mp_lv_img_set_upscale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_img_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_upscale), MP_ROM_PTR(&mp_lv_img_get_upscale_obj) },
    { MP_ROM_QSTR(MP_QSTR_src_get_type), MP_ROM_PTR(&mp_lv_img_src_get_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_decoder_set_custom), MP_ROM_PTR(&mp_lv_img_decoder_set_custom_obj) },
    { MP_ROM_QSTR(MP_QSTR_dsc_get_info), MP_ROM_PTR(&mp_lv_img_dsc_get_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_format_get_px_size), MP_ROM_PTR(&mp_lv_img_color_format_get_px_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_format_is_chroma_keyed), MP_ROM_PTR(&mp_lv_img_color_format_is_chroma_keyed_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_format_has_alpha), MP_ROM_PTR(&mp_lv_img_color_format_has_alpha_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_src), MP_ROM_PTR(&mp_lv_img_set_src_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_auto_size), MP_ROM_PTR(&mp_lv_img_set_auto_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_src), MP_ROM_PTR(&mp_lv_img_get_src_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_file_name), MP_ROM_PTR(&mp_lv_img_get_file_name_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_auto_size), MP_ROM_PTR(&mp_lv_img_get_auto_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_CF), MP_ROM_PTR(&mp_LV_IMG_CF_type) },
    { MP_ROM_QSTR(MP_QSTR_SRC), MP_ROM_PTR(&mp_LV_IMG_SRC_type) }
};

STATIC MP_DEFINE_CONST_DICT(img_locals_dict, img_locals_dict_table);

STATIC void img_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl img");
}


STATIC mp_obj_t img_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_img_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_img_type = {
    { &mp_type_type },
    .name = MP_QSTR_img,
    .print = img_print,
    .make_new = img_make_new,
    .locals_dict = (mp_obj_dict_t*)&img_locals_dict,
};
    

/*
 * Function NOT generated:
 * Missing conversion to char**
 * inline static void lv_kb_set_map(lv_obj_t *kb, const char **map)
 */
    

/*
 * lvgl extension definition for:
 * void lv_kb_set_ta(lv_obj_t *kb, lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_kb_set_ta(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    lv_obj_t *ta = mp_to_lv(args[1]);
    lv_kb_set_ta(kb, ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_ta_obj, 2, 2, mp_lv_kb_set_ta);

 

/*
 * lvgl extension definition for:
 * void lv_kb_set_mode(lv_obj_t *kb, lv_kb_mode_t mode)
 */
 
STATIC mp_obj_t mp_lv_kb_set_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    lv_kb_mode_t mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_kb_set_mode(kb, mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_mode_obj, 2, 2, mp_lv_kb_set_mode);

 

/*
 * lvgl extension definition for:
 * void lv_kb_set_cursor_manage(lv_obj_t *kb, bool en)
 */
 
STATIC mp_obj_t mp_lv_kb_set_cursor_manage(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_kb_set_cursor_manage(kb, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_cursor_manage_obj, 2, 2, mp_lv_kb_set_cursor_manage);

 

/*
 * lvgl extension definition for:
 * void lv_kb_set_ok_action(lv_obj_t *kb, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_kb_set_ok_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_kb_set_ok_action(kb, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_ok_action_obj, 2, 2, mp_lv_kb_set_ok_action);

 

/*
 * lvgl extension definition for:
 * void lv_kb_set_hide_action(lv_obj_t *kb, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_kb_set_hide_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_kb_set_hide_action(kb, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_hide_action_obj, 2, 2, mp_lv_kb_set_hide_action);

 

/*
 * lvgl extension definition for:
 * void lv_kb_set_style(lv_obj_t *kb, lv_kb_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_kb_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *kb = mp_to_lv(args[0]);
    lv_kb_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_kb_set_style(kb, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_set_style_obj, 3, 3, mp_lv_kb_set_style);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_kb_get_ta(const lv_obj_t *kb)
 */
 
STATIC mp_obj_t mp_lv_kb_get_ta(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    lv_obj_t* res = lv_kb_get_ta(kb);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_ta_obj, 1, 1, mp_lv_kb_get_ta);

 

/*
 * lvgl extension definition for:
 * lv_kb_mode_t lv_kb_get_mode(const lv_obj_t *kb)
 */
 
STATIC mp_obj_t mp_lv_kb_get_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    lv_kb_mode_t res = lv_kb_get_mode(kb);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_mode_obj, 1, 1, mp_lv_kb_get_mode);

 

/*
 * lvgl extension definition for:
 * bool lv_kb_get_cursor_manage(const lv_obj_t *kb)
 */
 
STATIC mp_obj_t mp_lv_kb_get_cursor_manage(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    bool res = lv_kb_get_cursor_manage(kb);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_cursor_manage_obj, 1, 1, mp_lv_kb_get_cursor_manage);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_kb_get_ok_action(const lv_obj_t *kb)
 */
 
STATIC mp_obj_t mp_lv_kb_get_ok_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    lv_action_t res = lv_kb_get_ok_action(kb);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_ok_action_obj, 1, 1, mp_lv_kb_get_ok_action);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_kb_get_hide_action(const lv_obj_t *kb)
 */
 
STATIC mp_obj_t mp_lv_kb_get_hide_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    lv_action_t res = lv_kb_get_hide_action(kb);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_hide_action_obj, 1, 1, mp_lv_kb_get_hide_action);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_kb_get_style(const lv_obj_t *kb, lv_kb_style_t type)
 */
 
STATIC mp_obj_t mp_lv_kb_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *kb = mp_to_lv(args[0]);
    lv_kb_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_kb_get_style(kb, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_kb_get_style_obj, 2, 2, mp_lv_kb_get_style);

 

    
/*
 * lvgl kb object definitions
 */

STATIC const mp_rom_map_elem_t kb_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_ta), MP_ROM_PTR(&mp_lv_kb_set_ta_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_mode), MP_ROM_PTR(&mp_lv_kb_set_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cursor_manage), MP_ROM_PTR(&mp_lv_kb_set_cursor_manage_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_ok_action), MP_ROM_PTR(&mp_lv_kb_set_ok_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hide_action), MP_ROM_PTR(&mp_lv_kb_set_hide_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_kb_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ta), MP_ROM_PTR(&mp_lv_kb_get_ta_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mode), MP_ROM_PTR(&mp_lv_kb_get_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cursor_manage), MP_ROM_PTR(&mp_lv_kb_get_cursor_manage_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ok_action), MP_ROM_PTR(&mp_lv_kb_get_ok_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hide_action), MP_ROM_PTR(&mp_lv_kb_get_hide_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_kb_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_btnm_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_toggle), MP_ROM_PTR(&mp_lv_btnm_set_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_btnm_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_btnm_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_toggled), MP_ROM_PTR(&mp_lv_btnm_get_toggled_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_btnm_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BTNM_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_MODE), MP_ROM_PTR(&mp_LV_KB_MODE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_KB_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(kb_locals_dict, kb_locals_dict_table);

STATIC void kb_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl kb");
}


STATIC mp_obj_t kb_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_kb_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_kb_type = {
    { &mp_type_type },
    .name = MP_QSTR_kb,
    .print = kb_print,
    .make_new = kb_make_new,
    .locals_dict = (mp_obj_dict_t*)&kb_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_led_set_style(lv_obj_t *led, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_led_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *led = mp_to_lv(args[0]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[1]);
    lv_led_set_style(led, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_set_style_obj, 2, 2, mp_lv_led_set_style);

 

/*
 * lvgl extension definition for:
 * inline static lv_style_t *lv_led_get_style(const lv_obj_t *led)
 */
 
STATIC mp_obj_t mp_lv_led_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *led = mp_to_lv(args[0]);
    lv_style_t* res = lv_led_get_style(led);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_get_style_obj, 1, 1, mp_lv_led_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_led_set_bright(lv_obj_t *led, uint8_t bright)
 */
 
STATIC mp_obj_t mp_lv_led_set_bright(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *led = mp_to_lv(args[0]);
    uint8_t bright = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_led_set_bright(led, bright);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_set_bright_obj, 2, 2, mp_lv_led_set_bright);

 

/*
 * lvgl extension definition for:
 * void lv_led_on(lv_obj_t *led)
 */
 
STATIC mp_obj_t mp_lv_led_on(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *led = mp_to_lv(args[0]);
    lv_led_on(led);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_on_obj, 1, 1, mp_lv_led_on);

 

/*
 * lvgl extension definition for:
 * void lv_led_off(lv_obj_t *led)
 */
 
STATIC mp_obj_t mp_lv_led_off(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *led = mp_to_lv(args[0]);
    lv_led_off(led);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_off_obj, 1, 1, mp_lv_led_off);

 

/*
 * lvgl extension definition for:
 * void lv_led_toggle(lv_obj_t *led)
 */
 
STATIC mp_obj_t mp_lv_led_toggle(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *led = mp_to_lv(args[0]);
    lv_led_toggle(led);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_toggle_obj, 1, 1, mp_lv_led_toggle);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_led_get_bright(const lv_obj_t *led)
 */
 
STATIC mp_obj_t mp_lv_led_get_bright(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *led = mp_to_lv(args[0]);
    uint8_t res = lv_led_get_bright(led);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_led_get_bright_obj, 1, 1, mp_lv_led_get_bright);

 

    
/*
 * lvgl led object definitions
 */

STATIC const mp_rom_map_elem_t led_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_led_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_led_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_bright), MP_ROM_PTR(&mp_lv_led_set_bright_obj) },
    { MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&mp_lv_led_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_off), MP_ROM_PTR(&mp_lv_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_toggle), MP_ROM_PTR(&mp_lv_led_toggle_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_bright), MP_ROM_PTR(&mp_lv_led_get_bright_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) }
};

STATIC MP_DEFINE_CONST_DICT(led_locals_dict, led_locals_dict_table);

STATIC void led_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl led");
}


STATIC mp_obj_t led_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_led_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_led_type = {
    { &mp_type_type },
    .name = MP_QSTR_led,
    .print = led_print,
    .make_new = led_make_new,
    .locals_dict = (mp_obj_dict_t*)&led_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_list_set_sb_mode(lv_obj_t *list, lv_sb_mode_t mode)
 */
 
STATIC mp_obj_t mp_lv_list_set_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *list = mp_to_lv(args[0]);
    lv_sb_mode_t mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_list_set_sb_mode(list, mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_set_sb_mode_obj, 2, 2, mp_lv_list_set_sb_mode);

 

/*
 * lvgl extension definition for:
 * inline static lv_sb_mode_t lv_list_get_sb_mode(const lv_obj_t *list)
 */
 
STATIC mp_obj_t mp_lv_list_get_sb_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_sb_mode_t res = lv_list_get_sb_mode(list);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_sb_mode_obj, 1, 1, mp_lv_list_get_sb_mode);

 

/*
 * lvgl extension definition for:
 * void lv_list_clean(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_list_clean(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_list_clean(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_clean_obj, 1, 1, mp_lv_list_clean);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_add(lv_obj_t *list, const void *img_src, const char *txt, lv_action_t rel_action)
 */
 
STATIC mp_obj_t mp_lv_list_add(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *list = mp_to_lv(args[0]);
    const void *img_src = mp_to_ptr(args[1]);
    const char *txt = (char*)mp_obj_str_get_str(args[2]);
    set_action(args[0], args[3]);
    lv_action_t rel_action = &lv_action_t_callback;
    lv_obj_t* res = lv_list_add(list, img_src, txt, rel_action);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_add_obj, 4, 4, mp_lv_list_add);

 

/*
 * lvgl extension definition for:
 * void lv_list_set_btn_selected(lv_obj_t *list, lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_list_set_btn_selected(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *list = mp_to_lv(args[0]);
    lv_obj_t *btn = mp_to_lv(args[1]);
    lv_list_set_btn_selected(list, btn);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_set_btn_selected_obj, 2, 2, mp_lv_list_set_btn_selected);

 

/*
 * lvgl extension definition for:
 * void lv_list_set_anim_time(lv_obj_t *list, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_list_set_anim_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *list = mp_to_lv(args[0]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_list_set_anim_time(list, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_set_anim_time_obj, 2, 2, mp_lv_list_set_anim_time);

 

/*
 * lvgl extension definition for:
 * void lv_list_set_style(lv_obj_t *list, lv_list_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_list_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *list = mp_to_lv(args[0]);
    lv_list_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_list_set_style(list, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_set_style_obj, 3, 3, mp_lv_list_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_list_get_btn_text(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_list_get_btn_text(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    const char* res = lv_list_get_btn_text(btn);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_btn_text_obj, 1, 1, mp_lv_list_get_btn_text);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_get_btn_label(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_list_get_btn_label(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_obj_t* res = lv_list_get_btn_label(btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_btn_label_obj, 1, 1, mp_lv_list_get_btn_label);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_get_btn_img(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_list_get_btn_img(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_obj_t* res = lv_list_get_btn_img(btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_btn_img_obj, 1, 1, mp_lv_list_get_btn_img);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_get_prev_btn(const lv_obj_t *list, lv_obj_t *prev_btn)
 */
 
STATIC mp_obj_t mp_lv_list_get_prev_btn(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_obj_t *prev_btn = mp_to_lv(args[1]);
    lv_obj_t* res = lv_list_get_prev_btn(list, prev_btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_prev_btn_obj, 2, 2, mp_lv_list_get_prev_btn);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_get_next_btn(const lv_obj_t *list, lv_obj_t *prev_btn)
 */
 
STATIC mp_obj_t mp_lv_list_get_next_btn(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_obj_t *prev_btn = mp_to_lv(args[1]);
    lv_obj_t* res = lv_list_get_next_btn(list, prev_btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_next_btn_obj, 2, 2, mp_lv_list_get_next_btn);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_list_get_btn_selected(const lv_obj_t *list)
 */
 
STATIC mp_obj_t mp_lv_list_get_btn_selected(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_obj_t* res = lv_list_get_btn_selected(list);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_btn_selected_obj, 1, 1, mp_lv_list_get_btn_selected);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_list_get_anim_time(const lv_obj_t *list)
 */
 
STATIC mp_obj_t mp_lv_list_get_anim_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    uint16_t res = lv_list_get_anim_time(list);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_anim_time_obj, 1, 1, mp_lv_list_get_anim_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_list_get_style(const lv_obj_t *list, lv_list_style_t type)
 */
 
STATIC mp_obj_t mp_lv_list_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_list_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_list_get_style(list, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_get_style_obj, 2, 2, mp_lv_list_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_list_up(const lv_obj_t *list)
 */
 
STATIC mp_obj_t mp_lv_list_up(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_list_up(list);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_up_obj, 1, 1, mp_lv_list_up);

 

/*
 * lvgl extension definition for:
 * void lv_list_down(const lv_obj_t *list)
 */
 
STATIC mp_obj_t mp_lv_list_down(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *list = mp_to_lv(args[0]);
    lv_list_down(list);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_down_obj, 1, 1, mp_lv_list_down);

 

/*
 * lvgl extension definition for:
 * void lv_list_focus(const lv_obj_t *btn, bool anim_en)
 */
 
STATIC mp_obj_t mp_lv_list_focus(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    bool anim_en = mp_obj_is_true(args[1]);
    lv_list_focus(btn, anim_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_list_focus_obj, 2, 2, mp_lv_list_focus);

 

    
/*
 * lvgl list object definitions
 */

STATIC const mp_rom_map_elem_t list_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_list_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_list_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_list_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_add), MP_ROM_PTR(&mp_lv_list_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_btn_selected), MP_ROM_PTR(&mp_lv_list_set_btn_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_list_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_list_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btn_text), MP_ROM_PTR(&mp_lv_list_get_btn_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btn_label), MP_ROM_PTR(&mp_lv_list_get_btn_label_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btn_img), MP_ROM_PTR(&mp_lv_list_get_btn_img_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_prev_btn), MP_ROM_PTR(&mp_lv_list_get_prev_btn_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_next_btn), MP_ROM_PTR(&mp_lv_list_get_next_btn_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btn_selected), MP_ROM_PTR(&mp_lv_list_get_btn_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_list_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_list_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_up), MP_ROM_PTR(&mp_lv_list_up_obj) },
    { MP_ROM_QSTR(MP_QSTR_down), MP_ROM_PTR(&mp_lv_list_down_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_list_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_fit), MP_ROM_PTR(&mp_lv_page_set_scrl_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_width), MP_ROM_PTR(&mp_lv_page_set_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_height), MP_ROM_PTR(&mp_lv_page_set_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_layout), MP_ROM_PTR(&mp_lv_page_set_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_width), MP_ROM_PTR(&mp_lv_page_get_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_height), MP_ROM_PTR(&mp_lv_page_get_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_layout), MP_ROM_PTR(&mp_lv_page_get_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_hor_fit), MP_ROM_PTR(&mp_lv_page_get_scrl_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_fit_ver), MP_ROM_PTR(&mp_lv_page_get_scrl_fit_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_page_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pr_action), MP_ROM_PTR(&mp_lv_page_get_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rel_action), MP_ROM_PTR(&mp_lv_page_get_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl), MP_ROM_PTR(&mp_lv_page_get_scrl_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rel_action), MP_ROM_PTR(&mp_lv_page_set_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pr_action), MP_ROM_PTR(&mp_lv_page_set_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_page_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arrow_scroll), MP_ROM_PTR(&mp_lv_page_set_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_page_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_page_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arrow_scroll), MP_ROM_PTR(&mp_lv_page_get_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_page_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_page_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_page_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_glue_obj), MP_ROM_PTR(&mp_lv_page_glue_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_page_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_page_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_page_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PAGE_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_LIST_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(list_locals_dict, list_locals_dict_table);

STATIC void list_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl list");
}


STATIC mp_obj_t list_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_list_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_list_type = {
    { &mp_type_type },
    .name = MP_QSTR_list,
    .print = list_print,
    .make_new = list_make_new,
    .locals_dict = (mp_obj_dict_t*)&list_locals_dict,
};
    

/*
 * Function NOT generated:
 * Missing conversion to char**
 * void lv_mbox_add_btns(lv_obj_t *mbox, const char **btn_map, lv_btnm_action_t action)
 */
    

/*
 * lvgl extension definition for:
 * void lv_mbox_set_text(lv_obj_t *mbox, const char *txt)
 */
 
STATIC mp_obj_t mp_lv_mbox_set_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    const char *txt = (char*)mp_obj_str_get_str(args[1]);
    lv_mbox_set_text(mbox, txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_set_text_obj, 2, 2, mp_lv_mbox_set_text);

 

/*
 * lvgl extension definition for:
 * void lv_mbox_set_action(lv_obj_t *mbox, lv_btnm_action_t action)
 */
 
STATIC mp_obj_t mp_lv_mbox_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_btnm_action_t action = &lv_btnm_action_t_callback;
    lv_mbox_set_action(mbox, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_set_action_obj, 2, 2, mp_lv_mbox_set_action);

 

/*
 * lvgl extension definition for:
 * void lv_mbox_set_anim_time(lv_obj_t *mbox, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_mbox_set_anim_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_mbox_set_anim_time(mbox, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_set_anim_time_obj, 2, 2, mp_lv_mbox_set_anim_time);

 

/*
 * lvgl extension definition for:
 * void lv_mbox_start_auto_close(lv_obj_t *mbox, uint16_t delay)
 */
 
STATIC mp_obj_t mp_lv_mbox_start_auto_close(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    uint16_t delay = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_mbox_start_auto_close(mbox, delay);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_start_auto_close_obj, 2, 2, mp_lv_mbox_start_auto_close);

 

/*
 * lvgl extension definition for:
 * void lv_mbox_stop_auto_close(lv_obj_t *mbox)
 */
 
STATIC mp_obj_t mp_lv_mbox_stop_auto_close(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    lv_mbox_stop_auto_close(mbox);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_stop_auto_close_obj, 1, 1, mp_lv_mbox_stop_auto_close);

 

/*
 * lvgl extension definition for:
 * void lv_mbox_set_style(lv_obj_t *mbox, lv_mbox_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_mbox_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *mbox = mp_to_lv(args[0]);
    lv_mbox_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_mbox_set_style(mbox, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_set_style_obj, 3, 3, mp_lv_mbox_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_mbox_get_text(const lv_obj_t *mbox)
 */
 
STATIC mp_obj_t mp_lv_mbox_get_text(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *mbox = mp_to_lv(args[0]);
    const char* res = lv_mbox_get_text(mbox);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_get_text_obj, 1, 1, mp_lv_mbox_get_text);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_mbox_get_from_btn(const lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_mbox_get_from_btn(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *btn = mp_to_lv(args[0]);
    lv_obj_t* res = lv_mbox_get_from_btn(btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_get_from_btn_obj, 1, 1, mp_lv_mbox_get_from_btn);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_mbox_get_anim_time(const lv_obj_t *mbox)
 */
 
STATIC mp_obj_t mp_lv_mbox_get_anim_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *mbox = mp_to_lv(args[0]);
    uint16_t res = lv_mbox_get_anim_time(mbox);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_get_anim_time_obj, 1, 1, mp_lv_mbox_get_anim_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_mbox_get_style(const lv_obj_t *mbox, lv_mbox_style_t type)
 */
 
STATIC mp_obj_t mp_lv_mbox_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *mbox = mp_to_lv(args[0]);
    lv_mbox_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_mbox_get_style(mbox, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mbox_get_style_obj, 2, 2, mp_lv_mbox_get_style);

 

    
/*
 * lvgl mbox object definitions
 */

STATIC const mp_rom_map_elem_t mbox_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_text), MP_ROM_PTR(&mp_lv_mbox_set_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_mbox_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_mbox_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_start_auto_close), MP_ROM_PTR(&mp_lv_mbox_start_auto_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop_auto_close), MP_ROM_PTR(&mp_lv_mbox_stop_auto_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_mbox_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_text), MP_ROM_PTR(&mp_lv_mbox_get_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_from_btn), MP_ROM_PTR(&mp_lv_mbox_get_from_btn_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_mbox_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_mbox_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_MBOX_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(mbox_locals_dict, mbox_locals_dict_table);

STATIC void mbox_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl mbox");
}


STATIC mp_obj_t mbox_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_mbox_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_mbox_type = {
    { &mp_type_type },
    .name = MP_QSTR_mbox,
    .print = mbox_print,
    .make_new = mbox_make_new,
    .locals_dict = (mp_obj_dict_t*)&mbox_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * void lv_preload_set_arc_length(lv_obj_t *preload, uint16_t deg)
 */
 
STATIC mp_obj_t mp_lv_preload_set_arc_length(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *preload = mp_to_lv(args[0]);
    uint16_t deg = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_preload_set_arc_length(preload, deg);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_set_arc_length_obj, 2, 2, mp_lv_preload_set_arc_length);

 

/*
 * lvgl extension definition for:
 * void lv_preload_set_spin_time(lv_obj_t *preload, uint16_t time)
 */
 
STATIC mp_obj_t mp_lv_preload_set_spin_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *preload = mp_to_lv(args[0]);
    uint16_t time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_preload_set_spin_time(preload, time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_set_spin_time_obj, 2, 2, mp_lv_preload_set_spin_time);

 

/*
 * lvgl extension definition for:
 * void lv_preload_set_style(lv_obj_t *preload, lv_preload_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_preload_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *preload = mp_to_lv(args[0]);
    lv_preload_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_preload_set_style(preload, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_set_style_obj, 3, 3, mp_lv_preload_set_style);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_preload_get_arc_length(const lv_obj_t *preload)
 */
 
STATIC mp_obj_t mp_lv_preload_get_arc_length(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *preload = mp_to_lv(args[0]);
    uint16_t res = lv_preload_get_arc_length(preload);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_get_arc_length_obj, 1, 1, mp_lv_preload_get_arc_length);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_preload_get_spin_time(const lv_obj_t *preload)
 */
 
STATIC mp_obj_t mp_lv_preload_get_spin_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *preload = mp_to_lv(args[0]);
    uint16_t res = lv_preload_get_spin_time(preload);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_get_spin_time_obj, 1, 1, mp_lv_preload_get_spin_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_preload_get_style(const lv_obj_t *preload, lv_preload_style_t type)
 */
 
STATIC mp_obj_t mp_lv_preload_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *preload = mp_to_lv(args[0]);
    lv_preload_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_preload_get_style(preload, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_get_style_obj, 2, 2, mp_lv_preload_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_preload_spinner_animation(void *ptr, int32_t val)
 */
 
STATIC mp_obj_t mp_lv_preload_spinner_animation(size_t n_args, const mp_obj_t *args)
{
    void *ptr = mp_to_ptr(args[0]);
    int32_t val = (int32_t)mp_obj_int_get_checked(args[1]);
    lv_preload_spinner_animation(ptr, val);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_preload_spinner_animation_obj, 2, 2, mp_lv_preload_spinner_animation);

 

    
/*
 * lvgl preload object definitions
 */

STATIC const mp_rom_map_elem_t preload_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_arc_length), MP_ROM_PTR(&mp_lv_preload_set_arc_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_spin_time), MP_ROM_PTR(&mp_lv_preload_set_spin_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_preload_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arc_length), MP_ROM_PTR(&mp_lv_preload_get_arc_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_spin_time), MP_ROM_PTR(&mp_lv_preload_get_spin_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_preload_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_spinner_animation), MP_ROM_PTR(&mp_lv_preload_spinner_animation_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_angles), MP_ROM_PTR(&mp_lv_arc_set_angles_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_arc_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle_start), MP_ROM_PTR(&mp_lv_arc_get_angle_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle_end), MP_ROM_PTR(&mp_lv_arc_get_angle_end_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_arc_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_ARC_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_TYPE_SPINNING), MP_ROM_PTR(&mp_LV_PRELOAD_TYPE_SPINNING_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PRELOAD_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(preload_locals_dict, preload_locals_dict_table);

STATIC void preload_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl preload");
}


STATIC mp_obj_t preload_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_preload_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_preload_type = {
    { &mp_type_type },
    .name = MP_QSTR_preload,
    .print = preload_print,
    .make_new = preload_make_new,
    .locals_dict = (mp_obj_dict_t*)&preload_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_roller_set_options(lv_obj_t *roller, const char *options)
 */
 
STATIC mp_obj_t mp_lv_roller_set_options(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    const char *options = (char*)mp_obj_str_get_str(args[1]);
    lv_roller_set_options(roller, options);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_options_obj, 2, 2, mp_lv_roller_set_options);

 

/*
 * lvgl extension definition for:
 * inline static void lv_roller_set_action(lv_obj_t *roller, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_roller_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_roller_set_action(roller, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_action_obj, 2, 2, mp_lv_roller_set_action);

 

/*
 * lvgl extension definition for:
 * inline static void lv_roller_set_hor_fit(lv_obj_t *roller, bool fit_en)
 */
 
STATIC mp_obj_t mp_lv_roller_set_hor_fit(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    bool fit_en = mp_obj_is_true(args[1]);
    lv_roller_set_hor_fit(roller, fit_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_hor_fit_obj, 2, 2, mp_lv_roller_set_hor_fit);

 

/*
 * lvgl extension definition for:
 * inline static void lv_roller_set_anim_time(lv_obj_t *roller, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_roller_set_anim_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_roller_set_anim_time(roller, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_anim_time_obj, 2, 2, mp_lv_roller_set_anim_time);

 

/*
 * lvgl extension definition for:
 * inline static const char *lv_roller_get_options(const lv_obj_t *roller)
 */
 
STATIC mp_obj_t mp_lv_roller_get_options(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    const char* res = lv_roller_get_options(roller);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_options_obj, 1, 1, mp_lv_roller_get_options);

 

/*
 * lvgl extension definition for:
 * inline static uint16_t lv_roller_get_selected(const lv_obj_t *roller)
 */
 
STATIC mp_obj_t mp_lv_roller_get_selected(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    uint16_t res = lv_roller_get_selected(roller);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_selected_obj, 1, 1, mp_lv_roller_get_selected);

 

/*
 * lvgl extension definition for:
 * inline static void lv_roller_get_selected_str(const lv_obj_t *roller, char *buf)
 */
 
STATIC mp_obj_t mp_lv_roller_get_selected_str(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    char *buf = (char*)mp_obj_str_get_str(args[1]);
    lv_roller_get_selected_str(roller, buf);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_selected_str_obj, 2, 2, mp_lv_roller_get_selected_str);

 

/*
 * lvgl extension definition for:
 * inline static lv_action_t lv_roller_get_action(const lv_obj_t *roller)
 */
 
STATIC mp_obj_t mp_lv_roller_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    lv_action_t res = lv_roller_get_action(roller);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_action_obj, 1, 1, mp_lv_roller_get_action);

 

/*
 * lvgl extension definition for:
 * inline static uint16_t lv_roller_get_anim_time(const lv_obj_t *roller)
 */
 
STATIC mp_obj_t mp_lv_roller_get_anim_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    uint16_t res = lv_roller_get_anim_time(roller);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_anim_time_obj, 1, 1, mp_lv_roller_get_anim_time);

 

/*
 * lvgl extension definition for:
 * void lv_roller_set_selected(lv_obj_t *roller, uint16_t sel_opt, bool anim_en)
 */
 
STATIC mp_obj_t mp_lv_roller_set_selected(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    uint16_t sel_opt = (uint16_t)mp_obj_int_get_checked(args[1]);
    bool anim_en = mp_obj_is_true(args[2]);
    lv_roller_set_selected(roller, sel_opt, anim_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_selected_obj, 3, 3, mp_lv_roller_set_selected);

 

/*
 * lvgl extension definition for:
 * void lv_roller_set_visible_row_count(lv_obj_t *roller, uint8_t row_cnt)
 */
 
STATIC mp_obj_t mp_lv_roller_set_visible_row_count(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    uint8_t row_cnt = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_roller_set_visible_row_count(roller, row_cnt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_visible_row_count_obj, 2, 2, mp_lv_roller_set_visible_row_count);

 

/*
 * lvgl extension definition for:
 * void lv_roller_set_style(lv_obj_t *roller, lv_roller_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_roller_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *roller = mp_to_lv(args[0]);
    lv_roller_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_roller_set_style(roller, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_set_style_obj, 3, 3, mp_lv_roller_set_style);

 

/*
 * lvgl extension definition for:
 * bool lv_roller_get_hor_fit(const lv_obj_t *roller)
 */
 
STATIC mp_obj_t mp_lv_roller_get_hor_fit(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    bool res = lv_roller_get_hor_fit(roller);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_hor_fit_obj, 1, 1, mp_lv_roller_get_hor_fit);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_roller_get_style(const lv_obj_t *roller, lv_roller_style_t type)
 */
 
STATIC mp_obj_t mp_lv_roller_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *roller = mp_to_lv(args[0]);
    lv_roller_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_roller_get_style(roller, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_roller_get_style_obj, 2, 2, mp_lv_roller_get_style);

 

    
/*
 * lvgl roller object definitions
 */

STATIC const mp_rom_map_elem_t roller_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_options), MP_ROM_PTR(&mp_lv_roller_set_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_roller_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hor_fit), MP_ROM_PTR(&mp_lv_roller_set_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_roller_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_options), MP_ROM_PTR(&mp_lv_roller_get_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected), MP_ROM_PTR(&mp_lv_roller_get_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected_str), MP_ROM_PTR(&mp_lv_roller_get_selected_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_roller_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_roller_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_selected), MP_ROM_PTR(&mp_lv_roller_set_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_visible_row_count), MP_ROM_PTR(&mp_lv_roller_set_visible_row_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_roller_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_roller_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_roller_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_ddlist_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_ddlist_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_options), MP_ROM_PTR(&mp_lv_ddlist_set_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_selected), MP_ROM_PTR(&mp_lv_ddlist_set_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_ddlist_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fix_height), MP_ROM_PTR(&mp_lv_ddlist_set_fix_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hor_fit), MP_ROM_PTR(&mp_lv_ddlist_set_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_ddlist_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_ddlist_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_options), MP_ROM_PTR(&mp_lv_ddlist_get_options_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected), MP_ROM_PTR(&mp_lv_ddlist_get_selected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_selected_str), MP_ROM_PTR(&mp_lv_ddlist_get_selected_str_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_ddlist_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fix_height), MP_ROM_PTR(&mp_lv_ddlist_get_fix_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_ddlist_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_ddlist_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_open), MP_ROM_PTR(&mp_lv_ddlist_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_close), MP_ROM_PTR(&mp_lv_ddlist_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_fit), MP_ROM_PTR(&mp_lv_page_set_scrl_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_width), MP_ROM_PTR(&mp_lv_page_set_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_height), MP_ROM_PTR(&mp_lv_page_set_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_layout), MP_ROM_PTR(&mp_lv_page_set_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_width), MP_ROM_PTR(&mp_lv_page_get_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_height), MP_ROM_PTR(&mp_lv_page_get_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_layout), MP_ROM_PTR(&mp_lv_page_get_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_hor_fit), MP_ROM_PTR(&mp_lv_page_get_scrl_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_fit_ver), MP_ROM_PTR(&mp_lv_page_get_scrl_fit_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_page_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pr_action), MP_ROM_PTR(&mp_lv_page_get_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rel_action), MP_ROM_PTR(&mp_lv_page_get_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl), MP_ROM_PTR(&mp_lv_page_get_scrl_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rel_action), MP_ROM_PTR(&mp_lv_page_set_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pr_action), MP_ROM_PTR(&mp_lv_page_set_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_page_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arrow_scroll), MP_ROM_PTR(&mp_lv_page_set_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_page_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_page_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arrow_scroll), MP_ROM_PTR(&mp_lv_page_get_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_page_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_page_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_page_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_glue_obj), MP_ROM_PTR(&mp_lv_page_glue_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_page_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_page_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_page_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PAGE_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_DDLIST_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_ROLLER_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(roller_locals_dict, roller_locals_dict_table);

STATIC void roller_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl roller");
}


STATIC mp_obj_t roller_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_roller_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_roller_type = {
    { &mp_type_type },
    .name = MP_QSTR_roller,
    .print = roller_print,
    .make_new = roller_make_new,
    .locals_dict = (mp_obj_dict_t*)&roller_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_slider_set_value(lv_obj_t *slider, int16_t value)
 */
 
STATIC mp_obj_t mp_lv_slider_set_value(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_slider_set_value(slider, value);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_value_obj, 2, 2, mp_lv_slider_set_value);

 

/*
 * lvgl extension definition for:
 * inline static void lv_slider_set_value_anim(lv_obj_t *slider, int16_t value, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_slider_set_value_anim(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t value = (int16_t)mp_obj_int_get_checked(args[1]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_slider_set_value_anim(slider, value, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_value_anim_obj, 3, 3, mp_lv_slider_set_value_anim);

 

/*
 * lvgl extension definition for:
 * inline static void lv_slider_set_range(lv_obj_t *slider, int16_t min, int16_t max)
 */
 
STATIC mp_obj_t mp_lv_slider_set_range(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t min = (int16_t)mp_obj_int_get_checked(args[1]);
    int16_t max = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_slider_set_range(slider, min, max);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_range_obj, 3, 3, mp_lv_slider_set_range);

 

/*
 * lvgl extension definition for:
 * inline static int16_t lv_slider_get_min_value(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_get_min_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t res = lv_slider_get_min_value(slider);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_min_value_obj, 1, 1, mp_lv_slider_get_min_value);

 

/*
 * lvgl extension definition for:
 * inline static int16_t lv_slider_get_max_value(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_get_max_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t res = lv_slider_get_max_value(slider);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_max_value_obj, 1, 1, mp_lv_slider_get_max_value);

 

/*
 * lvgl extension definition for:
 * void lv_slider_set_action(lv_obj_t *slider, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_slider_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_slider_set_action(slider, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_action_obj, 2, 2, mp_lv_slider_set_action);

 

/*
 * lvgl extension definition for:
 * void lv_slider_set_knob_in(lv_obj_t *slider, bool in)
 */
 
STATIC mp_obj_t mp_lv_slider_set_knob_in(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    bool in = mp_obj_is_true(args[1]);
    lv_slider_set_knob_in(slider, in);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_knob_in_obj, 2, 2, mp_lv_slider_set_knob_in);

 

/*
 * lvgl extension definition for:
 * void lv_slider_set_style(lv_obj_t *slider, lv_slider_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_slider_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *slider = mp_to_lv(args[0]);
    lv_slider_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_slider_set_style(slider, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_set_style_obj, 3, 3, mp_lv_slider_set_style);

 

/*
 * lvgl extension definition for:
 * int16_t lv_slider_get_value(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_get_value(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    int16_t res = lv_slider_get_value(slider);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_value_obj, 1, 1, mp_lv_slider_get_value);

 

/*
 * lvgl extension definition for:
 * lv_action_t lv_slider_get_action(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    lv_action_t res = lv_slider_get_action(slider);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_action_obj, 1, 1, mp_lv_slider_get_action);

 

/*
 * lvgl extension definition for:
 * bool lv_slider_is_dragged(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_is_dragged(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    bool res = lv_slider_is_dragged(slider);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_is_dragged_obj, 1, 1, mp_lv_slider_is_dragged);

 

/*
 * lvgl extension definition for:
 * bool lv_slider_get_knob_in(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_slider_get_knob_in(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    bool res = lv_slider_get_knob_in(slider);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_knob_in_obj, 1, 1, mp_lv_slider_get_knob_in);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_slider_get_style(const lv_obj_t *slider, lv_slider_style_t type)
 */
 
STATIC mp_obj_t mp_lv_slider_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    lv_slider_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_slider_get_style(slider, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_slider_get_style_obj, 2, 2, mp_lv_slider_get_style);

 

    
/*
 * lvgl slider object definitions
 */

STATIC const mp_rom_map_elem_t slider_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_slider_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value_anim), MP_ROM_PTR(&mp_lv_slider_set_value_anim_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_slider_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_slider_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_slider_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_slider_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_knob_in), MP_ROM_PTR(&mp_lv_slider_set_knob_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_slider_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_slider_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_slider_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_dragged), MP_ROM_PTR(&mp_lv_slider_is_dragged_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_knob_in), MP_ROM_PTR(&mp_lv_slider_get_knob_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_slider_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_bar_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value_anim), MP_ROM_PTR(&mp_lv_bar_set_value_anim_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_bar_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_bar_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_bar_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_bar_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_bar_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_bar_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BAR_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_SLIDER_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(slider_locals_dict, slider_locals_dict_table);

STATIC void slider_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl slider");
}


STATIC mp_obj_t slider_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_slider_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_slider_type = {
    { &mp_type_type },
    .name = MP_QSTR_slider,
    .print = slider_print,
    .make_new = slider_make_new,
    .locals_dict = (mp_obj_dict_t*)&slider_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_sw_set_action(lv_obj_t *sw, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_sw_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *sw = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_sw_set_action(sw, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_set_action_obj, 2, 2, mp_lv_sw_set_action);

 

/*
 * lvgl extension definition for:
 * inline static bool lv_sw_get_state(const lv_obj_t *sw)
 */
 
STATIC mp_obj_t mp_lv_sw_get_state(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *sw = mp_to_lv(args[0]);
    bool res = lv_sw_get_state(sw);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_get_state_obj, 1, 1, mp_lv_sw_get_state);

 

/*
 * lvgl extension definition for:
 * inline static lv_action_t lv_sw_get_action(const lv_obj_t *slider)
 */
 
STATIC mp_obj_t mp_lv_sw_get_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *slider = mp_to_lv(args[0]);
    lv_action_t res = lv_sw_get_action(slider);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_get_action_obj, 1, 1, mp_lv_sw_get_action);

 

/*
 * lvgl extension definition for:
 * void lv_sw_on(lv_obj_t *sw)
 */
 
STATIC mp_obj_t mp_lv_sw_on(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *sw = mp_to_lv(args[0]);
    lv_sw_on(sw);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_on_obj, 1, 1, mp_lv_sw_on);

 

/*
 * lvgl extension definition for:
 * void lv_sw_off(lv_obj_t *sw)
 */
 
STATIC mp_obj_t mp_lv_sw_off(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *sw = mp_to_lv(args[0]);
    lv_sw_off(sw);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_off_obj, 1, 1, mp_lv_sw_off);

 

/*
 * lvgl extension definition for:
 * void lv_sw_set_style(lv_obj_t *sw, lv_sw_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_sw_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *sw = mp_to_lv(args[0]);
    lv_sw_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_sw_set_style(sw, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_set_style_obj, 3, 3, mp_lv_sw_set_style);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_sw_get_style(const lv_obj_t *sw, lv_sw_style_t type)
 */
 
STATIC mp_obj_t mp_lv_sw_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *sw = mp_to_lv(args[0]);
    lv_sw_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_sw_get_style(sw, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_sw_get_style_obj, 2, 2, mp_lv_sw_get_style);

 

    
/*
 * lvgl sw object definitions
 */

STATIC const mp_rom_map_elem_t sw_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_sw_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_state), MP_ROM_PTR(&mp_lv_sw_get_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_sw_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_on), MP_ROM_PTR(&mp_lv_sw_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_off), MP_ROM_PTR(&mp_lv_sw_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_sw_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_sw_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_slider_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value_anim), MP_ROM_PTR(&mp_lv_slider_set_value_anim_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_slider_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_slider_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_slider_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_slider_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_knob_in), MP_ROM_PTR(&mp_lv_slider_set_knob_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_slider_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_slider_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_slider_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_dragged), MP_ROM_PTR(&mp_lv_slider_is_dragged_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_knob_in), MP_ROM_PTR(&mp_lv_slider_get_knob_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_slider_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value), MP_ROM_PTR(&mp_lv_bar_set_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_value_anim), MP_ROM_PTR(&mp_lv_bar_set_value_anim_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_range), MP_ROM_PTR(&mp_lv_bar_set_range_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_bar_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_value), MP_ROM_PTR(&mp_lv_bar_get_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_min_value), MP_ROM_PTR(&mp_lv_bar_get_min_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_value), MP_ROM_PTR(&mp_lv_bar_get_max_value_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_bar_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_BAR_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_SLIDER_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_SW_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(sw_locals_dict, sw_locals_dict_table);

STATIC void sw_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl sw");
}


STATIC mp_obj_t sw_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_sw_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_sw_type = {
    { &mp_type_type },
    .name = MP_QSTR_sw,
    .print = sw_print,
    .make_new = sw_make_new,
    .locals_dict = (mp_obj_dict_t*)&sw_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_win_scroll_hor(lv_obj_t *win, lv_coord_t dist)
 */
 
STATIC mp_obj_t mp_lv_win_scroll_hor(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_coord_t dist = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_win_scroll_hor(win, dist);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_scroll_hor_obj, 2, 2, mp_lv_win_scroll_hor);

 

/*
 * lvgl extension definition for:
 * inline static void lv_win_scroll_ver(lv_obj_t *win, lv_coord_t dist)
 */
 
STATIC mp_obj_t mp_lv_win_scroll_ver(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_coord_t dist = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_win_scroll_ver(win, dist);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_scroll_ver_obj, 2, 2, mp_lv_win_scroll_ver);

 

/*
 * lvgl extension definition for:
 * void lv_win_clean(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_win_clean(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_win_clean(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_clean_obj, 1, 1, mp_lv_win_clean);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_win_add_btn(lv_obj_t *win, const void *img_src, lv_action_t rel_action)
 */
 
STATIC mp_obj_t mp_lv_win_add_btn(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    const void *img_src = mp_to_ptr(args[1]);
    set_action(args[0], args[2]);
    lv_action_t rel_action = &lv_action_t_callback;
    lv_obj_t* res = lv_win_add_btn(win, img_src, rel_action);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_add_btn_obj, 3, 3, mp_lv_win_add_btn);

 

/*
 * lvgl extension definition for:
 * lv_res_t lv_win_close_action(lv_obj_t *btn)
 */
 
STATIC mp_obj_t mp_lv_win_close_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *btn = mp_to_lv(args[0]);
    lv_res_t res = lv_win_close_action(btn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_close_action_obj, 1, 1, mp_lv_win_close_action);

 

/*
 * lvgl extension definition for:
 * void lv_win_set_title(lv_obj_t *win, const char *title)
 */
 
STATIC mp_obj_t mp_lv_win_set_title(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    const char *title = (char*)mp_obj_str_get_str(args[1]);
    lv_win_set_title(win, title);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_set_title_obj, 2, 2, mp_lv_win_set_title);

 

/*
 * lvgl extension definition for:
 * void lv_win_set_btn_size(lv_obj_t *win, lv_coord_t size)
 */
 
STATIC mp_obj_t mp_lv_win_set_btn_size(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_coord_t size = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_win_set_btn_size(win, size);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_set_btn_size_obj, 2, 2, mp_lv_win_set_btn_size);

 

/*
 * lvgl extension definition for:
 * void lv_win_set_layout(lv_obj_t *win, lv_layout_t layout)
 */
 
STATIC mp_obj_t mp_lv_win_set_layout(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_layout_t layout = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_win_set_layout(win, layout);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_set_layout_obj, 2, 2, mp_lv_win_set_layout);

 

/*
 * lvgl extension definition for:
 * void lv_win_set_sb_mode(lv_obj_t *win, lv_sb_mode_t sb_mode)
 */
 
STATIC mp_obj_t mp_lv_win_set_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_sb_mode_t sb_mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_win_set_sb_mode(win, sb_mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_set_sb_mode_obj, 2, 2, mp_lv_win_set_sb_mode);

 

/*
 * lvgl extension definition for:
 * void lv_win_set_style(lv_obj_t *win, lv_win_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_win_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_win_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_win_set_style(win, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_set_style_obj, 3, 3, mp_lv_win_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_win_get_title(const lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_title(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *win = mp_to_lv(args[0]);
    const char* res = lv_win_get_title(win);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_title_obj, 1, 1, mp_lv_win_get_title);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_win_get_content(const lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_content(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *win = mp_to_lv(args[0]);
    lv_obj_t* res = lv_win_get_content(win);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_content_obj, 1, 1, mp_lv_win_get_content);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_win_get_btn_size(const lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_btn_size(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *win = mp_to_lv(args[0]);
    lv_coord_t res = lv_win_get_btn_size(win);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_btn_size_obj, 1, 1, mp_lv_win_get_btn_size);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_win_get_from_btn(const lv_obj_t *ctrl_btn)
 */
 
STATIC mp_obj_t mp_lv_win_get_from_btn(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ctrl_btn = mp_to_lv(args[0]);
    lv_obj_t* res = lv_win_get_from_btn(ctrl_btn);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_from_btn_obj, 1, 1, mp_lv_win_get_from_btn);

 

/*
 * lvgl extension definition for:
 * lv_layout_t lv_win_get_layout(lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_layout(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_layout_t res = lv_win_get_layout(win);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_layout_obj, 1, 1, mp_lv_win_get_layout);

 

/*
 * lvgl extension definition for:
 * lv_sb_mode_t lv_win_get_sb_mode(lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_sb_mode_t res = lv_win_get_sb_mode(win);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_sb_mode_obj, 1, 1, mp_lv_win_get_sb_mode);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_win_get_width(lv_obj_t *win)
 */
 
STATIC mp_obj_t mp_lv_win_get_width(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_coord_t res = lv_win_get_width(win);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_width_obj, 1, 1, mp_lv_win_get_width);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_win_get_style(const lv_obj_t *win, lv_win_style_t type)
 */
 
STATIC mp_obj_t mp_lv_win_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *win = mp_to_lv(args[0]);
    lv_win_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_win_get_style(win, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_get_style_obj, 2, 2, mp_lv_win_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_win_focus(lv_obj_t *win, lv_obj_t *obj, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_win_focus(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *win = mp_to_lv(args[0]);
    lv_obj_t *obj = mp_to_lv(args[1]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[2]);
    lv_win_focus(win, obj, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_win_focus_obj, 3, 3, mp_lv_win_focus);

 

    
/*
 * lvgl win object definitions
 */

STATIC const mp_rom_map_elem_t win_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_win_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_win_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_win_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_btn), MP_ROM_PTR(&mp_lv_win_add_btn_obj) },
    { MP_ROM_QSTR(MP_QSTR_close_action), MP_ROM_PTR(&mp_lv_win_close_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_title), MP_ROM_PTR(&mp_lv_win_set_title_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_btn_size), MP_ROM_PTR(&mp_lv_win_set_btn_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_win_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_win_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_win_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_title), MP_ROM_PTR(&mp_lv_win_get_title_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_content), MP_ROM_PTR(&mp_lv_win_get_content_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btn_size), MP_ROM_PTR(&mp_lv_win_get_btn_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_from_btn), MP_ROM_PTR(&mp_lv_win_get_from_btn_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_win_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_win_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_win_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_win_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_win_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_WIN_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(win_locals_dict, win_locals_dict_table);

STATIC void win_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl win");
}


STATIC mp_obj_t win_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_win_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_win_type = {
    { &mp_type_type },
    .name = MP_QSTR_win,
    .print = win_print,
    .make_new = win_make_new,
    .locals_dict = (mp_obj_dict_t*)&win_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * void lv_tabview_clean(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_tabview_clean(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_tabview_clean(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_clean_obj, 1, 1, mp_lv_tabview_clean);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_tabview_add_tab(lv_obj_t *tabview, const char *name)
 */
 
STATIC mp_obj_t mp_lv_tabview_add_tab(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    const char *name = (char*)mp_obj_str_get_str(args[1]);
    lv_obj_t* res = lv_tabview_add_tab(tabview, name);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_add_tab_obj, 2, 2, mp_lv_tabview_add_tab);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_tab_act(lv_obj_t *tabview, uint16_t id, bool anim_en)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_tab_act(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t id = (uint16_t)mp_obj_int_get_checked(args[1]);
    bool anim_en = mp_obj_is_true(args[2]);
    lv_tabview_set_tab_act(tabview, id, anim_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_tab_act_obj, 3, 3, mp_lv_tabview_set_tab_act);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_tab_load_action(lv_obj_t *tabview, lv_tabview_action_t action)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_tab_load_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_tabview_action_t action = &lv_tabview_action_t_callback;
    lv_tabview_set_tab_load_action(tabview, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_tab_load_action_obj, 2, 2, mp_lv_tabview_set_tab_load_action);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_sliding(lv_obj_t *tabview, bool en)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_sliding(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_tabview_set_sliding(tabview, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_sliding_obj, 2, 2, mp_lv_tabview_set_sliding);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_anim_time(lv_obj_t *tabview, uint16_t anim_time)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_anim_time(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t anim_time = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_tabview_set_anim_time(tabview, anim_time);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_anim_time_obj, 2, 2, mp_lv_tabview_set_anim_time);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_style(lv_obj_t *tabview, lv_tabview_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    lv_tabview_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_tabview_set_style(tabview, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_style_obj, 3, 3, mp_lv_tabview_set_style);

 

/*
 * lvgl extension definition for:
 * void lv_tabview_set_btns_pos(lv_obj_t *tabview, lv_tabview_btns_pos_t btns_pos)
 */
 
STATIC mp_obj_t mp_lv_tabview_set_btns_pos(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *tabview = mp_to_lv(args[0]);
    lv_tabview_btns_pos_t btns_pos = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_tabview_set_btns_pos(tabview, btns_pos);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_set_btns_pos_obj, 2, 2, mp_lv_tabview_set_btns_pos);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_tabview_get_tab_act(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_tab_act(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t res = lv_tabview_get_tab_act(tabview);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_tab_act_obj, 1, 1, mp_lv_tabview_get_tab_act);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_tabview_get_tab_count(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_tab_count(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t res = lv_tabview_get_tab_count(tabview);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_tab_count_obj, 1, 1, mp_lv_tabview_get_tab_count);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_tabview_get_tab(const lv_obj_t *tabview, uint16_t id)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_tab(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t id = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_obj_t* res = lv_tabview_get_tab(tabview, id);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_tab_obj, 2, 2, mp_lv_tabview_get_tab);

 

/*
 * lvgl extension definition for:
 * lv_tabview_action_t lv_tabview_get_tab_load_action(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_tab_load_action(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    lv_tabview_action_t res = lv_tabview_get_tab_load_action(tabview);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_tab_load_action_obj, 1, 1, mp_lv_tabview_get_tab_load_action);

 

/*
 * lvgl extension definition for:
 * bool lv_tabview_get_sliding(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_sliding(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    bool res = lv_tabview_get_sliding(tabview);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_sliding_obj, 1, 1, mp_lv_tabview_get_sliding);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_tabview_get_anim_time(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_anim_time(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    uint16_t res = lv_tabview_get_anim_time(tabview);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_anim_time_obj, 1, 1, mp_lv_tabview_get_anim_time);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_tabview_get_style(const lv_obj_t *tabview, lv_tabview_style_t type)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    lv_tabview_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_tabview_get_style(tabview, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_style_obj, 2, 2, mp_lv_tabview_get_style);

 

/*
 * lvgl extension definition for:
 * lv_tabview_btns_pos_t lv_tabview_get_btns_pos(const lv_obj_t *tabview)
 */
 
STATIC mp_obj_t mp_lv_tabview_get_btns_pos(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *tabview = mp_to_lv(args[0]);
    lv_tabview_btns_pos_t res = lv_tabview_get_btns_pos(tabview);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tabview_get_btns_pos_obj, 1, 1, mp_lv_tabview_get_btns_pos);

 

    
/*
 * lvgl tabview object definitions
 */

STATIC const mp_rom_map_elem_t tabview_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_tabview_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_tab), MP_ROM_PTR(&mp_lv_tabview_add_tab_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_tab_act), MP_ROM_PTR(&mp_lv_tabview_set_tab_act_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_tab_load_action), MP_ROM_PTR(&mp_lv_tabview_set_tab_load_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sliding), MP_ROM_PTR(&mp_lv_tabview_set_sliding_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_anim_time), MP_ROM_PTR(&mp_lv_tabview_set_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_tabview_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_btns_pos), MP_ROM_PTR(&mp_lv_tabview_set_btns_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tab_act), MP_ROM_PTR(&mp_lv_tabview_get_tab_act_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tab_count), MP_ROM_PTR(&mp_lv_tabview_get_tab_count_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tab), MP_ROM_PTR(&mp_lv_tabview_get_tab_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_tab_load_action), MP_ROM_PTR(&mp_lv_tabview_get_tab_load_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sliding), MP_ROM_PTR(&mp_lv_tabview_get_sliding_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_anim_time), MP_ROM_PTR(&mp_lv_tabview_get_anim_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_tabview_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_btns_pos), MP_ROM_PTR(&mp_lv_tabview_get_btns_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_BTNS_POS), MP_ROM_PTR(&mp_LV_TABVIEW_BTNS_POS_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_TABVIEW_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(tabview_locals_dict, tabview_locals_dict_table);

STATIC void tabview_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl tabview");
}


STATIC mp_obj_t tabview_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_tabview_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_tabview_type = {
    { &mp_type_type },
    .name = MP_QSTR_tabview,
    .print = tabview_print,
    .make_new = tabview_make_new,
    .locals_dict = (mp_obj_dict_t*)&tabview_locals_dict,
};
    

/*
 * lvgl extension definition for:
 * inline static void lv_ta_set_action(lv_obj_t *ta, lv_action_t action)
 */
 
STATIC mp_obj_t mp_lv_ta_set_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    set_action(args[0], args[1]);
    lv_action_t action = &lv_action_t_callback;
    lv_ta_set_action(ta, action);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_action_obj, 2, 2, mp_lv_ta_set_action);

 

/*
 * lvgl extension definition for:
 * inline static void lv_ta_set_sb_mode(lv_obj_t *ta, lv_sb_mode_t mode)
 */
 
STATIC mp_obj_t mp_lv_ta_set_sb_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_sb_mode_t mode = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_ta_set_sb_mode(ta, mode);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_sb_mode_obj, 2, 2, mp_lv_ta_set_sb_mode);

 

/*
 * lvgl extension definition for:
 * inline static lv_action_t lv_ta_get_action(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_action(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_action_t res = lv_ta_get_action(ta);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_action_obj, 1, 1, mp_lv_ta_get_action);

 

/*
 * lvgl extension definition for:
 * inline static lv_sb_mode_t lv_ta_get_sb_mode(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_sb_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    lv_sb_mode_t res = lv_ta_get_sb_mode(ta);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_sb_mode_obj, 1, 1, mp_lv_ta_get_sb_mode);

 

/*
 * lvgl extension definition for:
 * void lv_ta_add_char(lv_obj_t *ta, uint32_t c)
 */
 
STATIC mp_obj_t mp_lv_ta_add_char(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    uint32_t c = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_ta_add_char(ta, c);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_add_char_obj, 2, 2, mp_lv_ta_add_char);

 

/*
 * lvgl extension definition for:
 * void lv_ta_add_text(lv_obj_t *ta, const char *txt)
 */
 
STATIC mp_obj_t mp_lv_ta_add_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    const char *txt = (char*)mp_obj_str_get_str(args[1]);
    lv_ta_add_text(ta, txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_add_text_obj, 2, 2, mp_lv_ta_add_text);

 

/*
 * lvgl extension definition for:
 * void lv_ta_del_char(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_del_char(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_del_char(ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_del_char_obj, 1, 1, mp_lv_ta_del_char);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_text(lv_obj_t *ta, const char *txt)
 */
 
STATIC mp_obj_t mp_lv_ta_set_text(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    const char *txt = (char*)mp_obj_str_get_str(args[1]);
    lv_ta_set_text(ta, txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_text_obj, 2, 2, mp_lv_ta_set_text);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_cursor_pos(lv_obj_t *ta, int16_t pos)
 */
 
STATIC mp_obj_t mp_lv_ta_set_cursor_pos(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    int16_t pos = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_ta_set_cursor_pos(ta, pos);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_cursor_pos_obj, 2, 2, mp_lv_ta_set_cursor_pos);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_cursor_type(lv_obj_t *ta, lv_cursor_type_t cur_type)
 */
 
STATIC mp_obj_t mp_lv_ta_set_cursor_type(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_cursor_type_t cur_type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_ta_set_cursor_type(ta, cur_type);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_cursor_type_obj, 2, 2, mp_lv_ta_set_cursor_type);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_pwd_mode(lv_obj_t *ta, bool pwd_en)
 */
 
STATIC mp_obj_t mp_lv_ta_set_pwd_mode(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    bool pwd_en = mp_obj_is_true(args[1]);
    lv_ta_set_pwd_mode(ta, pwd_en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_pwd_mode_obj, 2, 2, mp_lv_ta_set_pwd_mode);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_one_line(lv_obj_t *ta, bool en)
 */
 
STATIC mp_obj_t mp_lv_ta_set_one_line(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_ta_set_one_line(ta, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_one_line_obj, 2, 2, mp_lv_ta_set_one_line);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_text_align(lv_obj_t *ta, lv_label_align_t align)
 */
 
STATIC mp_obj_t mp_lv_ta_set_text_align(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_label_align_t align = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_ta_set_text_align(ta, align);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_text_align_obj, 2, 2, mp_lv_ta_set_text_align);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_accepted_chars(lv_obj_t *ta, const char *list)
 */
 
STATIC mp_obj_t mp_lv_ta_set_accepted_chars(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    const char *list = (char*)mp_obj_str_get_str(args[1]);
    lv_ta_set_accepted_chars(ta, list);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_accepted_chars_obj, 2, 2, mp_lv_ta_set_accepted_chars);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_max_length(lv_obj_t *ta, uint16_t num)
 */
 
STATIC mp_obj_t mp_lv_ta_set_max_length(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    uint16_t num = (uint16_t)mp_obj_int_get_checked(args[1]);
    lv_ta_set_max_length(ta, num);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_max_length_obj, 2, 2, mp_lv_ta_set_max_length);

 

/*
 * lvgl extension definition for:
 * void lv_ta_set_style(lv_obj_t *ta, lv_ta_style_t type, lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_ta_set_style(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t *style = mp_write_ptr_lv_style_t(args[2]);
    lv_ta_set_style(ta, type, style);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_set_style_obj, 3, 3, mp_lv_ta_set_style);

 

/*
 * lvgl extension definition for:
 * const char *lv_ta_get_text(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_text(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    const char* res = lv_ta_get_text(ta);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_text_obj, 1, 1, mp_lv_ta_get_text);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_ta_get_label(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_label(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    lv_obj_t* res = lv_ta_get_label(ta);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_label_obj, 1, 1, mp_lv_ta_get_label);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_ta_get_cursor_pos(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_cursor_pos(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    uint16_t res = lv_ta_get_cursor_pos(ta);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_cursor_pos_obj, 1, 1, mp_lv_ta_get_cursor_pos);

 

/*
 * lvgl extension definition for:
 * lv_cursor_type_t lv_ta_get_cursor_type(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_cursor_type(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    lv_cursor_type_t res = lv_ta_get_cursor_type(ta);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_cursor_type_obj, 1, 1, mp_lv_ta_get_cursor_type);

 

/*
 * lvgl extension definition for:
 * bool lv_ta_get_pwd_mode(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_pwd_mode(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    bool res = lv_ta_get_pwd_mode(ta);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_pwd_mode_obj, 1, 1, mp_lv_ta_get_pwd_mode);

 

/*
 * lvgl extension definition for:
 * bool lv_ta_get_one_line(const lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_one_line(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    bool res = lv_ta_get_one_line(ta);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_one_line_obj, 1, 1, mp_lv_ta_get_one_line);

 

/*
 * lvgl extension definition for:
 * const char *lv_ta_get_accepted_chars(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_accepted_chars(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    const char* res = lv_ta_get_accepted_chars(ta);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_accepted_chars_obj, 1, 1, mp_lv_ta_get_accepted_chars);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_ta_get_max_length(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_get_max_length(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    uint16_t res = lv_ta_get_max_length(ta);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_max_length_obj, 1, 1, mp_lv_ta_get_max_length);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_ta_get_style(const lv_obj_t *ta, lv_ta_style_t type)
 */
 
STATIC mp_obj_t mp_lv_ta_get_style(size_t n_args, const mp_obj_t *args)
{
    const lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_style_t type = (uint8_t)mp_obj_int_get_checked(args[1]);
    lv_style_t* res = lv_ta_get_style(ta, type);
    return mp_read_ptr_lv_style_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_get_style_obj, 2, 2, mp_lv_ta_get_style);

 

/*
 * lvgl extension definition for:
 * void lv_ta_cursor_right(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_cursor_right(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_cursor_right(ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_cursor_right_obj, 1, 1, mp_lv_ta_cursor_right);

 

/*
 * lvgl extension definition for:
 * void lv_ta_cursor_left(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_cursor_left(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_cursor_left(ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_cursor_left_obj, 1, 1, mp_lv_ta_cursor_left);

 

/*
 * lvgl extension definition for:
 * void lv_ta_cursor_down(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_cursor_down(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_cursor_down(ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_cursor_down_obj, 1, 1, mp_lv_ta_cursor_down);

 

/*
 * lvgl extension definition for:
 * void lv_ta_cursor_up(lv_obj_t *ta)
 */
 
STATIC mp_obj_t mp_lv_ta_cursor_up(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *ta = mp_to_lv(args[0]);
    lv_ta_cursor_up(ta);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ta_cursor_up_obj, 1, 1, mp_lv_ta_cursor_up);

 

    
/*
 * lvgl ta object definitions
 */

STATIC const mp_rom_map_elem_t ta_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_set_action), MP_ROM_PTR(&mp_lv_ta_set_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_ta_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_action), MP_ROM_PTR(&mp_lv_ta_get_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_ta_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_char), MP_ROM_PTR(&mp_lv_ta_add_char_obj) },
    { MP_ROM_QSTR(MP_QSTR_add_text), MP_ROM_PTR(&mp_lv_ta_add_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_del_char), MP_ROM_PTR(&mp_lv_ta_del_char_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_text), MP_ROM_PTR(&mp_lv_ta_set_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cursor_pos), MP_ROM_PTR(&mp_lv_ta_set_cursor_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cursor_type), MP_ROM_PTR(&mp_lv_ta_set_cursor_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pwd_mode), MP_ROM_PTR(&mp_lv_ta_set_pwd_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_one_line), MP_ROM_PTR(&mp_lv_ta_set_one_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_text_align), MP_ROM_PTR(&mp_lv_ta_set_text_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_accepted_chars), MP_ROM_PTR(&mp_lv_ta_set_accepted_chars_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_max_length), MP_ROM_PTR(&mp_lv_ta_set_max_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_ta_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_text), MP_ROM_PTR(&mp_lv_ta_get_text_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_label), MP_ROM_PTR(&mp_lv_ta_get_label_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cursor_pos), MP_ROM_PTR(&mp_lv_ta_get_cursor_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cursor_type), MP_ROM_PTR(&mp_lv_ta_get_cursor_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pwd_mode), MP_ROM_PTR(&mp_lv_ta_get_pwd_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_one_line), MP_ROM_PTR(&mp_lv_ta_get_one_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_accepted_chars), MP_ROM_PTR(&mp_lv_ta_get_accepted_chars_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_max_length), MP_ROM_PTR(&mp_lv_ta_get_max_length_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_ta_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor_right), MP_ROM_PTR(&mp_lv_ta_cursor_right_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor_left), MP_ROM_PTR(&mp_lv_ta_cursor_left_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor_down), MP_ROM_PTR(&mp_lv_ta_cursor_down_obj) },
    { MP_ROM_QSTR(MP_QSTR_cursor_up), MP_ROM_PTR(&mp_lv_ta_cursor_up_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_fit), MP_ROM_PTR(&mp_lv_page_set_scrl_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_width), MP_ROM_PTR(&mp_lv_page_set_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_height), MP_ROM_PTR(&mp_lv_page_set_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_scrl_layout), MP_ROM_PTR(&mp_lv_page_set_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_width), MP_ROM_PTR(&mp_lv_page_get_scrl_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_height), MP_ROM_PTR(&mp_lv_page_get_scrl_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_layout), MP_ROM_PTR(&mp_lv_page_get_scrl_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_hor_fit), MP_ROM_PTR(&mp_lv_page_get_scrl_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl_fit_ver), MP_ROM_PTR(&mp_lv_page_get_scrl_fit_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_page_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pr_action), MP_ROM_PTR(&mp_lv_page_get_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_rel_action), MP_ROM_PTR(&mp_lv_page_get_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_scrl), MP_ROM_PTR(&mp_lv_page_get_scrl_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_rel_action), MP_ROM_PTR(&mp_lv_page_set_rel_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pr_action), MP_ROM_PTR(&mp_lv_page_set_pr_action_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_sb_mode), MP_ROM_PTR(&mp_lv_page_set_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_arrow_scroll), MP_ROM_PTR(&mp_lv_page_set_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_page_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_sb_mode), MP_ROM_PTR(&mp_lv_page_get_sb_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_arrow_scroll), MP_ROM_PTR(&mp_lv_page_get_arrow_scroll_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_page_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_page_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_page_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_glue_obj), MP_ROM_PTR(&mp_lv_page_glue_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_focus), MP_ROM_PTR(&mp_lv_page_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_hor), MP_ROM_PTR(&mp_lv_page_scroll_hor_obj) },
    { MP_ROM_QSTR(MP_QSTR_scroll_ver), MP_ROM_PTR(&mp_lv_page_scroll_ver_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_cont_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_cont_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_layout), MP_ROM_PTR(&mp_lv_cont_set_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_fit), MP_ROM_PTR(&mp_lv_cont_set_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_layout), MP_ROM_PTR(&mp_lv_cont_get_layout_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hor_fit), MP_ROM_PTR(&mp_lv_cont_get_hor_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ver_fit), MP_ROM_PTR(&mp_lv_cont_get_ver_fit_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_width), MP_ROM_PTR(&mp_lv_cont_get_fit_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_fit_height), MP_ROM_PTR(&mp_lv_cont_get_fit_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_del), MP_ROM_PTR(&mp_lv_obj_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_clean), MP_ROM_PTR(&mp_lv_obj_clean_obj) },
    { MP_ROM_QSTR(MP_QSTR_invalidate), MP_ROM_PTR(&mp_lv_obj_invalidate_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_parent), MP_ROM_PTR(&mp_lv_obj_set_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pos), MP_ROM_PTR(&mp_lv_obj_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_x), MP_ROM_PTR(&mp_lv_obj_set_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_y), MP_ROM_PTR(&mp_lv_obj_set_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_size), MP_ROM_PTR(&mp_lv_obj_set_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_width), MP_ROM_PTR(&mp_lv_obj_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_height), MP_ROM_PTR(&mp_lv_obj_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_align), MP_ROM_PTR(&mp_lv_obj_align_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_style), MP_ROM_PTR(&mp_lv_obj_set_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_style), MP_ROM_PTR(&mp_lv_obj_refresh_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_report_style_mod), MP_ROM_PTR(&mp_lv_obj_report_style_mod_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_hidden), MP_ROM_PTR(&mp_lv_obj_set_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_click), MP_ROM_PTR(&mp_lv_obj_set_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_top), MP_ROM_PTR(&mp_lv_obj_set_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag), MP_ROM_PTR(&mp_lv_obj_set_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_throw), MP_ROM_PTR(&mp_lv_obj_set_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_drag_parent), MP_ROM_PTR(&mp_lv_obj_set_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale_enable), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_opa_scale), MP_ROM_PTR(&mp_lv_obj_set_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_protect), MP_ROM_PTR(&mp_lv_obj_set_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_clear_protect), MP_ROM_PTR(&mp_lv_obj_clear_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_signal_func), MP_ROM_PTR(&mp_lv_obj_set_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_design_func), MP_ROM_PTR(&mp_lv_obj_set_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_allocate_ext_attr), MP_ROM_PTR(&mp_lv_obj_allocate_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_refresh_ext_size), MP_ROM_PTR(&mp_lv_obj_refresh_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_num), MP_ROM_PTR(&mp_lv_obj_set_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_free_ptr), MP_ROM_PTR(&mp_lv_obj_set_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_screen), MP_ROM_PTR(&mp_lv_obj_get_screen_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_parent), MP_ROM_PTR(&mp_lv_obj_get_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child), MP_ROM_PTR(&mp_lv_obj_get_child_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_child_back), MP_ROM_PTR(&mp_lv_obj_get_child_back_obj) },
    { MP_ROM_QSTR(MP_QSTR_count_children), MP_ROM_PTR(&mp_lv_obj_count_children_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_coords), MP_ROM_PTR(&mp_lv_obj_get_coords_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_x), MP_ROM_PTR(&mp_lv_obj_get_x_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_y), MP_ROM_PTR(&mp_lv_obj_get_y_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_width), MP_ROM_PTR(&mp_lv_obj_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_height), MP_ROM_PTR(&mp_lv_obj_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_size), MP_ROM_PTR(&mp_lv_obj_get_ext_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_style), MP_ROM_PTR(&mp_lv_obj_get_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_hidden), MP_ROM_PTR(&mp_lv_obj_get_hidden_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_click), MP_ROM_PTR(&mp_lv_obj_get_click_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_top), MP_ROM_PTR(&mp_lv_obj_get_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag), MP_ROM_PTR(&mp_lv_obj_get_drag_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_throw), MP_ROM_PTR(&mp_lv_obj_get_drag_throw_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_drag_parent), MP_ROM_PTR(&mp_lv_obj_get_drag_parent_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_opa_scale), MP_ROM_PTR(&mp_lv_obj_get_opa_scale_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_protect), MP_ROM_PTR(&mp_lv_obj_get_protect_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_protected), MP_ROM_PTR(&mp_lv_obj_is_protected_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_signal_func), MP_ROM_PTR(&mp_lv_obj_get_signal_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_design_func), MP_ROM_PTR(&mp_lv_obj_get_design_func_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_ext_attr), MP_ROM_PTR(&mp_lv_obj_get_ext_attr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_num), MP_ROM_PTR(&mp_lv_obj_get_free_num_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_free_ptr), MP_ROM_PTR(&mp_lv_obj_get_free_ptr_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_group), MP_ROM_PTR(&mp_lv_obj_get_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_focused), MP_ROM_PTR(&mp_lv_obj_is_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_PAGE_STYLE_type) },
    { MP_ROM_QSTR(MP_QSTR_STYLE), MP_ROM_PTR(&mp_LV_TA_STYLE_type) }
};

STATIC MP_DEFINE_CONST_DICT(ta_locals_dict, ta_locals_dict_table);

STATIC void ta_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl ta");
}


STATIC mp_obj_t ta_make_new(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    return make_new(&lv_ta_create, type, n_args, n_kw, args);           
}


STATIC const mp_obj_type_t mp_ta_type = {
    { &mp_type_type },
    .name = MP_QSTR_ta,
    .print = ta_print,
    .make_new = ta_make_new,
    .locals_dict = (mp_obj_dict_t*)&ta_locals_dict,
};
    

STATIC inline const mp_obj_type_t *get_BaseObj_type()
{
    return &mp_obj_type;
}


/* 
 *
 * Global Module Functions
 *
 */


/*
 * lvgl extension definition for:
 * inline static uint8_t lv_color_to1(lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_color_to1(size_t n_args, const mp_obj_t *args)
{
    lv_color_t color = mp_write_lv_color16_t(args[0]);
    uint8_t res = lv_color_to1(color);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_to1_obj, 1, 1, mp_lv_color_to1);

 

/*
 * lvgl extension definition for:
 * inline static uint8_t lv_color_to8(lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_color_to8(size_t n_args, const mp_obj_t *args)
{
    lv_color_t color = mp_write_lv_color16_t(args[0]);
    uint8_t res = lv_color_to8(color);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_to8_obj, 1, 1, mp_lv_color_to8);

 

/*
 * lvgl extension definition for:
 * inline static uint16_t lv_color_to16(lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_color_to16(size_t n_args, const mp_obj_t *args)
{
    lv_color_t color = mp_write_lv_color16_t(args[0]);
    uint16_t res = lv_color_to16(color);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_to16_obj, 1, 1, mp_lv_color_to16);

 

/*
 * lvgl extension definition for:
 * inline static uint32_t lv_color_to32(lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_color_to32(size_t n_args, const mp_obj_t *args)
{
    lv_color_t color = mp_write_lv_color16_t(args[0]);
    uint32_t res = lv_color_to32(color);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_to32_obj, 1, 1, mp_lv_color_to32);

 

/*
 * lvgl extension definition for:
 * inline static lv_color_t lv_color_mix(lv_color_t c1, lv_color_t c2, uint8_t mix)
 */
 
STATIC mp_obj_t mp_lv_color_mix(size_t n_args, const mp_obj_t *args)
{
    lv_color_t c1 = mp_write_lv_color16_t(args[0]);
    lv_color_t c2 = mp_write_lv_color16_t(args[1]);
    uint8_t mix = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_color_t res = lv_color_mix(c1, c2, mix);
    return mp_read_lv_color16_t(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_mix_obj, 3, 3, mp_lv_color_mix);

 

/*
 * lvgl extension definition for:
 * inline static uint8_t lv_color_brightness(lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_color_brightness(size_t n_args, const mp_obj_t *args)
{
    lv_color_t color = mp_write_lv_color16_t(args[0]);
    uint8_t res = lv_color_brightness(color);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_brightness_obj, 1, 1, mp_lv_color_brightness);

 

/*
 * lvgl extension definition for:
 * inline static void lv_area_copy(lv_area_t *dest, const lv_area_t *src)
 */
 
STATIC mp_obj_t mp_lv_area_copy(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *dest = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *src = mp_write_ptr_lv_area_t(args[1]);
    lv_area_copy(dest, src);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_copy_obj, 2, 2, mp_lv_area_copy);

 

/*
 * lvgl extension definition for:
 * inline static lv_coord_t lv_area_get_width(const lv_area_t *area_p)
 */
 
STATIC mp_obj_t mp_lv_area_get_width(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t res = lv_area_get_width(area_p);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_get_width_obj, 1, 1, mp_lv_area_get_width);

 

/*
 * lvgl extension definition for:
 * inline static lv_coord_t lv_area_get_height(const lv_area_t *area_p)
 */
 
STATIC mp_obj_t mp_lv_area_get_height(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t res = lv_area_get_height(area_p);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_get_height_obj, 1, 1, mp_lv_area_get_height);

 

/*
 * lvgl extension definition for:
 * inline static uint8_t lv_font_get_height(const lv_font_t *font_p)
 */
 
STATIC mp_obj_t mp_lv_font_get_height(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font_p = mp_to_ptr(args[0]);
    uint8_t res = lv_font_get_height(font_p);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_height_obj, 1, 1, mp_lv_font_get_height);

 

/*
 * lvgl extension definition for:
 * lv_color_t lv_color_hsv_to_rgb(uint16_t h, uint8_t s, uint8_t v)
 */
 
STATIC mp_obj_t mp_lv_color_hsv_to_rgb(size_t n_args, const mp_obj_t *args)
{
    uint16_t h = (uint16_t)mp_obj_int_get_checked(args[0]);
    uint8_t s = (uint8_t)mp_obj_int_get_checked(args[1]);
    uint8_t v = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_color_t res = lv_color_hsv_to_rgb(h, s, v);
    return mp_read_lv_color16_t(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_hsv_to_rgb_obj, 3, 3, mp_lv_color_hsv_to_rgb);

 

/*
 * Struct lv_color_hsv_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_color_hsv_t_type();

STATIC inline lv_color_hsv_t* mp_write_ptr_lv_color_hsv_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_color_hsv_t*)self->data;
}

#define mp_write_lv_color_hsv_t(struct_obj) *mp_write_ptr_lv_color_hsv_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_color_hsv_t(lv_color_hsv_t *field)
{
    return lv_to_mp_struct(get_mp_lv_color_hsv_t_type(), field);
}

#define mp_read_lv_color_hsv_t(field) mp_read_ptr_lv_color_hsv_t(&field)

STATIC void mp_lv_color_hsv_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_color_hsv_t *data = (lv_color_hsv_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_h: dest[0] = mp_obj_new_int_from_uint(data->h); break; // converting from uint16_t;
            case MP_QSTR_s: dest[0] = mp_obj_new_int_from_uint(data->s); break; // converting from uint8_t;
            case MP_QSTR_v: dest[0] = mp_obj_new_int_from_uint(data->v); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_color_hsv_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_h: data->h = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_s: data->s = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_v: data->v = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_color_hsv_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_color_hsv_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_color_hsv_t");
}

STATIC const mp_rom_map_elem_t mp_lv_color_hsv_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_color_hsv_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_color_hsv_t_locals_dict, mp_lv_color_hsv_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_color_hsv_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_color_hsv_t,
    .print = mp_lv_color_hsv_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_color_hsv_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_color_hsv_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_color_hsv_t_type()
{
    return &mp_lv_color_hsv_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_color_hsv_t lv_color_rgb_to_hsv(uint8_t r, uint8_t g, uint8_t b)
 */
 
STATIC mp_obj_t mp_lv_color_rgb_to_hsv(size_t n_args, const mp_obj_t *args)
{
    uint8_t r = (uint8_t)mp_obj_int_get_checked(args[0]);
    uint8_t g = (uint8_t)mp_obj_int_get_checked(args[1]);
    uint8_t b = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_color_hsv_t res = lv_color_rgb_to_hsv(r, g, b);
    return mp_read_lv_color_hsv_t(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_color_rgb_to_hsv_obj, 3, 3, mp_lv_color_rgb_to_hsv);

 

/*
 * lvgl extension definition for:
 * void lv_area_set(lv_area_t *area_p, lv_coord_t x1, lv_coord_t y1, lv_coord_t x2, lv_coord_t y2)
 */
 
STATIC mp_obj_t mp_lv_area_set(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t x1 = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t y1 = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_coord_t x2 = (int16_t)mp_obj_int_get_checked(args[3]);
    lv_coord_t y2 = (int16_t)mp_obj_int_get_checked(args[4]);
    lv_area_set(area_p, x1, y1, x2, y2);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_set_obj, 5, 5, mp_lv_area_set);

 

/*
 * lvgl extension definition for:
 * void lv_area_set_width(lv_area_t *area_p, lv_coord_t w)
 */
 
STATIC mp_obj_t mp_lv_area_set_width(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t w = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_area_set_width(area_p, w);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_set_width_obj, 2, 2, mp_lv_area_set_width);

 

/*
 * lvgl extension definition for:
 * void lv_area_set_height(lv_area_t *area_p, lv_coord_t h)
 */
 
STATIC mp_obj_t mp_lv_area_set_height(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t h = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_area_set_height(area_p, h);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_set_height_obj, 2, 2, mp_lv_area_set_height);

 

/*
 * lvgl extension definition for:
 * void lv_area_set_pos(lv_area_t *area_p, lv_coord_t x, lv_coord_t y)
 */
 
STATIC mp_obj_t mp_lv_area_set_pos(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    lv_coord_t x = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_area_set_pos(area_p, x, y);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_set_pos_obj, 3, 3, mp_lv_area_set_pos);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_area_get_size(const lv_area_t *area_p)
 */
 
STATIC mp_obj_t mp_lv_area_get_size(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *area_p = mp_write_ptr_lv_area_t(args[0]);
    uint32_t res = lv_area_get_size(area_p);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_get_size_obj, 1, 1, mp_lv_area_get_size);

 

/*
 * lvgl extension definition for:
 * bool lv_area_intersect(lv_area_t *res_p, const lv_area_t *a1_p, const lv_area_t *a2_p)
 */
 
STATIC mp_obj_t mp_lv_area_intersect(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *res_p = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *a1_p = mp_write_ptr_lv_area_t(args[1]);
    const lv_area_t *a2_p = mp_write_ptr_lv_area_t(args[2]);
    bool res = lv_area_intersect(res_p, a1_p, a2_p);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_intersect_obj, 3, 3, mp_lv_area_intersect);

 

/*
 * lvgl extension definition for:
 * void lv_area_join(lv_area_t *a_res_p, const lv_area_t *a1_p, const lv_area_t *a2_p)
 */
 
STATIC mp_obj_t mp_lv_area_join(size_t n_args, const mp_obj_t *args)
{
    lv_area_t *a_res_p = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *a1_p = mp_write_ptr_lv_area_t(args[1]);
    const lv_area_t *a2_p = mp_write_ptr_lv_area_t(args[2]);
    lv_area_join(a_res_p, a1_p, a2_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_join_obj, 3, 3, mp_lv_area_join);

 

/*
 * lvgl extension definition for:
 * bool lv_area_is_point_on(const lv_area_t *a_p, const lv_point_t *p_p)
 */
 
STATIC mp_obj_t mp_lv_area_is_point_on(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *a_p = mp_write_ptr_lv_area_t(args[0]);
    const lv_point_t *p_p = mp_write_ptr_lv_point_t(args[1]);
    bool res = lv_area_is_point_on(a_p, p_p);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_is_point_on_obj, 2, 2, mp_lv_area_is_point_on);

 

/*
 * lvgl extension definition for:
 * bool lv_area_is_on(const lv_area_t *a1_p, const lv_area_t *a2_p)
 */
 
STATIC mp_obj_t mp_lv_area_is_on(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *a1_p = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *a2_p = mp_write_ptr_lv_area_t(args[1]);
    bool res = lv_area_is_on(a1_p, a2_p);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_is_on_obj, 2, 2, mp_lv_area_is_on);

 

/*
 * lvgl extension definition for:
 * bool lv_area_is_in(const lv_area_t *ain_p, const lv_area_t *aholder_p)
 */
 
STATIC mp_obj_t mp_lv_area_is_in(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *ain_p = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *aholder_p = mp_write_ptr_lv_area_t(args[1]);
    bool res = lv_area_is_in(ain_p, aholder_p);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_area_is_in_obj, 2, 2, mp_lv_area_is_in);

 

/*
 * lvgl extension definition for:
 * void lv_font_init(void)
 */
 
STATIC mp_obj_t mp_lv_font_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_font_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_init_obj, 0, 0, mp_lv_font_init);

 

/*
 * lvgl extension definition for:
 * void lv_font_add(lv_font_t *child, lv_font_t *parent)
 */
 
STATIC mp_obj_t mp_lv_font_add(size_t n_args, const mp_obj_t *args)
{
    lv_font_t *child = mp_to_ptr(args[0]);
    lv_font_t *parent = mp_to_ptr(args[1]);
    lv_font_add(child, parent);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_add_obj, 2, 2, mp_lv_font_add);

 

/*
 * lvgl extension definition for:
 * bool lv_font_is_monospace(const lv_font_t *font_p, uint32_t letter)
 */
 
STATIC mp_obj_t mp_lv_font_is_monospace(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font_p = mp_to_ptr(args[0]);
    uint32_t letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    bool res = lv_font_is_monospace(font_p, letter);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_is_monospace_obj, 2, 2, mp_lv_font_is_monospace);

 

/*
 * Function NOT generated:
 * Missing convertion from const uint8_t*
 * const uint8_t *lv_font_get_bitmap(const lv_font_t *font_p, uint32_t letter)
 */
    

/*
 * lvgl extension definition for:
 * uint8_t lv_font_get_width(const lv_font_t *font_p, uint32_t letter)
 */
 
STATIC mp_obj_t mp_lv_font_get_width(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font_p = mp_to_ptr(args[0]);
    uint32_t letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    uint8_t res = lv_font_get_width(font_p, letter);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_width_obj, 2, 2, mp_lv_font_get_width);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_font_get_real_width(const lv_font_t *font_p, uint32_t letter)
 */
 
STATIC mp_obj_t mp_lv_font_get_real_width(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font_p = mp_to_ptr(args[0]);
    uint32_t letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    uint8_t res = lv_font_get_real_width(font_p, letter);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_real_width_obj, 2, 2, mp_lv_font_get_real_width);

 

/*
 * lvgl extension definition for:
 * uint8_t lv_font_get_bpp(const lv_font_t *font, uint32_t letter)
 */
 
STATIC mp_obj_t mp_lv_font_get_bpp(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font = mp_to_ptr(args[0]);
    uint32_t letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    uint8_t res = lv_font_get_bpp(font, letter);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_bpp_obj, 2, 2, mp_lv_font_get_bpp);

 

/*
 * Function NOT generated:
 * Missing convertion from const uint8_t*
 * const uint8_t *lv_font_get_bitmap_continuous(const lv_font_t *font, uint32_t unicode_letter)
 */
    

/*
 * Function NOT generated:
 * Missing convertion from const uint8_t*
 * const uint8_t *lv_font_get_bitmap_sparse(const lv_font_t *font, uint32_t unicode_letter)
 */
    

/*
 * lvgl extension definition for:
 * int16_t lv_font_get_width_continuous(const lv_font_t *font, uint32_t unicode_letter)
 */
 
STATIC mp_obj_t mp_lv_font_get_width_continuous(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font = mp_to_ptr(args[0]);
    uint32_t unicode_letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    int16_t res = lv_font_get_width_continuous(font, unicode_letter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_width_continuous_obj, 2, 2, mp_lv_font_get_width_continuous);

 

/*
 * lvgl extension definition for:
 * int16_t lv_font_get_width_sparse(const lv_font_t *font, uint32_t unicode_letter)
 */
 
STATIC mp_obj_t mp_lv_font_get_width_sparse(size_t n_args, const mp_obj_t *args)
{
    const lv_font_t *font = mp_to_ptr(args[0]);
    uint32_t unicode_letter = (uint32_t)mp_obj_int_get_checked(args[1]);
    int16_t res = lv_font_get_width_sparse(font, unicode_letter);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_get_width_sparse_obj, 2, 2, mp_lv_font_get_width_sparse);

 

/*
 * lvgl extension definition for:
 * void lv_font_builtin_init(void)
 */
 
STATIC mp_obj_t mp_lv_font_builtin_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_font_builtin_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_font_builtin_init_obj, 0, 0, mp_lv_font_builtin_init);

 

/*
 * lvgl extension definition for:
 * void lv_anim_init(void)
 */
 
STATIC mp_obj_t mp_lv_anim_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_anim_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_init_obj, 0, 0, mp_lv_anim_init);

 

/*
 * Struct lv_anim_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_anim_t_type();

STATIC inline lv_anim_t* mp_write_ptr_lv_anim_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_anim_t*)self->data;
}

#define mp_write_lv_anim_t(struct_obj) *mp_write_ptr_lv_anim_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_anim_t(lv_anim_t *field)
{
    return lv_to_mp_struct(get_mp_lv_anim_t_type(), field);
}

#define mp_read_lv_anim_t(field) mp_read_ptr_lv_anim_t(&field)

STATIC void mp_lv_anim_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_anim_t *data = (lv_anim_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_var: dest[0] = ptr_to_mp((void*)data->var); break; // converting from void*;
            case MP_QSTR_fp: dest[0] = ptr_to_mp(data->fp); break; // converting from lv_anim_fp_t;
            case MP_QSTR_end_cb: dest[0] = ptr_to_mp(data->end_cb); break; // converting from lv_anim_cb_t;
            case MP_QSTR_path: dest[0] = ptr_to_mp(data->path); break; // converting from lv_anim_path_t;
            case MP_QSTR_start: dest[0] = mp_obj_new_int(data->start); break; // converting from int32_t;
            case MP_QSTR_end: dest[0] = mp_obj_new_int(data->end); break; // converting from int32_t;
            case MP_QSTR_time: dest[0] = mp_obj_new_int_from_uint(data->time); break; // converting from uint16_t;
            case MP_QSTR_act_time: dest[0] = mp_obj_new_int(data->act_time); break; // converting from int16_t;
            case MP_QSTR_playback_pause: dest[0] = mp_obj_new_int_from_uint(data->playback_pause); break; // converting from uint16_t;
            case MP_QSTR_repeat_pause: dest[0] = mp_obj_new_int_from_uint(data->repeat_pause); break; // converting from uint16_t;
            case MP_QSTR_playback: dest[0] = mp_obj_new_int_from_uint(data->playback); break; // converting from uint8_t;
            case MP_QSTR_repeat: dest[0] = mp_obj_new_int_from_uint(data->repeat); break; // converting from uint8_t;
            case MP_QSTR_playback_now: dest[0] = mp_obj_new_int_from_uint(data->playback_now); break; // converting from uint8_t;
            case MP_QSTR_has_run: dest[0] = mp_obj_new_int_from_uint(data->has_run); break; // converting from uint32_t;
            default: field_not_found(MP_QSTR_lv_anim_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_var: data->var = (void*)mp_to_ptr(dest[1]); break; // converting to void*;
                case MP_QSTR_fp: data->fp = mp_to_ptr(dest[1]); break; // converting to lv_anim_fp_t;
                case MP_QSTR_end_cb: data->end_cb = mp_to_ptr(dest[1]); break; // converting to lv_anim_cb_t;
                case MP_QSTR_path: data->path = mp_to_ptr(dest[1]); break; // converting to lv_anim_path_t;
                case MP_QSTR_start: data->start = (int32_t)mp_obj_int_get_checked(dest[1]); break; // converting to int32_t;
                case MP_QSTR_end: data->end = (int32_t)mp_obj_int_get_checked(dest[1]); break; // converting to int32_t;
                case MP_QSTR_time: data->time = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_act_time: data->act_time = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to int16_t;
                case MP_QSTR_playback_pause: data->playback_pause = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_repeat_pause: data->repeat_pause = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_playback: data->playback = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_repeat: data->repeat = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_playback_now: data->playback_now = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_has_run: data->has_run = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                default: field_not_found(MP_QSTR_lv_anim_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_anim_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_anim_t");
}

STATIC const mp_rom_map_elem_t mp_lv_anim_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_anim_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_anim_t_locals_dict, mp_lv_anim_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_anim_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_anim_t,
    .print = mp_lv_anim_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_anim_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_anim_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_anim_t_type()
{
    return &mp_lv_anim_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_anim_create(lv_anim_t *anim_p)
 */
 
STATIC mp_obj_t mp_lv_anim_create(size_t n_args, const mp_obj_t *args)
{
    lv_anim_t *anim_p = mp_write_ptr_lv_anim_t(args[0]);
    lv_anim_create(anim_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_create_obj, 1, 1, mp_lv_anim_create);

 

/*
 * lvgl extension definition for:
 * bool lv_anim_del(void *var, lv_anim_fp_t fp)
 */
 
STATIC mp_obj_t mp_lv_anim_del(size_t n_args, const mp_obj_t *args)
{
    void *var = mp_to_ptr(args[0]);
    set_action(args[0], args[1]);
    lv_anim_fp_t fp = &lv_anim_fp_t_callback;
    bool res = lv_anim_del(var, fp);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_del_obj, 2, 2, mp_lv_anim_del);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_anim_speed_to_time(uint16_t speed, int32_t start, int32_t end)
 */
 
STATIC mp_obj_t mp_lv_anim_speed_to_time(size_t n_args, const mp_obj_t *args)
{
    uint16_t speed = (uint16_t)mp_obj_int_get_checked(args[0]);
    int32_t start = (int32_t)mp_obj_int_get_checked(args[1]);
    int32_t end = (int32_t)mp_obj_int_get_checked(args[2]);
    uint16_t res = lv_anim_speed_to_time(speed, start, end);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_speed_to_time_obj, 3, 3, mp_lv_anim_speed_to_time);

 

/*
 * lvgl extension definition for:
 * int32_t lv_anim_path_linear(const lv_anim_t *a)
 */
 
STATIC mp_obj_t mp_lv_anim_path_linear(size_t n_args, const mp_obj_t *args)
{
    const lv_anim_t *a = mp_write_ptr_lv_anim_t(args[0]);
    int32_t res = lv_anim_path_linear(a);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_path_linear_obj, 1, 1, mp_lv_anim_path_linear);

 

/*
 * lvgl extension definition for:
 * int32_t lv_anim_path_ease_in_out(const lv_anim_t *a)
 */
 
STATIC mp_obj_t mp_lv_anim_path_ease_in_out(size_t n_args, const mp_obj_t *args)
{
    const lv_anim_t *a = mp_write_ptr_lv_anim_t(args[0]);
    int32_t res = lv_anim_path_ease_in_out(a);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_path_ease_in_out_obj, 1, 1, mp_lv_anim_path_ease_in_out);

 

/*
 * lvgl extension definition for:
 * int32_t lv_anim_path_step(const lv_anim_t *a)
 */
 
STATIC mp_obj_t mp_lv_anim_path_step(size_t n_args, const mp_obj_t *args)
{
    const lv_anim_t *a = mp_write_ptr_lv_anim_t(args[0]);
    int32_t res = lv_anim_path_step(a);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_anim_path_step_obj, 1, 1, mp_lv_anim_path_step);

 

/*
 * lvgl extension definition for:
 * void lv_style_init(void)
 */
 
STATIC mp_obj_t mp_lv_style_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_style_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_style_init_obj, 0, 0, mp_lv_style_init);

 

/*
 * lvgl extension definition for:
 * void lv_style_copy(lv_style_t *dest, const lv_style_t *src)
 */
 
STATIC mp_obj_t mp_lv_style_copy(size_t n_args, const mp_obj_t *args)
{
    lv_style_t *dest = mp_write_ptr_lv_style_t(args[0]);
    const lv_style_t *src = mp_write_ptr_lv_style_t(args[1]);
    lv_style_copy(dest, src);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_style_copy_obj, 2, 2, mp_lv_style_copy);

 

/*
 * lvgl extension definition for:
 * void lv_style_mix(const lv_style_t *start, const lv_style_t *end, lv_style_t *res, uint16_t ratio)
 */
 
STATIC mp_obj_t mp_lv_style_mix(size_t n_args, const mp_obj_t *args)
{
    const lv_style_t *start = mp_write_ptr_lv_style_t(args[0]);
    const lv_style_t *end = mp_write_ptr_lv_style_t(args[1]);
    lv_style_t *res = mp_write_ptr_lv_style_t(args[2]);
    uint16_t ratio = (uint16_t)mp_obj_int_get_checked(args[3]);
    lv_style_mix(start, end, res, ratio);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_style_mix_obj, 4, 4, mp_lv_style_mix);

 

/*
 * Struct lv_style_anim_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_style_anim_t_type();

STATIC inline lv_style_anim_t* mp_write_ptr_lv_style_anim_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_style_anim_t*)self->data;
}

#define mp_write_lv_style_anim_t(struct_obj) *mp_write_ptr_lv_style_anim_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_style_anim_t(lv_style_anim_t *field)
{
    return lv_to_mp_struct(get_mp_lv_style_anim_t_type(), field);
}

#define mp_read_lv_style_anim_t(field) mp_read_ptr_lv_style_anim_t(&field)

STATIC void mp_lv_style_anim_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_style_anim_t *data = (lv_style_anim_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_style_start: dest[0] = ptr_to_mp((void*)data->style_start); break; // converting from lv_style_t*;
            case MP_QSTR_style_end: dest[0] = ptr_to_mp((void*)data->style_end); break; // converting from lv_style_t*;
            case MP_QSTR_style_anim: dest[0] = ptr_to_mp((void*)data->style_anim); break; // converting from lv_style_t*;
            case MP_QSTR_end_cb: dest[0] = ptr_to_mp(data->end_cb); break; // converting from lv_anim_cb_t;
            case MP_QSTR_time: dest[0] = mp_obj_new_int(data->time); break; // converting from int16_t;
            case MP_QSTR_act_time: dest[0] = mp_obj_new_int(data->act_time); break; // converting from int16_t;
            case MP_QSTR_playback_pause: dest[0] = mp_obj_new_int_from_uint(data->playback_pause); break; // converting from uint16_t;
            case MP_QSTR_repeat_pause: dest[0] = mp_obj_new_int_from_uint(data->repeat_pause); break; // converting from uint16_t;
            case MP_QSTR_playback: dest[0] = mp_obj_new_int_from_uint(data->playback); break; // converting from uint8_t;
            case MP_QSTR_repeat: dest[0] = mp_obj_new_int_from_uint(data->repeat); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_style_anim_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_style_start: data->style_start = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_style_end: data->style_end = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_style_anim: data->style_anim = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_end_cb: data->end_cb = mp_to_ptr(dest[1]); break; // converting to lv_anim_cb_t;
                case MP_QSTR_time: data->time = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to int16_t;
                case MP_QSTR_act_time: data->act_time = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to int16_t;
                case MP_QSTR_playback_pause: data->playback_pause = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_repeat_pause: data->repeat_pause = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                case MP_QSTR_playback: data->playback = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_repeat: data->repeat = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_style_anim_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_style_anim_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_style_anim_t");
}

STATIC const mp_rom_map_elem_t mp_lv_style_anim_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_style_anim_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_style_anim_t_locals_dict, mp_lv_style_anim_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_style_anim_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_anim_t,
    .print = mp_lv_style_anim_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_style_anim_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_anim_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_style_anim_t_type()
{
    return &mp_lv_style_anim_t_type;
}
    

/*
 * lvgl extension definition for:
 * void *lv_style_anim_create(lv_style_anim_t *anim)
 */
 
STATIC mp_obj_t mp_lv_style_anim_create(size_t n_args, const mp_obj_t *args)
{
    lv_style_anim_t *anim = mp_write_ptr_lv_style_anim_t(args[0]);
    void* res = lv_style_anim_create(anim);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_style_anim_create_obj, 1, 1, mp_lv_style_anim_create);

 

/*
 * lvgl extension definition for:
 * void lv_mem_init(void)
 */
 
STATIC mp_obj_t mp_lv_mem_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_mem_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_init_obj, 0, 0, mp_lv_mem_init);

 

/*
 * lvgl extension definition for:
 * void *lv_mem_alloc(uint32_t size)
 */
 
STATIC mp_obj_t mp_lv_mem_alloc(size_t n_args, const mp_obj_t *args)
{
    uint32_t size = (uint32_t)mp_obj_int_get_checked(args[0]);
    void* res = lv_mem_alloc(size);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_alloc_obj, 1, 1, mp_lv_mem_alloc);

 

/*
 * lvgl extension definition for:
 * void lv_mem_free(const void *data)
 */
 
STATIC mp_obj_t mp_lv_mem_free(size_t n_args, const mp_obj_t *args)
{
    const void *data = mp_to_ptr(args[0]);
    lv_mem_free(data);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_free_obj, 1, 1, mp_lv_mem_free);

 

/*
 * lvgl extension definition for:
 * void *lv_mem_realloc(void *data_p, uint32_t new_size)
 */
 
STATIC mp_obj_t mp_lv_mem_realloc(size_t n_args, const mp_obj_t *args)
{
    void *data_p = mp_to_ptr(args[0]);
    uint32_t new_size = (uint32_t)mp_obj_int_get_checked(args[1]);
    void* res = lv_mem_realloc(data_p, new_size);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_realloc_obj, 2, 2, mp_lv_mem_realloc);

 

/*
 * lvgl extension definition for:
 * void lv_mem_defrag(void)
 */
 
STATIC mp_obj_t mp_lv_mem_defrag(size_t n_args, const mp_obj_t *args)
{
    
    lv_mem_defrag();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_defrag_obj, 0, 0, mp_lv_mem_defrag);

 

/*
 * Struct lv_mem_monitor_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_mem_monitor_t_type();

STATIC inline lv_mem_monitor_t* mp_write_ptr_lv_mem_monitor_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_mem_monitor_t*)self->data;
}

#define mp_write_lv_mem_monitor_t(struct_obj) *mp_write_ptr_lv_mem_monitor_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_mem_monitor_t(lv_mem_monitor_t *field)
{
    return lv_to_mp_struct(get_mp_lv_mem_monitor_t_type(), field);
}

#define mp_read_lv_mem_monitor_t(field) mp_read_ptr_lv_mem_monitor_t(&field)

STATIC void mp_lv_mem_monitor_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_mem_monitor_t *data = (lv_mem_monitor_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_total_size: dest[0] = mp_obj_new_int_from_uint(data->total_size); break; // converting from uint32_t;
            case MP_QSTR_free_cnt: dest[0] = mp_obj_new_int_from_uint(data->free_cnt); break; // converting from uint32_t;
            case MP_QSTR_free_size: dest[0] = mp_obj_new_int_from_uint(data->free_size); break; // converting from uint32_t;
            case MP_QSTR_free_biggest_size: dest[0] = mp_obj_new_int_from_uint(data->free_biggest_size); break; // converting from uint32_t;
            case MP_QSTR_used_cnt: dest[0] = mp_obj_new_int_from_uint(data->used_cnt); break; // converting from uint32_t;
            case MP_QSTR_used_pct: dest[0] = mp_obj_new_int_from_uint(data->used_pct); break; // converting from uint8_t;
            case MP_QSTR_frag_pct: dest[0] = mp_obj_new_int_from_uint(data->frag_pct); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_mem_monitor_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_total_size: data->total_size = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_free_cnt: data->free_cnt = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_free_size: data->free_size = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_free_biggest_size: data->free_biggest_size = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_used_cnt: data->used_cnt = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_used_pct: data->used_pct = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_frag_pct: data->frag_pct = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_mem_monitor_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_mem_monitor_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_mem_monitor_t");
}

STATIC const mp_rom_map_elem_t mp_lv_mem_monitor_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_mem_monitor_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_mem_monitor_t_locals_dict, mp_lv_mem_monitor_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_mem_monitor_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_mem_monitor_t,
    .print = mp_lv_mem_monitor_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_mem_monitor_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_mem_monitor_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_mem_monitor_t_type()
{
    return &mp_lv_mem_monitor_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_mem_monitor(lv_mem_monitor_t *mon_p)
 */
 
STATIC mp_obj_t mp_lv_mem_monitor(size_t n_args, const mp_obj_t *args)
{
    lv_mem_monitor_t *mon_p = mp_write_ptr_lv_mem_monitor_t(args[0]);
    lv_mem_monitor(mon_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_monitor_obj, 1, 1, mp_lv_mem_monitor);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_mem_get_size(const void *data)
 */
 
STATIC mp_obj_t mp_lv_mem_get_size(size_t n_args, const mp_obj_t *args)
{
    const void *data = mp_to_ptr(args[0]);
    uint32_t res = lv_mem_get_size(data);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_mem_get_size_obj, 1, 1, mp_lv_mem_get_size);

 

/*
 * Struct lv_ll_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_ll_t_type();

STATIC inline lv_ll_t* mp_write_ptr_lv_ll_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_ll_t*)self->data;
}

#define mp_write_lv_ll_t(struct_obj) *mp_write_ptr_lv_ll_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_ll_t(lv_ll_t *field)
{
    return lv_to_mp_struct(get_mp_lv_ll_t_type(), field);
}

#define mp_read_lv_ll_t(field) mp_read_ptr_lv_ll_t(&field)

STATIC void mp_lv_ll_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_ll_t *data = (lv_ll_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_n_size: dest[0] = mp_obj_new_int_from_uint(data->n_size); break; // converting from uint32_t;
            case MP_QSTR_head: dest[0] = ptr_to_mp((void*)data->head); break; // converting from lv_ll_node_t*;
            case MP_QSTR_tail: dest[0] = ptr_to_mp((void*)data->tail); break; // converting from lv_ll_node_t*;
            default: field_not_found(MP_QSTR_lv_ll_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_n_size: data->n_size = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_head: data->head = (void*)mp_to_ptr(dest[1]); break; // converting to lv_ll_node_t*;
                case MP_QSTR_tail: data->tail = (void*)mp_to_ptr(dest[1]); break; // converting to lv_ll_node_t*;
                default: field_not_found(MP_QSTR_lv_ll_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_ll_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_ll_t");
}

STATIC const mp_rom_map_elem_t mp_lv_ll_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_ll_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_ll_t_locals_dict, mp_lv_ll_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_ll_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_ll_t,
    .print = mp_lv_ll_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_ll_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_ll_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_ll_t_type()
{
    return &mp_lv_ll_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_ll_init(lv_ll_t *ll_p, uint32_t node_size)
 */
 
STATIC mp_obj_t mp_lv_ll_init(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    uint32_t node_size = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_ll_init(ll_p, node_size);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_init_obj, 2, 2, mp_lv_ll_init);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_ins_head(lv_ll_t *ll_p)
 */
 
STATIC mp_obj_t mp_lv_ll_ins_head(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void* res = lv_ll_ins_head(ll_p);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_ins_head_obj, 1, 1, mp_lv_ll_ins_head);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_ins_prev(lv_ll_t *ll_p, void *n_act)
 */
 
STATIC mp_obj_t mp_lv_ll_ins_prev(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void *n_act = mp_to_ptr(args[1]);
    void* res = lv_ll_ins_prev(ll_p, n_act);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_ins_prev_obj, 2, 2, mp_lv_ll_ins_prev);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_ins_tail(lv_ll_t *ll_p)
 */
 
STATIC mp_obj_t mp_lv_ll_ins_tail(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void* res = lv_ll_ins_tail(ll_p);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_ins_tail_obj, 1, 1, mp_lv_ll_ins_tail);

 

/*
 * lvgl extension definition for:
 * void lv_ll_rem(lv_ll_t *ll_p, void *node_p)
 */
 
STATIC mp_obj_t mp_lv_ll_rem(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void *node_p = mp_to_ptr(args[1]);
    lv_ll_rem(ll_p, node_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_rem_obj, 2, 2, mp_lv_ll_rem);

 

/*
 * lvgl extension definition for:
 * void lv_ll_clear(lv_ll_t *ll_p)
 */
 
STATIC mp_obj_t mp_lv_ll_clear(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    lv_ll_clear(ll_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_clear_obj, 1, 1, mp_lv_ll_clear);

 

/*
 * lvgl extension definition for:
 * void lv_ll_chg_list(lv_ll_t *ll_ori_p, lv_ll_t *ll_new_p, void *node)
 */
 
STATIC mp_obj_t mp_lv_ll_chg_list(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_ori_p = mp_write_ptr_lv_ll_t(args[0]);
    lv_ll_t *ll_new_p = mp_write_ptr_lv_ll_t(args[1]);
    void *node = mp_to_ptr(args[2]);
    lv_ll_chg_list(ll_ori_p, ll_new_p, node);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_chg_list_obj, 3, 3, mp_lv_ll_chg_list);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_get_head(const lv_ll_t *ll_p)
 */
 
STATIC mp_obj_t mp_lv_ll_get_head(size_t n_args, const mp_obj_t *args)
{
    const lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void* res = lv_ll_get_head(ll_p);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_get_head_obj, 1, 1, mp_lv_ll_get_head);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_get_tail(const lv_ll_t *ll_p)
 */
 
STATIC mp_obj_t mp_lv_ll_get_tail(size_t n_args, const mp_obj_t *args)
{
    const lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void* res = lv_ll_get_tail(ll_p);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_get_tail_obj, 1, 1, mp_lv_ll_get_tail);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_get_next(const lv_ll_t *ll_p, const void *n_act)
 */
 
STATIC mp_obj_t mp_lv_ll_get_next(size_t n_args, const mp_obj_t *args)
{
    const lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    const void *n_act = mp_to_ptr(args[1]);
    void* res = lv_ll_get_next(ll_p, n_act);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_get_next_obj, 2, 2, mp_lv_ll_get_next);

 

/*
 * lvgl extension definition for:
 * void *lv_ll_get_prev(const lv_ll_t *ll_p, const void *n_act)
 */
 
STATIC mp_obj_t mp_lv_ll_get_prev(size_t n_args, const mp_obj_t *args)
{
    const lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    const void *n_act = mp_to_ptr(args[1]);
    void* res = lv_ll_get_prev(ll_p, n_act);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_get_prev_obj, 2, 2, mp_lv_ll_get_prev);

 

/*
 * lvgl extension definition for:
 * void lv_ll_move_before(lv_ll_t *ll_p, void *n_act, void *n_after)
 */
 
STATIC mp_obj_t mp_lv_ll_move_before(size_t n_args, const mp_obj_t *args)
{
    lv_ll_t *ll_p = mp_write_ptr_lv_ll_t(args[0]);
    void *n_act = mp_to_ptr(args[1]);
    void *n_after = mp_to_ptr(args[2]);
    lv_ll_move_before(ll_p, n_act, n_after);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_ll_move_before_obj, 3, 3, mp_lv_ll_move_before);

 

/*
 * lvgl extension definition for:
 * void lv_init(void)
 */
 
STATIC mp_obj_t mp_lv_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_init_obj, 0, 0, mp_lv_init);

 

/*
 * lvgl extension definition for:
 * void lv_scr_load(lv_obj_t *scr)
 */
 
STATIC mp_obj_t mp_lv_scr_load(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *scr = mp_to_lv(args[0]);
    lv_scr_load(scr);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_scr_load_obj, 1, 1, mp_lv_scr_load);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_scr_act(void)
 */
 
STATIC mp_obj_t mp_lv_scr_act(size_t n_args, const mp_obj_t *args)
{
    
    lv_obj_t* res = lv_scr_act();
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_scr_act_obj, 0, 0, mp_lv_scr_act);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_layer_top(void)
 */
 
STATIC mp_obj_t mp_lv_layer_top(size_t n_args, const mp_obj_t *args)
{
    
    lv_obj_t* res = lv_layer_top();
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_layer_top_obj, 0, 0, mp_lv_layer_top);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_layer_sys(void)
 */
 
STATIC mp_obj_t mp_lv_layer_sys(size_t n_args, const mp_obj_t *args)
{
    
    lv_obj_t* res = lv_layer_sys();
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_layer_sys_obj, 0, 0, mp_lv_layer_sys);

 

/*
 * Struct lv_disp_drv_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_disp_drv_t_type();

STATIC inline lv_disp_drv_t* mp_write_ptr_lv_disp_drv_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_disp_drv_t*)self->data;
}

#define mp_write_lv_disp_drv_t(struct_obj) *mp_write_ptr_lv_disp_drv_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_disp_drv_t(lv_disp_drv_t *field)
{
    return lv_to_mp_struct(get_mp_lv_disp_drv_t_type(), field);
}

#define mp_read_lv_disp_drv_t(field) mp_read_ptr_lv_disp_drv_t(&field)

STATIC void mp_lv_disp_drv_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_disp_drv_t *data = (lv_disp_drv_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_disp_flush: dest[0] = ptr_to_mp((void*)data->disp_flush); break; // converting from void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)*;
            case MP_QSTR_disp_fill: dest[0] = ptr_to_mp((void*)data->disp_fill); break; // converting from void disp_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)*;
            case MP_QSTR_disp_map: dest[0] = ptr_to_mp((void*)data->disp_map); break; // converting from void disp_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)*;
            case MP_QSTR_mem_blend: dest[0] = ptr_to_mp((void*)data->mem_blend); break; // converting from void mem_blend(lv_color_t *dest, const lv_color_t *src, uint32_t length, lv_opa_t opa)*;
            case MP_QSTR_mem_fill: dest[0] = ptr_to_mp((void*)data->mem_fill); break; // converting from void mem_fill(lv_color_t *dest, uint32_t length, lv_color_t color)*;
            case MP_QSTR_vdb_wr: dest[0] = ptr_to_mp((void*)data->vdb_wr); break; // converting from void vdb_wr(uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa)*;
            default: field_not_found(MP_QSTR_lv_disp_drv_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_disp_flush: data->disp_flush = (void*)mp_to_ptr(dest[1]); break; // converting to void disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)*;
                case MP_QSTR_disp_fill: data->disp_fill = (void*)mp_to_ptr(dest[1]); break; // converting to void disp_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)*;
                case MP_QSTR_disp_map: data->disp_map = (void*)mp_to_ptr(dest[1]); break; // converting to void disp_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_p)*;
                case MP_QSTR_mem_blend: data->mem_blend = (void*)mp_to_ptr(dest[1]); break; // converting to void mem_blend(lv_color_t *dest, const lv_color_t *src, uint32_t length, lv_opa_t opa)*;
                case MP_QSTR_mem_fill: data->mem_fill = (void*)mp_to_ptr(dest[1]); break; // converting to void mem_fill(lv_color_t *dest, uint32_t length, lv_color_t color)*;
                case MP_QSTR_vdb_wr: data->vdb_wr = (void*)mp_to_ptr(dest[1]); break; // converting to void vdb_wr(uint8_t *buf, lv_coord_t buf_w, lv_coord_t x, lv_coord_t y, lv_color_t color, lv_opa_t opa)*;
                default: field_not_found(MP_QSTR_lv_disp_drv_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_disp_drv_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_disp_drv_t");
}

STATIC const mp_rom_map_elem_t mp_lv_disp_drv_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_disp_drv_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_disp_drv_t_locals_dict, mp_lv_disp_drv_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_disp_drv_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_disp_drv_t,
    .print = mp_lv_disp_drv_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_disp_drv_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_disp_drv_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_disp_drv_t_type()
{
    return &mp_lv_disp_drv_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_disp_drv_init(lv_disp_drv_t *driver)
 */
 
STATIC mp_obj_t mp_lv_disp_drv_init(size_t n_args, const mp_obj_t *args)
{
    lv_disp_drv_t *driver = mp_write_ptr_lv_disp_drv_t(args[0]);
    lv_disp_drv_init(driver);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_drv_init_obj, 1, 1, mp_lv_disp_drv_init);

 

/*
 * Struct lv_disp_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_disp_t_type();

STATIC inline lv_disp_t* mp_write_ptr_lv_disp_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_disp_t*)self->data;
}

#define mp_write_lv_disp_t(struct_obj) *mp_write_ptr_lv_disp_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_disp_t(lv_disp_t *field)
{
    return lv_to_mp_struct(get_mp_lv_disp_t_type(), field);
}

#define mp_read_lv_disp_t(field) mp_read_ptr_lv_disp_t(&field)

STATIC void mp_lv_disp_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_disp_t *data = (lv_disp_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_driver: dest[0] = mp_read_lv_disp_drv_t(data->driver); break; // converting from lv_disp_drv_t;
            case MP_QSTR_next: dest[0] = ptr_to_mp((void*)data->next); break; // converting from struct _disp_t*;
            default: field_not_found(MP_QSTR_lv_disp_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_driver: data->driver = mp_write_lv_disp_drv_t(dest[1]); break; // converting to lv_disp_drv_t;
                case MP_QSTR_next: data->next = (void*)mp_to_ptr(dest[1]); break; // converting to struct _disp_t*;
                default: field_not_found(MP_QSTR_lv_disp_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_disp_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_disp_t");
}

STATIC const mp_rom_map_elem_t mp_lv_disp_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_disp_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_disp_t_locals_dict, mp_lv_disp_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_disp_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_disp_t,
    .print = mp_lv_disp_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_disp_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_disp_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_disp_t_type()
{
    return &mp_lv_disp_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_disp_t *lv_disp_drv_register(lv_disp_drv_t *driver)
 */
 
STATIC mp_obj_t mp_lv_disp_drv_register(size_t n_args, const mp_obj_t *args)
{
    lv_disp_drv_t *driver = mp_write_ptr_lv_disp_drv_t(args[0]);
    lv_disp_t* res = lv_disp_drv_register(driver);
    return mp_read_ptr_lv_disp_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_drv_register_obj, 1, 1, mp_lv_disp_drv_register);

 

/*
 * lvgl extension definition for:
 * void lv_disp_set_active(lv_disp_t *disp)
 */
 
STATIC mp_obj_t mp_lv_disp_set_active(size_t n_args, const mp_obj_t *args)
{
    lv_disp_t *disp = mp_write_ptr_lv_disp_t(args[0]);
    lv_disp_set_active(disp);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_set_active_obj, 1, 1, mp_lv_disp_set_active);

 

/*
 * lvgl extension definition for:
 * lv_disp_t *lv_disp_get_active(void)
 */
 
STATIC mp_obj_t mp_lv_disp_get_active(size_t n_args, const mp_obj_t *args)
{
    
    lv_disp_t* res = lv_disp_get_active();
    return mp_read_ptr_lv_disp_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_get_active_obj, 0, 0, mp_lv_disp_get_active);

 

/*
 * lvgl extension definition for:
 * lv_disp_t *lv_disp_next(lv_disp_t *disp)
 */
 
STATIC mp_obj_t mp_lv_disp_next(size_t n_args, const mp_obj_t *args)
{
    lv_disp_t *disp = mp_write_ptr_lv_disp_t(args[0]);
    lv_disp_t* res = lv_disp_next(disp);
    return mp_read_ptr_lv_disp_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_next_obj, 1, 1, mp_lv_disp_next);

 

/*
 * lvgl extension definition for:
 * void lv_disp_flush(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t *color_p)
 */
 
STATIC mp_obj_t mp_lv_disp_flush(size_t n_args, const mp_obj_t *args)
{
    int32_t x1 = (int32_t)mp_obj_int_get_checked(args[0]);
    int32_t y1 = (int32_t)mp_obj_int_get_checked(args[1]);
    int32_t x2 = (int32_t)mp_obj_int_get_checked(args[2]);
    int32_t y2 = (int32_t)mp_obj_int_get_checked(args[3]);
    lv_color_t *color_p = mp_write_ptr_lv_color16_t(args[4]);
    lv_disp_flush(x1, y1, x2, y2, color_p);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_flush_obj, 5, 5, mp_lv_disp_flush);

 

/*
 * lvgl extension definition for:
 * void lv_disp_fill(int32_t x1, int32_t y1, int32_t x2, int32_t y2, lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_disp_fill(size_t n_args, const mp_obj_t *args)
{
    int32_t x1 = (int32_t)mp_obj_int_get_checked(args[0]);
    int32_t y1 = (int32_t)mp_obj_int_get_checked(args[1]);
    int32_t x2 = (int32_t)mp_obj_int_get_checked(args[2]);
    int32_t y2 = (int32_t)mp_obj_int_get_checked(args[3]);
    lv_color_t color = mp_write_lv_color16_t(args[4]);
    lv_disp_fill(x1, y1, x2, y2, color);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_fill_obj, 5, 5, mp_lv_disp_fill);

 

/*
 * lvgl extension definition for:
 * void lv_disp_map(int32_t x1, int32_t y1, int32_t x2, int32_t y2, const lv_color_t *color_map)
 */
 
STATIC mp_obj_t mp_lv_disp_map(size_t n_args, const mp_obj_t *args)
{
    int32_t x1 = (int32_t)mp_obj_int_get_checked(args[0]);
    int32_t y1 = (int32_t)mp_obj_int_get_checked(args[1]);
    int32_t x2 = (int32_t)mp_obj_int_get_checked(args[2]);
    int32_t y2 = (int32_t)mp_obj_int_get_checked(args[3]);
    const lv_color_t *color_map = mp_write_ptr_lv_color16_t(args[4]);
    lv_disp_map(x1, y1, x2, y2, color_map);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_map_obj, 5, 5, mp_lv_disp_map);

 

/*
 * lvgl extension definition for:
 * void lv_disp_mem_blend(lv_color_t *dest, const lv_color_t *src, uint32_t length, lv_opa_t opa)
 */
 
STATIC mp_obj_t mp_lv_disp_mem_blend(size_t n_args, const mp_obj_t *args)
{
    lv_color_t *dest = mp_write_ptr_lv_color16_t(args[0]);
    const lv_color_t *src = mp_write_ptr_lv_color16_t(args[1]);
    uint32_t length = (uint32_t)mp_obj_int_get_checked(args[2]);
    lv_opa_t opa = (uint8_t)mp_obj_int_get_checked(args[3]);
    lv_disp_mem_blend(dest, src, length, opa);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_mem_blend_obj, 4, 4, mp_lv_disp_mem_blend);

 

/*
 * lvgl extension definition for:
 * void lv_disp_mem_fill(lv_color_t *dest, uint32_t length, lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_disp_mem_fill(size_t n_args, const mp_obj_t *args)
{
    lv_color_t *dest = mp_write_ptr_lv_color16_t(args[0]);
    uint32_t length = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_color_t color = mp_write_lv_color16_t(args[2]);
    lv_disp_mem_fill(dest, length, color);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_mem_fill_obj, 3, 3, mp_lv_disp_mem_fill);

 

/*
 * lvgl extension definition for:
 * bool lv_disp_is_mem_blend_supported(void)
 */
 
STATIC mp_obj_t mp_lv_disp_is_mem_blend_supported(size_t n_args, const mp_obj_t *args)
{
    
    bool res = lv_disp_is_mem_blend_supported();
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_is_mem_blend_supported_obj, 0, 0, mp_lv_disp_is_mem_blend_supported);

 

/*
 * lvgl extension definition for:
 * bool lv_disp_is_mem_fill_supported(void)
 */
 
STATIC mp_obj_t mp_lv_disp_is_mem_fill_supported(size_t n_args, const mp_obj_t *args)
{
    
    bool res = lv_disp_is_mem_fill_supported();
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_disp_is_mem_fill_supported_obj, 0, 0, mp_lv_disp_is_mem_fill_supported);

 

/*
 * lvgl extension definition for:
 * void lv_tick_inc(uint32_t tick_period)
 */
 
STATIC mp_obj_t mp_lv_tick_inc(size_t n_args, const mp_obj_t *args)
{
    uint32_t tick_period = (uint32_t)mp_obj_int_get_checked(args[0]);
    lv_tick_inc(tick_period);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tick_inc_obj, 1, 1, mp_lv_tick_inc);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_tick_get(void)
 */
 
STATIC mp_obj_t mp_lv_tick_get(size_t n_args, const mp_obj_t *args)
{
    
    uint32_t res = lv_tick_get();
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tick_get_obj, 0, 0, mp_lv_tick_get);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_tick_elaps(uint32_t prev_tick)
 */
 
STATIC mp_obj_t mp_lv_tick_elaps(size_t n_args, const mp_obj_t *args)
{
    uint32_t prev_tick = (uint32_t)mp_obj_int_get_checked(args[0]);
    uint32_t res = lv_tick_elaps(prev_tick);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_tick_elaps_obj, 1, 1, mp_lv_tick_elaps);

 

/*
 * Struct lv_indev_drv_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_indev_drv_t_type();

STATIC inline lv_indev_drv_t* mp_write_ptr_lv_indev_drv_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_indev_drv_t*)self->data;
}

#define mp_write_lv_indev_drv_t(struct_obj) *mp_write_ptr_lv_indev_drv_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_indev_drv_t(lv_indev_drv_t *field)
{
    return lv_to_mp_struct(get_mp_lv_indev_drv_t_type(), field);
}

#define mp_read_lv_indev_drv_t(field) mp_read_ptr_lv_indev_drv_t(&field)

STATIC void mp_lv_indev_drv_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_indev_drv_t *data = (lv_indev_drv_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_type: dest[0] = mp_obj_new_int_from_uint(data->type); break; // converting from lv_hal_indev_type_t;
            case MP_QSTR_read: dest[0] = ptr_to_mp((void*)data->read); break; // converting from bool read(lv_indev_data_t *data)*;
            case MP_QSTR_user_data: dest[0] = ptr_to_mp((void*)data->user_data); break; // converting from void*;
            default: field_not_found(MP_QSTR_lv_indev_drv_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_type: data->type = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_hal_indev_type_t;
                case MP_QSTR_read: data->read = (void*)mp_to_ptr(dest[1]); break; // converting to bool read(lv_indev_data_t *data)*;
                case MP_QSTR_user_data: data->user_data = (void*)mp_to_ptr(dest[1]); break; // converting to void*;
                default: field_not_found(MP_QSTR_lv_indev_drv_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_indev_drv_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_indev_drv_t");
}

STATIC const mp_rom_map_elem_t mp_lv_indev_drv_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_indev_drv_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_indev_drv_t_locals_dict, mp_lv_indev_drv_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_indev_drv_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_indev_drv_t,
    .print = mp_lv_indev_drv_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_indev_drv_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_indev_drv_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_indev_drv_t_type()
{
    return &mp_lv_indev_drv_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_indev_drv_init(lv_indev_drv_t *driver)
 */
 
STATIC mp_obj_t mp_lv_indev_drv_init(size_t n_args, const mp_obj_t *args)
{
    lv_indev_drv_t *driver = mp_write_ptr_lv_indev_drv_t(args[0]);
    lv_indev_drv_init(driver);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_drv_init_obj, 1, 1, mp_lv_indev_drv_init);

 

/*
 * Struct lv_indev_proc_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_indev_proc_t_type();

STATIC inline lv_indev_proc_t* mp_write_ptr_lv_indev_proc_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_indev_proc_t*)self->data;
}

#define mp_write_lv_indev_proc_t(struct_obj) *mp_write_ptr_lv_indev_proc_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_indev_proc_t(lv_indev_proc_t *field)
{
    return lv_to_mp_struct(get_mp_lv_indev_proc_t_type(), field);
}

#define mp_read_lv_indev_proc_t(field) mp_read_ptr_lv_indev_proc_t(&field)

STATIC void mp_lv_indev_proc_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_indev_proc_t *data = (lv_indev_proc_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_state: dest[0] = mp_obj_new_int_from_uint(data->state); break; // converting from lv_indev_state_t;
            case MP_QSTR_act_point: dest[0] = mp_read_lv_point_t(data->act_point); break; // converting from lv_point_t;
            case MP_QSTR_last_point: dest[0] = mp_read_lv_point_t(data->last_point); break; // converting from lv_point_t;
            case MP_QSTR_vect: dest[0] = mp_read_lv_point_t(data->vect); break; // converting from lv_point_t;
            case MP_QSTR_drag_sum: dest[0] = mp_read_lv_point_t(data->drag_sum); break; // converting from lv_point_t;
            case MP_QSTR_act_obj: dest[0] = ptr_to_mp((void*)data->act_obj); break; // converting from struct _lv_obj_t*;
            case MP_QSTR_last_obj: dest[0] = ptr_to_mp((void*)data->last_obj); break; // converting from struct _lv_obj_t*;
            case MP_QSTR_drag_range_out: dest[0] = mp_obj_new_int_from_uint(data->drag_range_out); break; // converting from uint8_t;
            case MP_QSTR_drag_in_prog: dest[0] = mp_obj_new_int_from_uint(data->drag_in_prog); break; // converting from uint8_t;
            case MP_QSTR_wait_unil_release: dest[0] = mp_obj_new_int_from_uint(data->wait_unil_release); break; // converting from uint8_t;
            case MP_QSTR_last_state: dest[0] = mp_obj_new_int_from_uint(data->last_state); break; // converting from lv_indev_state_t;
            case MP_QSTR_last_key: dest[0] = mp_obj_new_int_from_uint(data->last_key); break; // converting from uint32_t;
            case MP_QSTR_pr_timestamp: dest[0] = mp_obj_new_int_from_uint(data->pr_timestamp); break; // converting from uint32_t;
            case MP_QSTR_longpr_rep_timestamp: dest[0] = mp_obj_new_int_from_uint(data->longpr_rep_timestamp); break; // converting from uint32_t;
            case MP_QSTR_long_pr_sent: dest[0] = mp_obj_new_int_from_uint(data->long_pr_sent); break; // converting from uint8_t;
            case MP_QSTR_reset_query: dest[0] = mp_obj_new_int_from_uint(data->reset_query); break; // converting from uint8_t;
            case MP_QSTR_disabled: dest[0] = mp_obj_new_int_from_uint(data->disabled); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_indev_proc_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_state: data->state = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_indev_state_t;
                case MP_QSTR_act_point: data->act_point = mp_write_lv_point_t(dest[1]); break; // converting to lv_point_t;
                case MP_QSTR_last_point: data->last_point = mp_write_lv_point_t(dest[1]); break; // converting to lv_point_t;
                case MP_QSTR_vect: data->vect = mp_write_lv_point_t(dest[1]); break; // converting to lv_point_t;
                case MP_QSTR_drag_sum: data->drag_sum = mp_write_lv_point_t(dest[1]); break; // converting to lv_point_t;
                case MP_QSTR_act_obj: data->act_obj = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_obj_t*;
                case MP_QSTR_last_obj: data->last_obj = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_obj_t*;
                case MP_QSTR_drag_range_out: data->drag_range_out = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_drag_in_prog: data->drag_in_prog = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_wait_unil_release: data->wait_unil_release = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_last_state: data->last_state = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_indev_state_t;
                case MP_QSTR_last_key: data->last_key = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_pr_timestamp: data->pr_timestamp = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_longpr_rep_timestamp: data->longpr_rep_timestamp = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_long_pr_sent: data->long_pr_sent = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_reset_query: data->reset_query = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_disabled: data->disabled = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_indev_proc_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_indev_proc_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_indev_proc_t");
}

STATIC const mp_rom_map_elem_t mp_lv_indev_proc_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_indev_proc_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_indev_proc_t_locals_dict, mp_lv_indev_proc_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_indev_proc_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_indev_proc_t,
    .print = mp_lv_indev_proc_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_indev_proc_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_indev_proc_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_indev_proc_t_type()
{
    return &mp_lv_indev_proc_t_type;
}
    

/*
 * Struct lv_indev_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_indev_t_type();

STATIC inline lv_indev_t* mp_write_ptr_lv_indev_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_indev_t*)self->data;
}

#define mp_write_lv_indev_t(struct_obj) *mp_write_ptr_lv_indev_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_indev_t(lv_indev_t *field)
{
    return lv_to_mp_struct(get_mp_lv_indev_t_type(), field);
}

#define mp_read_lv_indev_t(field) mp_read_ptr_lv_indev_t(&field)

STATIC void mp_lv_indev_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_indev_t *data = (lv_indev_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_driver: dest[0] = mp_read_lv_indev_drv_t(data->driver); break; // converting from lv_indev_drv_t;
            case MP_QSTR_proc: dest[0] = mp_read_lv_indev_proc_t(data->proc); break; // converting from lv_indev_proc_t;
            case MP_QSTR_last_activity_time: dest[0] = mp_obj_new_int_from_uint(data->last_activity_time); break; // converting from uint32_t;
            case MP_QSTR_cursor: dest[0] = ptr_to_mp((void*)data->cursor); break; // converting from struct _lv_obj_t*;
            case MP_QSTR_group: dest[0] = ptr_to_mp((void*)data->group); break; // converting from struct _lv_group_t*;
            case MP_QSTR_btn_points: dest[0] = ptr_to_mp((void*)data->btn_points); break; // converting from lv_point_t*;
            case MP_QSTR_next: dest[0] = ptr_to_mp((void*)data->next); break; // converting from struct _lv_indev_t*;
            default: field_not_found(MP_QSTR_lv_indev_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_driver: data->driver = mp_write_lv_indev_drv_t(dest[1]); break; // converting to lv_indev_drv_t;
                case MP_QSTR_proc: data->proc = mp_write_lv_indev_proc_t(dest[1]); break; // converting to lv_indev_proc_t;
                case MP_QSTR_last_activity_time: data->last_activity_time = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_cursor: data->cursor = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_obj_t*;
                case MP_QSTR_group: data->group = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_group_t*;
                case MP_QSTR_btn_points: data->btn_points = (void*)mp_to_ptr(dest[1]); break; // converting to lv_point_t*;
                case MP_QSTR_next: data->next = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_indev_t*;
                default: field_not_found(MP_QSTR_lv_indev_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_indev_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_indev_t");
}

STATIC const mp_rom_map_elem_t mp_lv_indev_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_indev_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_indev_t_locals_dict, mp_lv_indev_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_indev_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_indev_t,
    .print = mp_lv_indev_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_indev_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_indev_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_indev_t_type()
{
    return &mp_lv_indev_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_indev_t *lv_indev_drv_register(lv_indev_drv_t *driver)
 */
 
STATIC mp_obj_t mp_lv_indev_drv_register(size_t n_args, const mp_obj_t *args)
{
    lv_indev_drv_t *driver = mp_write_ptr_lv_indev_drv_t(args[0]);
    lv_indev_t* res = lv_indev_drv_register(driver);
    return mp_read_ptr_lv_indev_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_drv_register_obj, 1, 1, mp_lv_indev_drv_register);

 

/*
 * lvgl extension definition for:
 * lv_indev_t *lv_indev_next(lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_next(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_indev_t* res = lv_indev_next(indev);
    return mp_read_ptr_lv_indev_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_next_obj, 1, 1, mp_lv_indev_next);

 

/*
 * Struct lv_indev_data_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_indev_data_t_type();

STATIC inline lv_indev_data_t* mp_write_ptr_lv_indev_data_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_indev_data_t*)self->data;
}

#define mp_write_lv_indev_data_t(struct_obj) *mp_write_ptr_lv_indev_data_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_indev_data_t(lv_indev_data_t *field)
{
    return lv_to_mp_struct(get_mp_lv_indev_data_t_type(), field);
}

#define mp_read_lv_indev_data_t(field) mp_read_ptr_lv_indev_data_t(&field)

STATIC void mp_lv_indev_data_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_indev_data_t *data = (lv_indev_data_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_point: dest[0] = mp_read_lv_point_t(data->point); break; // converting from lv_point_t;
            case MP_QSTR_key: dest[0] = mp_obj_new_int_from_uint(data->key); break; // converting from uint32_t;
            case MP_QSTR_btn: dest[0] = mp_obj_new_int_from_uint(data->btn); break; // converting from uint32_t;
            case MP_QSTR_enc_diff: dest[0] = mp_obj_new_int(data->enc_diff); break; // converting from int16_t;
            case MP_QSTR_user_data: dest[0] = ptr_to_mp((void*)data->user_data); break; // converting from void*;
            case MP_QSTR_state: dest[0] = mp_obj_new_int_from_uint(data->state); break; // converting from lv_indev_state_t;
            default: field_not_found(MP_QSTR_lv_indev_data_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_point: data->point = mp_write_lv_point_t(dest[1]); break; // converting to lv_point_t;
                case MP_QSTR_key: data->key = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_btn: data->btn = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_enc_diff: data->enc_diff = (int16_t)mp_obj_int_get_checked(dest[1]); break; // converting to int16_t;
                case MP_QSTR_user_data: data->user_data = (void*)mp_to_ptr(dest[1]); break; // converting to void*;
                case MP_QSTR_state: data->state = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to lv_indev_state_t;
                default: field_not_found(MP_QSTR_lv_indev_data_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_indev_data_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_indev_data_t");
}

STATIC const mp_rom_map_elem_t mp_lv_indev_data_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_indev_data_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_indev_data_t_locals_dict, mp_lv_indev_data_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_indev_data_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_indev_data_t,
    .print = mp_lv_indev_data_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_indev_data_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_indev_data_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_indev_data_t_type()
{
    return &mp_lv_indev_data_t_type;
}
    

/*
 * lvgl extension definition for:
 * bool lv_indev_read(lv_indev_t *indev, lv_indev_data_t *data)
 */
 
STATIC mp_obj_t mp_lv_indev_read(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_indev_data_t *data = mp_write_ptr_lv_indev_data_t(args[1]);
    bool res = lv_indev_read(indev, data);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_read_obj, 2, 2, mp_lv_indev_read);

 

/*
 * Struct lv_group_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_group_t_type();

STATIC inline lv_group_t* mp_write_ptr_lv_group_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_group_t*)self->data;
}

#define mp_write_lv_group_t(struct_obj) *mp_write_ptr_lv_group_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_group_t(lv_group_t *field)
{
    return lv_to_mp_struct(get_mp_lv_group_t_type(), field);
}

#define mp_read_lv_group_t(field) mp_read_ptr_lv_group_t(&field)

STATIC void mp_lv_group_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_group_t *data = (lv_group_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_obj_ll: dest[0] = mp_read_lv_ll_t(data->obj_ll); break; // converting from lv_ll_t;
            case MP_QSTR_obj_focus: dest[0] = ptr_to_mp((void*)data->obj_focus); break; // converting from lv_obj_t**;
            case MP_QSTR_style_mod: dest[0] = ptr_to_mp(data->style_mod); break; // converting from lv_group_style_mod_func_t;
            case MP_QSTR_style_mod_edit: dest[0] = ptr_to_mp(data->style_mod_edit); break; // converting from lv_group_style_mod_func_t;
            case MP_QSTR_focus_cb: dest[0] = ptr_to_mp(data->focus_cb); break; // converting from lv_group_focus_cb_t;
            case MP_QSTR_style_tmp: dest[0] = mp_read_lv_style_t(data->style_tmp); break; // converting from lv_style_t;
            case MP_QSTR_frozen: dest[0] = mp_obj_new_int_from_uint(data->frozen); break; // converting from uint8_t;
            case MP_QSTR_editing: dest[0] = mp_obj_new_int_from_uint(data->editing); break; // converting from uint8_t;
            case MP_QSTR_click_focus: dest[0] = mp_obj_new_int_from_uint(data->click_focus); break; // converting from uint8_t;
            default: field_not_found(MP_QSTR_lv_group_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_obj_ll: data->obj_ll = mp_write_lv_ll_t(dest[1]); break; // converting to lv_ll_t;
                case MP_QSTR_obj_focus: data->obj_focus = (void*)mp_to_ptr(dest[1]); break; // converting to lv_obj_t**;
                case MP_QSTR_style_mod: data->style_mod = mp_to_ptr(dest[1]); break; // converting to lv_group_style_mod_func_t;
                case MP_QSTR_style_mod_edit: data->style_mod_edit = mp_to_ptr(dest[1]); break; // converting to lv_group_style_mod_func_t;
                case MP_QSTR_focus_cb: data->focus_cb = mp_to_ptr(dest[1]); break; // converting to lv_group_focus_cb_t;
                case MP_QSTR_style_tmp: data->style_tmp = mp_write_lv_style_t(dest[1]); break; // converting to lv_style_t;
                case MP_QSTR_frozen: data->frozen = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_editing: data->editing = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                case MP_QSTR_click_focus: data->click_focus = (uint8_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint8_t;
                default: field_not_found(MP_QSTR_lv_group_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_group_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_group_t");
}

STATIC const mp_rom_map_elem_t mp_lv_group_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_group_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_group_t_locals_dict, mp_lv_group_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_group_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_group_t,
    .print = mp_lv_group_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_group_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_group_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_group_t_type()
{
    return &mp_lv_group_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_group_t *lv_group_create(void)
 */
 
STATIC mp_obj_t mp_lv_group_create(size_t n_args, const mp_obj_t *args)
{
    
    lv_group_t* res = lv_group_create();
    return mp_read_ptr_lv_group_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_create_obj, 0, 0, mp_lv_group_create);

 

/*
 * lvgl extension definition for:
 * void lv_group_del(lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_del(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_del(group);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_del_obj, 1, 1, mp_lv_group_del);

 

/*
 * lvgl extension definition for:
 * void lv_group_add_obj(lv_group_t *group, lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_group_add_obj(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_obj_t *obj = mp_to_lv(args[1]);
    lv_group_add_obj(group, obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_add_obj_obj, 2, 2, mp_lv_group_add_obj);

 

/*
 * lvgl extension definition for:
 * void lv_group_remove_obj(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_group_remove_obj(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_group_remove_obj(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_remove_obj_obj, 1, 1, mp_lv_group_remove_obj);

 

/*
 * lvgl extension definition for:
 * void lv_group_focus_obj(lv_obj_t *obj)
 */
 
STATIC mp_obj_t mp_lv_group_focus_obj(size_t n_args, const mp_obj_t *args)
{
    lv_obj_t *obj = mp_to_lv(args[0]);
    lv_group_focus_obj(obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_focus_obj_obj, 1, 1, mp_lv_group_focus_obj);

 

/*
 * lvgl extension definition for:
 * void lv_group_focus_next(lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_focus_next(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_focus_next(group);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_focus_next_obj, 1, 1, mp_lv_group_focus_next);

 

/*
 * lvgl extension definition for:
 * void lv_group_focus_prev(lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_focus_prev(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_focus_prev(group);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_focus_prev_obj, 1, 1, mp_lv_group_focus_prev);

 

/*
 * lvgl extension definition for:
 * void lv_group_focus_freeze(lv_group_t *group, bool en)
 */
 
STATIC mp_obj_t mp_lv_group_focus_freeze(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_group_focus_freeze(group, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_focus_freeze_obj, 2, 2, mp_lv_group_focus_freeze);

 

/*
 * lvgl extension definition for:
 * void lv_group_send_data(lv_group_t *group, uint32_t c)
 */
 
STATIC mp_obj_t mp_lv_group_send_data(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    uint32_t c = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_group_send_data(group, c);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_send_data_obj, 2, 2, mp_lv_group_send_data);

 

/*
 * lvgl extension definition for:
 * void lv_group_set_style_mod_cb(lv_group_t *group, lv_group_style_mod_func_t style_mod_func)
 */
 
STATIC mp_obj_t mp_lv_group_set_style_mod_cb(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    set_action(args[0], args[1]);
    lv_group_style_mod_func_t style_mod_func = &lv_group_style_mod_func_t_callback;
    lv_group_set_style_mod_cb(group, style_mod_func);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_set_style_mod_cb_obj, 2, 2, mp_lv_group_set_style_mod_cb);

 

/*
 * lvgl extension definition for:
 * void lv_group_set_style_mod_edit_cb(lv_group_t *group, lv_group_style_mod_func_t style_mod_func)
 */
 
STATIC mp_obj_t mp_lv_group_set_style_mod_edit_cb(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    set_action(args[0], args[1]);
    lv_group_style_mod_func_t style_mod_func = &lv_group_style_mod_func_t_callback;
    lv_group_set_style_mod_edit_cb(group, style_mod_func);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_set_style_mod_edit_cb_obj, 2, 2, mp_lv_group_set_style_mod_edit_cb);

 

/*
 * lvgl extension definition for:
 * void lv_group_set_focus_cb(lv_group_t *group, lv_group_focus_cb_t focus_cb)
 */
 
STATIC mp_obj_t mp_lv_group_set_focus_cb(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_focus_cb_t focus_cb = mp_to_ptr(args[1]);
    lv_group_set_focus_cb(group, focus_cb);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_set_focus_cb_obj, 2, 2, mp_lv_group_set_focus_cb);

 

/*
 * lvgl extension definition for:
 * void lv_group_set_editing(lv_group_t *group, bool edit)
 */
 
STATIC mp_obj_t mp_lv_group_set_editing(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    bool edit = mp_obj_is_true(args[1]);
    lv_group_set_editing(group, edit);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_set_editing_obj, 2, 2, mp_lv_group_set_editing);

 

/*
 * lvgl extension definition for:
 * void lv_group_set_click_focus(lv_group_t *group, bool en)
 */
 
STATIC mp_obj_t mp_lv_group_set_click_focus(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    bool en = mp_obj_is_true(args[1]);
    lv_group_set_click_focus(group, en);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_set_click_focus_obj, 2, 2, mp_lv_group_set_click_focus);

 

/*
 * lvgl extension definition for:
 * lv_style_t *lv_group_mod_style(lv_group_t *group, const lv_style_t *style)
 */
 
STATIC mp_obj_t mp_lv_group_mod_style(size_t n_args, const mp_obj_t *args)
{
    lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    const lv_style_t *style = mp_to_ptr(args[1]);
    lv_style_t* res = lv_group_mod_style(group, style);
    return ptr_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_mod_style_obj, 2, 2, mp_lv_group_mod_style);

 

/*
 * lvgl extension definition for:
 * lv_obj_t *lv_group_get_focused(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_focused(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_obj_t* res = lv_group_get_focused(group);
    return lv_to_mp((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_focused_obj, 1, 1, mp_lv_group_get_focused);

 

/*
 * lvgl extension definition for:
 * lv_group_style_mod_func_t lv_group_get_style_mod_cb(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_style_mod_cb(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_style_mod_func_t res = lv_group_get_style_mod_cb(group);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_style_mod_cb_obj, 1, 1, mp_lv_group_get_style_mod_cb);

 

/*
 * lvgl extension definition for:
 * lv_group_style_mod_func_t lv_group_get_style_mod_edit_cb(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_style_mod_edit_cb(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_style_mod_func_t res = lv_group_get_style_mod_edit_cb(group);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_style_mod_edit_cb_obj, 1, 1, mp_lv_group_get_style_mod_edit_cb);

 

/*
 * lvgl extension definition for:
 * lv_group_focus_cb_t lv_group_get_focus_cb(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_focus_cb(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    lv_group_focus_cb_t res = lv_group_get_focus_cb(group);
    return ptr_to_mp(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_focus_cb_obj, 1, 1, mp_lv_group_get_focus_cb);

 

/*
 * lvgl extension definition for:
 * bool lv_group_get_editing(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_editing(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    bool res = lv_group_get_editing(group);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_editing_obj, 1, 1, mp_lv_group_get_editing);

 

/*
 * lvgl extension definition for:
 * bool lv_group_get_click_focus(const lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_group_get_click_focus(size_t n_args, const mp_obj_t *args)
{
    const lv_group_t *group = mp_write_ptr_lv_group_t(args[0]);
    bool res = lv_group_get_click_focus(group);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_group_get_click_focus_obj, 1, 1, mp_lv_group_get_click_focus);

 

/*
 * lvgl extension definition for:
 * void lv_indev_init(void)
 */
 
STATIC mp_obj_t mp_lv_indev_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_indev_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_init_obj, 0, 0, mp_lv_indev_init);

 

/*
 * lvgl extension definition for:
 * lv_indev_t *lv_indev_get_act(void)
 */
 
STATIC mp_obj_t mp_lv_indev_get_act(size_t n_args, const mp_obj_t *args)
{
    
    lv_indev_t* res = lv_indev_get_act();
    return mp_read_ptr_lv_indev_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_act_obj, 0, 0, mp_lv_indev_get_act);

 

/*
 * lvgl extension definition for:
 * lv_hal_indev_type_t lv_indev_get_type(const lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_get_type(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_hal_indev_type_t res = lv_indev_get_type(indev);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_type_obj, 1, 1, mp_lv_indev_get_type);

 

/*
 * lvgl extension definition for:
 * void lv_indev_reset(lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_reset(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_indev_reset(indev);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_reset_obj, 1, 1, mp_lv_indev_reset);

 

/*
 * lvgl extension definition for:
 * void lv_indev_reset_lpr(lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_reset_lpr(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_indev_reset_lpr(indev);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_reset_lpr_obj, 1, 1, mp_lv_indev_reset_lpr);

 

/*
 * lvgl extension definition for:
 * void lv_indev_enable(lv_hal_indev_type_t type, bool enable)
 */
 
STATIC mp_obj_t mp_lv_indev_enable(size_t n_args, const mp_obj_t *args)
{
    lv_hal_indev_type_t type = (uint8_t)mp_obj_int_get_checked(args[0]);
    bool enable = mp_obj_is_true(args[1]);
    lv_indev_enable(type, enable);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_enable_obj, 2, 2, mp_lv_indev_enable);

 

/*
 * lvgl extension definition for:
 * void lv_indev_set_cursor(lv_indev_t *indev, lv_obj_t *cur_obj)
 */
 
STATIC mp_obj_t mp_lv_indev_set_cursor(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_obj_t *cur_obj = mp_to_lv(args[1]);
    lv_indev_set_cursor(indev, cur_obj);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_set_cursor_obj, 2, 2, mp_lv_indev_set_cursor);

 

/*
 * lvgl extension definition for:
 * void lv_indev_set_group(lv_indev_t *indev, lv_group_t *group)
 */
 
STATIC mp_obj_t mp_lv_indev_set_group(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_group_t *group = mp_write_ptr_lv_group_t(args[1]);
    lv_indev_set_group(indev, group);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_set_group_obj, 2, 2, mp_lv_indev_set_group);

 

/*
 * lvgl extension definition for:
 * void lv_indev_set_button_points(lv_indev_t *indev, lv_point_t *points)
 */
 
STATIC mp_obj_t mp_lv_indev_set_button_points(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_point_t *points = mp_to_ptr(args[1]);
    lv_indev_set_button_points(indev, points);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_set_button_points_obj, 2, 2, mp_lv_indev_set_button_points);

 

/*
 * lvgl extension definition for:
 * void lv_indev_get_point(const lv_indev_t *indev, lv_point_t *point)
 */
 
STATIC mp_obj_t mp_lv_indev_get_point(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_point_t *point = mp_to_ptr(args[1]);
    lv_indev_get_point(indev, point);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_point_obj, 2, 2, mp_lv_indev_get_point);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_indev_get_key(const lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_get_key(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    uint32_t res = lv_indev_get_key(indev);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_key_obj, 1, 1, mp_lv_indev_get_key);

 

/*
 * lvgl extension definition for:
 * bool lv_indev_is_dragging(const lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_is_dragging(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    bool res = lv_indev_is_dragging(indev);
    return convert_to_bool(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_is_dragging_obj, 1, 1, mp_lv_indev_is_dragging);

 

/*
 * lvgl extension definition for:
 * void lv_indev_get_vect(const lv_indev_t *indev, lv_point_t *point)
 */
 
STATIC mp_obj_t mp_lv_indev_get_vect(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_point_t *point = mp_to_ptr(args[1]);
    lv_indev_get_vect(indev, point);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_vect_obj, 2, 2, mp_lv_indev_get_vect);

 

/*
 * lvgl extension definition for:
 * uint32_t lv_indev_get_inactive_time(const lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_get_inactive_time(size_t n_args, const mp_obj_t *args)
{
    const lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    uint32_t res = lv_indev_get_inactive_time(indev);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_get_inactive_time_obj, 1, 1, mp_lv_indev_get_inactive_time);

 

/*
 * lvgl extension definition for:
 * void lv_indev_wait_release(lv_indev_t *indev)
 */
 
STATIC mp_obj_t mp_lv_indev_wait_release(size_t n_args, const mp_obj_t *args)
{
    lv_indev_t *indev = mp_write_ptr_lv_indev_t(args[0]);
    lv_indev_wait_release(indev);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_indev_wait_release_obj, 1, 1, mp_lv_indev_wait_release);

 

/*
 * lvgl extension definition for:
 * void lv_txt_get_size(lv_point_t *size_res, const char *text, const lv_font_t *font, lv_coord_t letter_space, lv_coord_t line_space, lv_coord_t max_width, lv_txt_flag_t flag)
 */
 
STATIC mp_obj_t mp_lv_txt_get_size(size_t n_args, const mp_obj_t *args)
{
    lv_point_t *size_res = mp_to_ptr(args[0]);
    const char *text = (char*)mp_obj_str_get_str(args[1]);
    const lv_font_t *font = mp_to_ptr(args[2]);
    lv_coord_t letter_space = (int16_t)mp_obj_int_get_checked(args[3]);
    lv_coord_t line_space = (int16_t)mp_obj_int_get_checked(args[4]);
    lv_coord_t max_width = (int16_t)mp_obj_int_get_checked(args[5]);
    lv_txt_flag_t flag = (uint8_t)mp_obj_int_get_checked(args[6]);
    lv_txt_get_size(size_res, text, font, letter_space, line_space, max_width, flag);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_txt_get_size_obj, 7, 7, mp_lv_txt_get_size);

 

/*
 * lvgl extension definition for:
 * uint16_t lv_txt_get_next_line(const char *txt, const lv_font_t *font_p, lv_coord_t letter_space, lv_coord_t max_l, lv_txt_flag_t flag)
 */
 
STATIC mp_obj_t mp_lv_txt_get_next_line(size_t n_args, const mp_obj_t *args)
{
    const char *txt = (char*)mp_obj_str_get_str(args[0]);
    const lv_font_t *font_p = mp_to_ptr(args[1]);
    lv_coord_t letter_space = (int16_t)mp_obj_int_get_checked(args[2]);
    lv_coord_t max_l = (int16_t)mp_obj_int_get_checked(args[3]);
    lv_txt_flag_t flag = (uint8_t)mp_obj_int_get_checked(args[4]);
    uint16_t res = lv_txt_get_next_line(txt, font_p, letter_space, max_l, flag);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_txt_get_next_line_obj, 5, 5, mp_lv_txt_get_next_line);

 

/*
 * lvgl extension definition for:
 * lv_coord_t lv_txt_get_width(const char *txt, uint16_t length, const lv_font_t *font, lv_coord_t letter_space, lv_txt_flag_t flag)
 */
 
STATIC mp_obj_t mp_lv_txt_get_width(size_t n_args, const mp_obj_t *args)
{
    const char *txt = (char*)mp_obj_str_get_str(args[0]);
    uint16_t length = (uint16_t)mp_obj_int_get_checked(args[1]);
    const lv_font_t *font = mp_to_ptr(args[2]);
    lv_coord_t letter_space = (int16_t)mp_obj_int_get_checked(args[3]);
    lv_txt_flag_t flag = (uint8_t)mp_obj_int_get_checked(args[4]);
    lv_coord_t res = lv_txt_get_width(txt, length, font, letter_space, flag);
    return mp_obj_new_int(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_txt_get_width_obj, 5, 5, mp_lv_txt_get_width);

 

/*
 * Function NOT generated:
 * Missing conversion to lv_txt_cmd_state_t*
 * bool lv_txt_is_cmd(lv_txt_cmd_state_t *state, uint32_t c)
 */
    

/*
 * lvgl extension definition for:
 * void lv_txt_ins(char *txt_buf, uint32_t pos, const char *ins_txt)
 */
 
STATIC mp_obj_t mp_lv_txt_ins(size_t n_args, const mp_obj_t *args)
{
    char *txt_buf = (char*)mp_obj_str_get_str(args[0]);
    uint32_t pos = (uint32_t)mp_obj_int_get_checked(args[1]);
    const char *ins_txt = (char*)mp_obj_str_get_str(args[2]);
    lv_txt_ins(txt_buf, pos, ins_txt);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_txt_ins_obj, 3, 3, mp_lv_txt_ins);

 

/*
 * lvgl extension definition for:
 * void lv_txt_cut(char *txt, uint32_t pos, uint32_t len)
 */
 
STATIC mp_obj_t mp_lv_txt_cut(size_t n_args, const mp_obj_t *args)
{
    char *txt = (char*)mp_obj_str_get_str(args[0]);
    uint32_t pos = (uint32_t)mp_obj_int_get_checked(args[1]);
    uint32_t len = (uint32_t)mp_obj_int_get_checked(args[2]);
    lv_txt_cut(txt, pos, len);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_txt_cut_obj, 3, 3, mp_lv_txt_cut);

 

/*
 * lvgl extension definition for:
 * void lv_fs_init(void)
 */
 
STATIC mp_obj_t mp_lv_fs_init(size_t n_args, const mp_obj_t *args)
{
    
    lv_fs_init();
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_init_obj, 0, 0, mp_lv_fs_init);

 

/*
 * Function NOT generated:
 * Missing conversion to char when generating struct lv_fs_drv_t.letter
 * void lv_fs_add_drv(lv_fs_drv_t *drv_p)
 */
    

/*
 * Struct lv_fs_file_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_fs_file_t_type();

STATIC inline lv_fs_file_t* mp_write_ptr_lv_fs_file_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_fs_file_t*)self->data;
}

#define mp_write_lv_fs_file_t(struct_obj) *mp_write_ptr_lv_fs_file_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_fs_file_t(lv_fs_file_t *field)
{
    return lv_to_mp_struct(get_mp_lv_fs_file_t_type(), field);
}

#define mp_read_lv_fs_file_t(field) mp_read_ptr_lv_fs_file_t(&field)

STATIC void mp_lv_fs_file_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_fs_file_t *data = (lv_fs_file_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_file_d: dest[0] = ptr_to_mp((void*)data->file_d); break; // converting from void*;
            case MP_QSTR_drv: dest[0] = ptr_to_mp((void*)data->drv); break; // converting from struct __lv_fs_drv_t*;
            default: field_not_found(MP_QSTR_lv_fs_file_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_file_d: data->file_d = (void*)mp_to_ptr(dest[1]); break; // converting to void*;
                case MP_QSTR_drv: data->drv = (void*)mp_to_ptr(dest[1]); break; // converting to struct __lv_fs_drv_t*;
                default: field_not_found(MP_QSTR_lv_fs_file_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_fs_file_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_fs_file_t");
}

STATIC const mp_rom_map_elem_t mp_lv_fs_file_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_fs_file_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_fs_file_t_locals_dict, mp_lv_fs_file_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_fs_file_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_fs_file_t,
    .print = mp_lv_fs_file_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_fs_file_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_fs_file_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_fs_file_t_type()
{
    return &mp_lv_fs_file_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_open(lv_fs_file_t *file_p, const char *path, lv_fs_mode_t mode)
 */
 
STATIC mp_obj_t mp_lv_fs_open(size_t n_args, const mp_obj_t *args)
{
    lv_fs_file_t *file_p = mp_write_ptr_lv_fs_file_t(args[0]);
    const char *path = (char*)mp_obj_str_get_str(args[1]);
    lv_fs_mode_t mode = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_fs_res_t res = lv_fs_open(file_p, path, mode);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_open_obj, 3, 3, mp_lv_fs_open);

 

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_close(lv_fs_file_t *file_p)
 */
 
STATIC mp_obj_t mp_lv_fs_close(size_t n_args, const mp_obj_t *args)
{
    lv_fs_file_t *file_p = mp_write_ptr_lv_fs_file_t(args[0]);
    lv_fs_res_t res = lv_fs_close(file_p);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_close_obj, 1, 1, mp_lv_fs_close);

 

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_remove(const char *path)
 */
 
STATIC mp_obj_t mp_lv_fs_remove(size_t n_args, const mp_obj_t *args)
{
    const char *path = (char*)mp_obj_str_get_str(args[0]);
    lv_fs_res_t res = lv_fs_remove(path);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_remove_obj, 1, 1, mp_lv_fs_remove);

 

/*
 * Function NOT generated:
 * Missing conversion to uint32_t*
 * lv_fs_res_t lv_fs_read(lv_fs_file_t *file_p, void *buf, uint32_t btr, uint32_t *br)
 */
    

/*
 * Function NOT generated:
 * Missing conversion to uint32_t*
 * lv_fs_res_t lv_fs_write(lv_fs_file_t *file_p, const void *buf, uint32_t btw, uint32_t *bw)
 */
    

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_seek(lv_fs_file_t *file_p, uint32_t pos)
 */
 
STATIC mp_obj_t mp_lv_fs_seek(size_t n_args, const mp_obj_t *args)
{
    lv_fs_file_t *file_p = mp_write_ptr_lv_fs_file_t(args[0]);
    uint32_t pos = (uint32_t)mp_obj_int_get_checked(args[1]);
    lv_fs_res_t res = lv_fs_seek(file_p, pos);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_seek_obj, 2, 2, mp_lv_fs_seek);

 

/*
 * Function NOT generated:
 * Missing conversion to uint32_t*
 * lv_fs_res_t lv_fs_tell(lv_fs_file_t *file_p, uint32_t *pos)
 */
    

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_trunc(lv_fs_file_t *file_p)
 */
 
STATIC mp_obj_t mp_lv_fs_trunc(size_t n_args, const mp_obj_t *args)
{
    lv_fs_file_t *file_p = mp_write_ptr_lv_fs_file_t(args[0]);
    lv_fs_res_t res = lv_fs_trunc(file_p);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_trunc_obj, 1, 1, mp_lv_fs_trunc);

 

/*
 * Function NOT generated:
 * Missing conversion to uint32_t*
 * lv_fs_res_t lv_fs_size(lv_fs_file_t *file_p, uint32_t *size)
 */
    

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_rename(const char *oldname, const char *newname)
 */
 
STATIC mp_obj_t mp_lv_fs_rename(size_t n_args, const mp_obj_t *args)
{
    const char *oldname = (char*)mp_obj_str_get_str(args[0]);
    const char *newname = (char*)mp_obj_str_get_str(args[1]);
    lv_fs_res_t res = lv_fs_rename(oldname, newname);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_rename_obj, 2, 2, mp_lv_fs_rename);

 

/*
 * Struct lv_fs_dir_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_fs_dir_t_type();

STATIC inline lv_fs_dir_t* mp_write_ptr_lv_fs_dir_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_fs_dir_t*)self->data;
}

#define mp_write_lv_fs_dir_t(struct_obj) *mp_write_ptr_lv_fs_dir_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_fs_dir_t(lv_fs_dir_t *field)
{
    return lv_to_mp_struct(get_mp_lv_fs_dir_t_type(), field);
}

#define mp_read_lv_fs_dir_t(field) mp_read_ptr_lv_fs_dir_t(&field)

STATIC void mp_lv_fs_dir_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_fs_dir_t *data = (lv_fs_dir_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_dir_d: dest[0] = ptr_to_mp((void*)data->dir_d); break; // converting from void*;
            case MP_QSTR_drv: dest[0] = ptr_to_mp((void*)data->drv); break; // converting from struct __lv_fs_drv_t*;
            default: field_not_found(MP_QSTR_lv_fs_dir_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_dir_d: data->dir_d = (void*)mp_to_ptr(dest[1]); break; // converting to void*;
                case MP_QSTR_drv: data->drv = (void*)mp_to_ptr(dest[1]); break; // converting to struct __lv_fs_drv_t*;
                default: field_not_found(MP_QSTR_lv_fs_dir_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_fs_dir_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_fs_dir_t");
}

STATIC const mp_rom_map_elem_t mp_lv_fs_dir_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_fs_dir_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_fs_dir_t_locals_dict, mp_lv_fs_dir_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_fs_dir_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_fs_dir_t,
    .print = mp_lv_fs_dir_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_fs_dir_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_fs_dir_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_fs_dir_t_type()
{
    return &mp_lv_fs_dir_t_type;
}
    

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_dir_open(lv_fs_dir_t *rddir_p, const char *path)
 */
 
STATIC mp_obj_t mp_lv_fs_dir_open(size_t n_args, const mp_obj_t *args)
{
    lv_fs_dir_t *rddir_p = mp_write_ptr_lv_fs_dir_t(args[0]);
    const char *path = (char*)mp_obj_str_get_str(args[1]);
    lv_fs_res_t res = lv_fs_dir_open(rddir_p, path);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_dir_open_obj, 2, 2, mp_lv_fs_dir_open);

 

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_dir_read(lv_fs_dir_t *rddir_p, char *fn)
 */
 
STATIC mp_obj_t mp_lv_fs_dir_read(size_t n_args, const mp_obj_t *args)
{
    lv_fs_dir_t *rddir_p = mp_write_ptr_lv_fs_dir_t(args[0]);
    char *fn = (char*)mp_obj_str_get_str(args[1]);
    lv_fs_res_t res = lv_fs_dir_read(rddir_p, fn);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_dir_read_obj, 2, 2, mp_lv_fs_dir_read);

 

/*
 * lvgl extension definition for:
 * lv_fs_res_t lv_fs_dir_close(lv_fs_dir_t *rddir_p)
 */
 
STATIC mp_obj_t mp_lv_fs_dir_close(size_t n_args, const mp_obj_t *args)
{
    lv_fs_dir_t *rddir_p = mp_write_ptr_lv_fs_dir_t(args[0]);
    lv_fs_res_t res = lv_fs_dir_close(rddir_p);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_dir_close_obj, 1, 1, mp_lv_fs_dir_close);

 

/*
 * Function NOT generated:
 * Missing conversion to char
 * lv_fs_res_t lv_fs_free(char letter, uint32_t *total_p, uint32_t *free_p)
 */
    

/*
 * lvgl extension definition for:
 * char *lv_fs_get_letters(char *buf)
 */
 
STATIC mp_obj_t mp_lv_fs_get_letters(size_t n_args, const mp_obj_t *args)
{
    char *buf = (char*)mp_obj_str_get_str(args[0]);
    char* res = lv_fs_get_letters(buf);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_get_letters_obj, 1, 1, mp_lv_fs_get_letters);

 

/*
 * lvgl extension definition for:
 * const char *lv_fs_get_ext(const char *fn)
 */
 
STATIC mp_obj_t mp_lv_fs_get_ext(size_t n_args, const mp_obj_t *args)
{
    const char *fn = (char*)mp_obj_str_get_str(args[0]);
    const char* res = lv_fs_get_ext(fn);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_get_ext_obj, 1, 1, mp_lv_fs_get_ext);

 

/*
 * lvgl extension definition for:
 * char *lv_fs_up(char *path)
 */
 
STATIC mp_obj_t mp_lv_fs_up(size_t n_args, const mp_obj_t *args)
{
    char *path = (char*)mp_obj_str_get_str(args[0]);
    char* res = lv_fs_up(path);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_up_obj, 1, 1, mp_lv_fs_up);

 

/*
 * lvgl extension definition for:
 * const char *lv_fs_get_last(const char *path)
 */
 
STATIC mp_obj_t mp_lv_fs_get_last(size_t n_args, const mp_obj_t *args)
{
    const char *path = (char*)mp_obj_str_get_str(args[0]);
    const char* res = lv_fs_get_last(path);
    return convert_to_str((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_fs_get_last_obj, 1, 1, mp_lv_fs_get_last);

 

/*
 * lvgl extension definition for:
 * lv_opa_t lv_draw_aa_get_opa(lv_coord_t seg, lv_coord_t px_id, lv_opa_t base_opa)
 */
 
STATIC mp_obj_t mp_lv_draw_aa_get_opa(size_t n_args, const mp_obj_t *args)
{
    lv_coord_t seg = (int16_t)mp_obj_int_get_checked(args[0]);
    lv_coord_t px_id = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_opa_t base_opa = (uint8_t)mp_obj_int_get_checked(args[2]);
    lv_opa_t res = lv_draw_aa_get_opa(seg, px_id, base_opa);
    return mp_obj_new_int_from_uint(res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_aa_get_opa_obj, 3, 3, mp_lv_draw_aa_get_opa);

 

/*
 * lvgl extension definition for:
 * void lv_draw_aa_ver_seg(lv_coord_t x, lv_coord_t y, lv_coord_t length, const lv_area_t *mask, lv_color_t color, lv_opa_t opa)
 */
 
STATIC mp_obj_t mp_lv_draw_aa_ver_seg(size_t n_args, const mp_obj_t *args)
{
    lv_coord_t x = (int16_t)mp_obj_int_get_checked(args[0]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t length = (int16_t)mp_obj_int_get_checked(args[2]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[3]);
    lv_color_t color = mp_write_lv_color16_t(args[4]);
    lv_opa_t opa = (uint8_t)mp_obj_int_get_checked(args[5]);
    lv_draw_aa_ver_seg(x, y, length, mask, color, opa);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_aa_ver_seg_obj, 6, 6, mp_lv_draw_aa_ver_seg);

 

/*
 * lvgl extension definition for:
 * void lv_draw_aa_hor_seg(lv_coord_t x, lv_coord_t y, lv_coord_t length, const lv_area_t *mask, lv_color_t color, lv_opa_t opa)
 */
 
STATIC mp_obj_t mp_lv_draw_aa_hor_seg(size_t n_args, const mp_obj_t *args)
{
    lv_coord_t x = (int16_t)mp_obj_int_get_checked(args[0]);
    lv_coord_t y = (int16_t)mp_obj_int_get_checked(args[1]);
    lv_coord_t length = (int16_t)mp_obj_int_get_checked(args[2]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[3]);
    lv_color_t color = mp_write_lv_color16_t(args[4]);
    lv_opa_t opa = (uint8_t)mp_obj_int_get_checked(args[5]);
    lv_draw_aa_hor_seg(x, y, length, mask, color, opa);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_aa_hor_seg_obj, 6, 6, mp_lv_draw_aa_hor_seg);

 

/*
 * lvgl extension definition for:
 * void lv_draw_rect(const lv_area_t *coords, const lv_area_t *mask, const lv_style_t *style, lv_opa_t opa_scale)
 */
 
STATIC mp_obj_t mp_lv_draw_rect(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *coords = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[1]);
    const lv_style_t *style = mp_to_ptr(args[2]);
    lv_opa_t opa_scale = (uint8_t)mp_obj_int_get_checked(args[3]);
    lv_draw_rect(coords, mask, style, opa_scale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_rect_obj, 4, 4, mp_lv_draw_rect);

 

/*
 * lvgl extension definition for:
 * void lv_draw_label(const lv_area_t *coords, const lv_area_t *mask, const lv_style_t *style, lv_opa_t opa_scale, const char *txt, lv_txt_flag_t flag, lv_point_t *offset)
 */
 
STATIC mp_obj_t mp_lv_draw_label(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *coords = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[1]);
    const lv_style_t *style = mp_to_ptr(args[2]);
    lv_opa_t opa_scale = (uint8_t)mp_obj_int_get_checked(args[3]);
    const char *txt = (char*)mp_obj_str_get_str(args[4]);
    lv_txt_flag_t flag = (uint8_t)mp_obj_int_get_checked(args[5]);
    lv_point_t *offset = mp_to_ptr(args[6]);
    lv_draw_label(coords, mask, style, opa_scale, txt, flag, offset);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_label_obj, 7, 7, mp_lv_draw_label);

 

/*
 * lvgl extension definition for:
 * void lv_draw_img(const lv_area_t *coords, const lv_area_t *mask, const void *src, const lv_style_t *style, lv_opa_t opa_scale)
 */
 
STATIC mp_obj_t mp_lv_draw_img(size_t n_args, const mp_obj_t *args)
{
    const lv_area_t *coords = mp_write_ptr_lv_area_t(args[0]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[1]);
    const void *src = mp_to_ptr(args[2]);
    const lv_style_t *style = mp_to_ptr(args[3]);
    lv_opa_t opa_scale = (uint8_t)mp_obj_int_get_checked(args[4]);
    lv_draw_img(coords, mask, src, style, opa_scale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_img_obj, 5, 5, mp_lv_draw_img);

 

/*
 * lvgl extension definition for:
 * void lv_draw_line(const lv_point_t *point1, const lv_point_t *point2, const lv_area_t *mask, const lv_style_t *style, lv_opa_t opa_scale)
 */
 
STATIC mp_obj_t mp_lv_draw_line(size_t n_args, const mp_obj_t *args)
{
    const lv_point_t *point1 = mp_to_ptr(args[0]);
    const lv_point_t *point2 = mp_to_ptr(args[1]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[2]);
    const lv_style_t *style = mp_to_ptr(args[3]);
    lv_opa_t opa_scale = (uint8_t)mp_obj_int_get_checked(args[4]);
    lv_draw_line(point1, point2, mask, style, opa_scale);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_line_obj, 5, 5, mp_lv_draw_line);

 

/*
 * lvgl extension definition for:
 * void lv_draw_triangle(const lv_point_t *points, const lv_area_t *mask, lv_color_t color)
 */
 
STATIC mp_obj_t mp_lv_draw_triangle(size_t n_args, const mp_obj_t *args)
{
    const lv_point_t *points = mp_to_ptr(args[0]);
    const lv_area_t *mask = mp_write_ptr_lv_area_t(args[1]);
    lv_color_t color = mp_write_lv_color16_t(args[2]);
    lv_draw_triangle(points, mask, color);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_draw_triangle_obj, 3, 3, mp_lv_draw_triangle);

 
typedef typeof( ((lv_theme_t*)(0))->btn ) lv_theme_btn_t;

/*
 * Struct lv_theme_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btn_t_type();

STATIC inline lv_theme_btn_t* mp_write_ptr_lv_theme_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_btn_t*)self->data;
}

#define mp_write_lv_theme_btn_t(struct_obj) *mp_write_ptr_lv_theme_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_btn_t(lv_theme_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_btn_t_type(), field);
}

#define mp_read_lv_theme_btn_t(field) mp_read_ptr_lv_theme_btn_t(&field)

STATIC void mp_lv_theme_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_btn_t *data = (lv_theme_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            case MP_QSTR_ina: dest[0] = ptr_to_mp((void*)data->ina); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_ina: data->ina = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_btn_t_locals_dict, mp_lv_theme_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_btn_t,
    .print = mp_lv_theme_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btn_t_type()
{
    return &mp_lv_theme_btn_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->label ) lv_theme_label_t;

/*
 * Struct lv_theme_label_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_label_t_type();

STATIC inline lv_theme_label_t* mp_write_ptr_lv_theme_label_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_label_t*)self->data;
}

#define mp_write_lv_theme_label_t(struct_obj) *mp_write_ptr_lv_theme_label_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_label_t(lv_theme_label_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_label_t_type(), field);
}

#define mp_read_lv_theme_label_t(field) mp_read_ptr_lv_theme_label_t(&field)

STATIC void mp_lv_theme_label_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_label_t *data = (lv_theme_label_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_prim: dest[0] = ptr_to_mp((void*)data->prim); break; // converting from lv_style_t*;
            case MP_QSTR_sec: dest[0] = ptr_to_mp((void*)data->sec); break; // converting from lv_style_t*;
            case MP_QSTR_hint: dest[0] = ptr_to_mp((void*)data->hint); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_label_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_prim: data->prim = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sec: data->sec = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_hint: data->hint = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_label_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_label_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_label_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_label_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_label_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_label_t_locals_dict, mp_lv_theme_label_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_label_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_label_t,
    .print = mp_lv_theme_label_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_label_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_label_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_label_t_type()
{
    return &mp_lv_theme_label_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->img ) lv_theme_img_t;

/*
 * Struct lv_theme_img_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_img_t_type();

STATIC inline lv_theme_img_t* mp_write_ptr_lv_theme_img_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_img_t*)self->data;
}

#define mp_write_lv_theme_img_t(struct_obj) *mp_write_ptr_lv_theme_img_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_img_t(lv_theme_img_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_img_t_type(), field);
}

#define mp_read_lv_theme_img_t(field) mp_read_ptr_lv_theme_img_t(&field)

STATIC void mp_lv_theme_img_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_img_t *data = (lv_theme_img_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_light: dest[0] = ptr_to_mp((void*)data->light); break; // converting from lv_style_t*;
            case MP_QSTR_dark: dest[0] = ptr_to_mp((void*)data->dark); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_img_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_light: data->light = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_dark: data->dark = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_img_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_img_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_img_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_img_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_img_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_img_t_locals_dict, mp_lv_theme_img_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_img_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_img_t,
    .print = mp_lv_theme_img_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_img_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_img_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_img_t_type()
{
    return &mp_lv_theme_img_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->line ) lv_theme_line_t;

/*
 * Struct lv_theme_line_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_line_t_type();

STATIC inline lv_theme_line_t* mp_write_ptr_lv_theme_line_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_line_t*)self->data;
}

#define mp_write_lv_theme_line_t(struct_obj) *mp_write_ptr_lv_theme_line_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_line_t(lv_theme_line_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_line_t_type(), field);
}

#define mp_read_lv_theme_line_t(field) mp_read_ptr_lv_theme_line_t(&field)

STATIC void mp_lv_theme_line_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_line_t *data = (lv_theme_line_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_decor: dest[0] = ptr_to_mp((void*)data->decor); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_line_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_decor: data->decor = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_line_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_line_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_line_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_line_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_line_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_line_t_locals_dict, mp_lv_theme_line_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_line_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_line_t,
    .print = mp_lv_theme_line_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_line_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_line_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_line_t_type()
{
    return &mp_lv_theme_line_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->bar ) lv_theme_bar_t;

/*
 * Struct lv_theme_bar_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_bar_t_type();

STATIC inline lv_theme_bar_t* mp_write_ptr_lv_theme_bar_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_bar_t*)self->data;
}

#define mp_write_lv_theme_bar_t(struct_obj) *mp_write_ptr_lv_theme_bar_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_bar_t(lv_theme_bar_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_bar_t_type(), field);
}

#define mp_read_lv_theme_bar_t(field) mp_read_ptr_lv_theme_bar_t(&field)

STATIC void mp_lv_theme_bar_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_bar_t *data = (lv_theme_bar_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_indic: dest[0] = ptr_to_mp((void*)data->indic); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_bar_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_indic: data->indic = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_bar_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_bar_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_bar_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_bar_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_bar_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_bar_t_locals_dict, mp_lv_theme_bar_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_bar_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_bar_t,
    .print = mp_lv_theme_bar_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_bar_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_bar_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_bar_t_type()
{
    return &mp_lv_theme_bar_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->slider ) lv_theme_slider_t;

/*
 * Struct lv_theme_slider_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_slider_t_type();

STATIC inline lv_theme_slider_t* mp_write_ptr_lv_theme_slider_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_slider_t*)self->data;
}

#define mp_write_lv_theme_slider_t(struct_obj) *mp_write_ptr_lv_theme_slider_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_slider_t(lv_theme_slider_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_slider_t_type(), field);
}

#define mp_read_lv_theme_slider_t(field) mp_read_ptr_lv_theme_slider_t(&field)

STATIC void mp_lv_theme_slider_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_slider_t *data = (lv_theme_slider_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_indic: dest[0] = ptr_to_mp((void*)data->indic); break; // converting from lv_style_t*;
            case MP_QSTR_knob: dest[0] = ptr_to_mp((void*)data->knob); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_slider_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_indic: data->indic = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_knob: data->knob = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_slider_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_slider_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_slider_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_slider_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_slider_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_slider_t_locals_dict, mp_lv_theme_slider_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_slider_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_slider_t,
    .print = mp_lv_theme_slider_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_slider_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_slider_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_slider_t_type()
{
    return &mp_lv_theme_slider_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->sw ) lv_theme_sw_t;

/*
 * Struct lv_theme_sw_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_sw_t_type();

STATIC inline lv_theme_sw_t* mp_write_ptr_lv_theme_sw_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_sw_t*)self->data;
}

#define mp_write_lv_theme_sw_t(struct_obj) *mp_write_ptr_lv_theme_sw_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_sw_t(lv_theme_sw_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_sw_t_type(), field);
}

#define mp_read_lv_theme_sw_t(field) mp_read_ptr_lv_theme_sw_t(&field)

STATIC void mp_lv_theme_sw_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_sw_t *data = (lv_theme_sw_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_indic: dest[0] = ptr_to_mp((void*)data->indic); break; // converting from lv_style_t*;
            case MP_QSTR_knob_off: dest[0] = ptr_to_mp((void*)data->knob_off); break; // converting from lv_style_t*;
            case MP_QSTR_knob_on: dest[0] = ptr_to_mp((void*)data->knob_on); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_sw_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_indic: data->indic = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_knob_off: data->knob_off = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_knob_on: data->knob_on = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_sw_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_sw_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_sw_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_sw_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_sw_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_sw_t_locals_dict, mp_lv_theme_sw_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_sw_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_sw_t,
    .print = mp_lv_theme_sw_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_sw_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_sw_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_sw_t_type()
{
    return &mp_lv_theme_sw_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->cb ) lv_theme_cb_t;
typedef typeof( ((lv_theme_cb_t*)(0))->box ) lv_theme_cb_box_t;

/*
 * Struct lv_theme_cb_box_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_cb_box_t_type();

STATIC inline lv_theme_cb_box_t* mp_write_ptr_lv_theme_cb_box_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_cb_box_t*)self->data;
}

#define mp_write_lv_theme_cb_box_t(struct_obj) *mp_write_ptr_lv_theme_cb_box_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_cb_box_t(lv_theme_cb_box_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_cb_box_t_type(), field);
}

#define mp_read_lv_theme_cb_box_t(field) mp_read_ptr_lv_theme_cb_box_t(&field)

STATIC void mp_lv_theme_cb_box_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_cb_box_t *data = (lv_theme_cb_box_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            case MP_QSTR_ina: dest[0] = ptr_to_mp((void*)data->ina); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_cb_box_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_ina: data->ina = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_cb_box_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_cb_box_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_cb_box_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_cb_box_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_cb_box_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_cb_box_t_locals_dict, mp_lv_theme_cb_box_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_cb_box_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_cb_box_t,
    .print = mp_lv_theme_cb_box_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_cb_box_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_cb_box_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_cb_box_t_type()
{
    return &mp_lv_theme_cb_box_t_type;
}
    

/*
 * Struct lv_theme_cb_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_cb_t_type();

STATIC inline lv_theme_cb_t* mp_write_ptr_lv_theme_cb_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_cb_t*)self->data;
}

#define mp_write_lv_theme_cb_t(struct_obj) *mp_write_ptr_lv_theme_cb_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_cb_t(lv_theme_cb_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_cb_t_type(), field);
}

#define mp_read_lv_theme_cb_t(field) mp_read_ptr_lv_theme_cb_t(&field)

STATIC void mp_lv_theme_cb_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_cb_t *data = (lv_theme_cb_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_box: dest[0] = mp_read_lv_theme_cb_box_t(data->box); break; // converting from lv_theme_cb_box_t;
            default: field_not_found(MP_QSTR_lv_theme_cb_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_box: data->box = mp_write_lv_theme_cb_box_t(dest[1]); break; // converting to lv_theme_cb_box_t;
                default: field_not_found(MP_QSTR_lv_theme_cb_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_cb_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_cb_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_cb_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_cb_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_cb_t_locals_dict, mp_lv_theme_cb_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_cb_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_cb_t,
    .print = mp_lv_theme_cb_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_cb_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_cb_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_cb_t_type()
{
    return &mp_lv_theme_cb_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->btnm ) lv_theme_btnm_t;
typedef typeof( ((lv_theme_btnm_t*)(0))->btn ) lv_theme_btnm_btn_t;

/*
 * Struct lv_theme_btnm_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btnm_btn_t_type();

STATIC inline lv_theme_btnm_btn_t* mp_write_ptr_lv_theme_btnm_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_btnm_btn_t*)self->data;
}

#define mp_write_lv_theme_btnm_btn_t(struct_obj) *mp_write_ptr_lv_theme_btnm_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_btnm_btn_t(lv_theme_btnm_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_btnm_btn_t_type(), field);
}

#define mp_read_lv_theme_btnm_btn_t(field) mp_read_ptr_lv_theme_btnm_btn_t(&field)

STATIC void mp_lv_theme_btnm_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_btnm_btn_t *data = (lv_theme_btnm_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            case MP_QSTR_ina: dest[0] = ptr_to_mp((void*)data->ina); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_btnm_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_ina: data->ina = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_btnm_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_btnm_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_btnm_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_btnm_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_btnm_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_btnm_btn_t_locals_dict, mp_lv_theme_btnm_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_btnm_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_btnm_btn_t,
    .print = mp_lv_theme_btnm_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_btnm_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_btnm_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btnm_btn_t_type()
{
    return &mp_lv_theme_btnm_btn_t_type;
}
    

/*
 * Struct lv_theme_btnm_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btnm_t_type();

STATIC inline lv_theme_btnm_t* mp_write_ptr_lv_theme_btnm_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_btnm_t*)self->data;
}

#define mp_write_lv_theme_btnm_t(struct_obj) *mp_write_ptr_lv_theme_btnm_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_btnm_t(lv_theme_btnm_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_btnm_t_type(), field);
}

#define mp_read_lv_theme_btnm_t(field) mp_read_ptr_lv_theme_btnm_t(&field)

STATIC void mp_lv_theme_btnm_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_btnm_t *data = (lv_theme_btnm_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_btnm_btn_t(data->btn); break; // converting from lv_theme_btnm_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_btnm_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_btnm_btn_t(dest[1]); break; // converting to lv_theme_btnm_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_btnm_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_btnm_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_btnm_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_btnm_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_btnm_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_btnm_t_locals_dict, mp_lv_theme_btnm_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_btnm_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_btnm_t,
    .print = mp_lv_theme_btnm_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_btnm_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_btnm_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_btnm_t_type()
{
    return &mp_lv_theme_btnm_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->kb ) lv_theme_kb_t;
typedef typeof( ((lv_theme_kb_t*)(0))->btn ) lv_theme_kb_btn_t;

/*
 * Struct lv_theme_kb_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_kb_btn_t_type();

STATIC inline lv_theme_kb_btn_t* mp_write_ptr_lv_theme_kb_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_kb_btn_t*)self->data;
}

#define mp_write_lv_theme_kb_btn_t(struct_obj) *mp_write_ptr_lv_theme_kb_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_kb_btn_t(lv_theme_kb_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_kb_btn_t_type(), field);
}

#define mp_read_lv_theme_kb_btn_t(field) mp_read_ptr_lv_theme_kb_btn_t(&field)

STATIC void mp_lv_theme_kb_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_kb_btn_t *data = (lv_theme_kb_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            case MP_QSTR_ina: dest[0] = ptr_to_mp((void*)data->ina); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_kb_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_ina: data->ina = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_kb_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_kb_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_kb_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_kb_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_kb_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_kb_btn_t_locals_dict, mp_lv_theme_kb_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_kb_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_kb_btn_t,
    .print = mp_lv_theme_kb_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_kb_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_kb_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_kb_btn_t_type()
{
    return &mp_lv_theme_kb_btn_t_type;
}
    

/*
 * Struct lv_theme_kb_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_kb_t_type();

STATIC inline lv_theme_kb_t* mp_write_ptr_lv_theme_kb_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_kb_t*)self->data;
}

#define mp_write_lv_theme_kb_t(struct_obj) *mp_write_ptr_lv_theme_kb_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_kb_t(lv_theme_kb_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_kb_t_type(), field);
}

#define mp_read_lv_theme_kb_t(field) mp_read_ptr_lv_theme_kb_t(&field)

STATIC void mp_lv_theme_kb_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_kb_t *data = (lv_theme_kb_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_kb_btn_t(data->btn); break; // converting from lv_theme_kb_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_kb_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_kb_btn_t(dest[1]); break; // converting to lv_theme_kb_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_kb_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_kb_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_kb_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_kb_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_kb_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_kb_t_locals_dict, mp_lv_theme_kb_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_kb_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_kb_t,
    .print = mp_lv_theme_kb_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_kb_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_kb_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_kb_t_type()
{
    return &mp_lv_theme_kb_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->mbox ) lv_theme_mbox_t;
typedef typeof( ((lv_theme_mbox_t*)(0))->btn ) lv_theme_mbox_btn_t;

/*
 * Struct lv_theme_mbox_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_mbox_btn_t_type();

STATIC inline lv_theme_mbox_btn_t* mp_write_ptr_lv_theme_mbox_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_mbox_btn_t*)self->data;
}

#define mp_write_lv_theme_mbox_btn_t(struct_obj) *mp_write_ptr_lv_theme_mbox_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_mbox_btn_t(lv_theme_mbox_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_mbox_btn_t_type(), field);
}

#define mp_read_lv_theme_mbox_btn_t(field) mp_read_ptr_lv_theme_mbox_btn_t(&field)

STATIC void mp_lv_theme_mbox_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_mbox_btn_t *data = (lv_theme_mbox_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_mbox_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_mbox_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_mbox_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_mbox_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_mbox_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_mbox_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_mbox_btn_t_locals_dict, mp_lv_theme_mbox_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_mbox_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_mbox_btn_t,
    .print = mp_lv_theme_mbox_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_mbox_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_mbox_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_mbox_btn_t_type()
{
    return &mp_lv_theme_mbox_btn_t_type;
}
    

/*
 * Struct lv_theme_mbox_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_mbox_t_type();

STATIC inline lv_theme_mbox_t* mp_write_ptr_lv_theme_mbox_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_mbox_t*)self->data;
}

#define mp_write_lv_theme_mbox_t(struct_obj) *mp_write_ptr_lv_theme_mbox_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_mbox_t(lv_theme_mbox_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_mbox_t_type(), field);
}

#define mp_read_lv_theme_mbox_t(field) mp_read_ptr_lv_theme_mbox_t(&field)

STATIC void mp_lv_theme_mbox_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_mbox_t *data = (lv_theme_mbox_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_mbox_btn_t(data->btn); break; // converting from lv_theme_mbox_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_mbox_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_mbox_btn_t(dest[1]); break; // converting to lv_theme_mbox_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_mbox_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_mbox_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_mbox_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_mbox_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_mbox_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_mbox_t_locals_dict, mp_lv_theme_mbox_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_mbox_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_mbox_t,
    .print = mp_lv_theme_mbox_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_mbox_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_mbox_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_mbox_t_type()
{
    return &mp_lv_theme_mbox_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->page ) lv_theme_page_t;

/*
 * Struct lv_theme_page_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_page_t_type();

STATIC inline lv_theme_page_t* mp_write_ptr_lv_theme_page_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_page_t*)self->data;
}

#define mp_write_lv_theme_page_t(struct_obj) *mp_write_ptr_lv_theme_page_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_page_t(lv_theme_page_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_page_t_type(), field);
}

#define mp_read_lv_theme_page_t(field) mp_read_ptr_lv_theme_page_t(&field)

STATIC void mp_lv_theme_page_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_page_t *data = (lv_theme_page_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_scrl: dest[0] = ptr_to_mp((void*)data->scrl); break; // converting from lv_style_t*;
            case MP_QSTR_sb: dest[0] = ptr_to_mp((void*)data->sb); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_page_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_scrl: data->scrl = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sb: data->sb = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_page_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_page_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_page_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_page_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_page_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_page_t_locals_dict, mp_lv_theme_page_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_page_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_page_t,
    .print = mp_lv_theme_page_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_page_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_page_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_page_t_type()
{
    return &mp_lv_theme_page_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->ta ) lv_theme_ta_t;

/*
 * Struct lv_theme_ta_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_ta_t_type();

STATIC inline lv_theme_ta_t* mp_write_ptr_lv_theme_ta_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_ta_t*)self->data;
}

#define mp_write_lv_theme_ta_t(struct_obj) *mp_write_ptr_lv_theme_ta_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_ta_t(lv_theme_ta_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_ta_t_type(), field);
}

#define mp_read_lv_theme_ta_t(field) mp_read_ptr_lv_theme_ta_t(&field)

STATIC void mp_lv_theme_ta_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_ta_t *data = (lv_theme_ta_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_area: dest[0] = ptr_to_mp((void*)data->area); break; // converting from lv_style_t*;
            case MP_QSTR_oneline: dest[0] = ptr_to_mp((void*)data->oneline); break; // converting from lv_style_t*;
            case MP_QSTR_cursor: dest[0] = ptr_to_mp((void*)data->cursor); break; // converting from lv_style_t*;
            case MP_QSTR_sb: dest[0] = ptr_to_mp((void*)data->sb); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_ta_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_area: data->area = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_oneline: data->oneline = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_cursor: data->cursor = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sb: data->sb = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_ta_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_ta_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_ta_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_ta_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_ta_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_ta_t_locals_dict, mp_lv_theme_ta_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_ta_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_ta_t,
    .print = mp_lv_theme_ta_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_ta_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_ta_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_ta_t_type()
{
    return &mp_lv_theme_ta_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->list ) lv_theme_list_t;
typedef typeof( ((lv_theme_list_t*)(0))->btn ) lv_theme_list_btn_t;

/*
 * Struct lv_theme_list_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_list_btn_t_type();

STATIC inline lv_theme_list_btn_t* mp_write_ptr_lv_theme_list_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_list_btn_t*)self->data;
}

#define mp_write_lv_theme_list_btn_t(struct_obj) *mp_write_ptr_lv_theme_list_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_list_btn_t(lv_theme_list_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_list_btn_t_type(), field);
}

#define mp_read_lv_theme_list_btn_t(field) mp_read_ptr_lv_theme_list_btn_t(&field)

STATIC void mp_lv_theme_list_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_list_btn_t *data = (lv_theme_list_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            case MP_QSTR_ina: dest[0] = ptr_to_mp((void*)data->ina); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_list_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_ina: data->ina = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_list_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_list_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_list_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_list_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_list_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_list_btn_t_locals_dict, mp_lv_theme_list_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_list_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_list_btn_t,
    .print = mp_lv_theme_list_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_list_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_list_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_list_btn_t_type()
{
    return &mp_lv_theme_list_btn_t_type;
}
    

/*
 * Struct lv_theme_list_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_list_t_type();

STATIC inline lv_theme_list_t* mp_write_ptr_lv_theme_list_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_list_t*)self->data;
}

#define mp_write_lv_theme_list_t(struct_obj) *mp_write_ptr_lv_theme_list_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_list_t(lv_theme_list_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_list_t_type(), field);
}

#define mp_read_lv_theme_list_t(field) mp_read_ptr_lv_theme_list_t(&field)

STATIC void mp_lv_theme_list_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_list_t *data = (lv_theme_list_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_scrl: dest[0] = ptr_to_mp((void*)data->scrl); break; // converting from lv_style_t*;
            case MP_QSTR_sb: dest[0] = ptr_to_mp((void*)data->sb); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_list_btn_t(data->btn); break; // converting from lv_theme_list_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_list_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_scrl: data->scrl = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sb: data->sb = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_list_btn_t(dest[1]); break; // converting to lv_theme_list_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_list_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_list_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_list_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_list_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_list_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_list_t_locals_dict, mp_lv_theme_list_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_list_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_list_t,
    .print = mp_lv_theme_list_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_list_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_list_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_list_t_type()
{
    return &mp_lv_theme_list_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->ddlist ) lv_theme_ddlist_t;

/*
 * Struct lv_theme_ddlist_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_ddlist_t_type();

STATIC inline lv_theme_ddlist_t* mp_write_ptr_lv_theme_ddlist_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_ddlist_t*)self->data;
}

#define mp_write_lv_theme_ddlist_t(struct_obj) *mp_write_ptr_lv_theme_ddlist_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_ddlist_t(lv_theme_ddlist_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_ddlist_t_type(), field);
}

#define mp_read_lv_theme_ddlist_t(field) mp_read_ptr_lv_theme_ddlist_t(&field)

STATIC void mp_lv_theme_ddlist_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_ddlist_t *data = (lv_theme_ddlist_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_sel: dest[0] = ptr_to_mp((void*)data->sel); break; // converting from lv_style_t*;
            case MP_QSTR_sb: dest[0] = ptr_to_mp((void*)data->sb); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_ddlist_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sel: data->sel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sb: data->sb = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_ddlist_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_ddlist_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_ddlist_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_ddlist_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_ddlist_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_ddlist_t_locals_dict, mp_lv_theme_ddlist_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_ddlist_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_ddlist_t,
    .print = mp_lv_theme_ddlist_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_ddlist_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_ddlist_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_ddlist_t_type()
{
    return &mp_lv_theme_ddlist_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->roller ) lv_theme_roller_t;

/*
 * Struct lv_theme_roller_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_roller_t_type();

STATIC inline lv_theme_roller_t* mp_write_ptr_lv_theme_roller_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_roller_t*)self->data;
}

#define mp_write_lv_theme_roller_t(struct_obj) *mp_write_ptr_lv_theme_roller_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_roller_t(lv_theme_roller_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_roller_t_type(), field);
}

#define mp_read_lv_theme_roller_t(field) mp_read_ptr_lv_theme_roller_t(&field)

STATIC void mp_lv_theme_roller_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_roller_t *data = (lv_theme_roller_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_sel: dest[0] = ptr_to_mp((void*)data->sel); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_roller_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sel: data->sel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_roller_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_roller_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_roller_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_roller_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_roller_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_roller_t_locals_dict, mp_lv_theme_roller_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_roller_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_roller_t,
    .print = mp_lv_theme_roller_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_roller_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_roller_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_roller_t_type()
{
    return &mp_lv_theme_roller_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->tabview ) lv_theme_tabview_t;
typedef typeof( ((lv_theme_tabview_t*)(0))->btn ) lv_theme_tabview_btn_t;

/*
 * Struct lv_theme_tabview_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_tabview_btn_t_type();

STATIC inline lv_theme_tabview_btn_t* mp_write_ptr_lv_theme_tabview_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_tabview_btn_t*)self->data;
}

#define mp_write_lv_theme_tabview_btn_t(struct_obj) *mp_write_ptr_lv_theme_tabview_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_tabview_btn_t(lv_theme_tabview_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_tabview_btn_t_type(), field);
}

#define mp_read_lv_theme_tabview_btn_t(field) mp_read_ptr_lv_theme_tabview_btn_t(&field)

STATIC void mp_lv_theme_tabview_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_tabview_btn_t *data = (lv_theme_tabview_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_rel: dest[0] = ptr_to_mp((void*)data->tgl_rel); break; // converting from lv_style_t*;
            case MP_QSTR_tgl_pr: dest[0] = ptr_to_mp((void*)data->tgl_pr); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_tabview_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_rel: data->tgl_rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_tgl_pr: data->tgl_pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_tabview_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_tabview_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_tabview_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_tabview_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_tabview_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_tabview_btn_t_locals_dict, mp_lv_theme_tabview_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_tabview_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_tabview_btn_t,
    .print = mp_lv_theme_tabview_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_tabview_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_tabview_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_tabview_btn_t_type()
{
    return &mp_lv_theme_tabview_btn_t_type;
}
    

/*
 * Struct lv_theme_tabview_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_tabview_t_type();

STATIC inline lv_theme_tabview_t* mp_write_ptr_lv_theme_tabview_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_tabview_t*)self->data;
}

#define mp_write_lv_theme_tabview_t(struct_obj) *mp_write_ptr_lv_theme_tabview_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_tabview_t(lv_theme_tabview_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_tabview_t_type(), field);
}

#define mp_read_lv_theme_tabview_t(field) mp_read_ptr_lv_theme_tabview_t(&field)

STATIC void mp_lv_theme_tabview_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_tabview_t *data = (lv_theme_tabview_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_indic: dest[0] = ptr_to_mp((void*)data->indic); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_tabview_btn_t(data->btn); break; // converting from lv_theme_tabview_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_tabview_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_indic: data->indic = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_tabview_btn_t(dest[1]); break; // converting to lv_theme_tabview_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_tabview_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_tabview_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_tabview_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_tabview_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_tabview_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_tabview_t_locals_dict, mp_lv_theme_tabview_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_tabview_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_tabview_t,
    .print = mp_lv_theme_tabview_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_tabview_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_tabview_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_tabview_t_type()
{
    return &mp_lv_theme_tabview_t_type;
}
    
typedef typeof( ((lv_theme_t*)(0))->win ) lv_theme_win_t;
typedef typeof( ((lv_theme_win_t*)(0))->content ) lv_theme_win_content_t;

/*
 * Struct lv_theme_win_content_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_content_t_type();

STATIC inline lv_theme_win_content_t* mp_write_ptr_lv_theme_win_content_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_win_content_t*)self->data;
}

#define mp_write_lv_theme_win_content_t(struct_obj) *mp_write_ptr_lv_theme_win_content_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_win_content_t(lv_theme_win_content_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_win_content_t_type(), field);
}

#define mp_read_lv_theme_win_content_t(field) mp_read_ptr_lv_theme_win_content_t(&field)

STATIC void mp_lv_theme_win_content_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_win_content_t *data = (lv_theme_win_content_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_scrl: dest[0] = ptr_to_mp((void*)data->scrl); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_win_content_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_scrl: data->scrl = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_win_content_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_win_content_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_win_content_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_win_content_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_win_content_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_win_content_t_locals_dict, mp_lv_theme_win_content_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_win_content_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_win_content_t,
    .print = mp_lv_theme_win_content_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_win_content_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_win_content_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_content_t_type()
{
    return &mp_lv_theme_win_content_t_type;
}
    
typedef typeof( ((lv_theme_win_t*)(0))->btn ) lv_theme_win_btn_t;

/*
 * Struct lv_theme_win_btn_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_btn_t_type();

STATIC inline lv_theme_win_btn_t* mp_write_ptr_lv_theme_win_btn_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_win_btn_t*)self->data;
}

#define mp_write_lv_theme_win_btn_t(struct_obj) *mp_write_ptr_lv_theme_win_btn_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_win_btn_t(lv_theme_win_btn_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_win_btn_t_type(), field);
}

#define mp_read_lv_theme_win_btn_t(field) mp_read_ptr_lv_theme_win_btn_t(&field)

STATIC void mp_lv_theme_win_btn_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_win_btn_t *data = (lv_theme_win_btn_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_rel: dest[0] = ptr_to_mp((void*)data->rel); break; // converting from lv_style_t*;
            case MP_QSTR_pr: dest[0] = ptr_to_mp((void*)data->pr); break; // converting from lv_style_t*;
            default: field_not_found(MP_QSTR_lv_theme_win_btn_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_rel: data->rel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_pr: data->pr = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                default: field_not_found(MP_QSTR_lv_theme_win_btn_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_win_btn_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_win_btn_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_win_btn_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_win_btn_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_win_btn_t_locals_dict, mp_lv_theme_win_btn_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_win_btn_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_win_btn_t,
    .print = mp_lv_theme_win_btn_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_win_btn_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_win_btn_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_btn_t_type()
{
    return &mp_lv_theme_win_btn_t_type;
}
    

/*
 * Struct lv_theme_win_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_t_type();

STATIC inline lv_theme_win_t* mp_write_ptr_lv_theme_win_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_win_t*)self->data;
}

#define mp_write_lv_theme_win_t(struct_obj) *mp_write_ptr_lv_theme_win_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_win_t(lv_theme_win_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_win_t_type(), field);
}

#define mp_read_lv_theme_win_t(field) mp_read_ptr_lv_theme_win_t(&field)

STATIC void mp_lv_theme_win_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_win_t *data = (lv_theme_win_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_sb: dest[0] = ptr_to_mp((void*)data->sb); break; // converting from lv_style_t*;
            case MP_QSTR_header: dest[0] = ptr_to_mp((void*)data->header); break; // converting from lv_style_t*;
            case MP_QSTR_content: dest[0] = mp_read_lv_theme_win_content_t(data->content); break; // converting from lv_theme_win_content_t;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_win_btn_t(data->btn); break; // converting from lv_theme_win_btn_t;
            default: field_not_found(MP_QSTR_lv_theme_win_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sb: data->sb = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_header: data->header = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_content: data->content = mp_write_lv_theme_win_content_t(dest[1]); break; // converting to lv_theme_win_content_t;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_win_btn_t(dest[1]); break; // converting to lv_theme_win_btn_t;
                default: field_not_found(MP_QSTR_lv_theme_win_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_win_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_win_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_win_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_win_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_win_t_locals_dict, mp_lv_theme_win_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_win_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_win_t,
    .print = mp_lv_theme_win_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_win_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_win_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_win_t_type()
{
    return &mp_lv_theme_win_t_type;
}
    

/*
 * Struct lv_theme_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_theme_t_type();

STATIC inline lv_theme_t* mp_write_ptr_lv_theme_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_theme_t*)self->data;
}

#define mp_write_lv_theme_t(struct_obj) *mp_write_ptr_lv_theme_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_theme_t(lv_theme_t *field)
{
    return lv_to_mp_struct(get_mp_lv_theme_t_type(), field);
}

#define mp_read_lv_theme_t(field) mp_read_ptr_lv_theme_t(&field)

STATIC void mp_lv_theme_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_theme_t *data = (lv_theme_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_bg: dest[0] = ptr_to_mp((void*)data->bg); break; // converting from lv_style_t*;
            case MP_QSTR_panel: dest[0] = ptr_to_mp((void*)data->panel); break; // converting from lv_style_t*;
            case MP_QSTR_cont: dest[0] = ptr_to_mp((void*)data->cont); break; // converting from lv_style_t*;
            case MP_QSTR_btn: dest[0] = mp_read_lv_theme_btn_t(data->btn); break; // converting from lv_theme_btn_t;
            case MP_QSTR_label: dest[0] = mp_read_lv_theme_label_t(data->label); break; // converting from lv_theme_label_t;
            case MP_QSTR_img: dest[0] = mp_read_lv_theme_img_t(data->img); break; // converting from lv_theme_img_t;
            case MP_QSTR_line: dest[0] = mp_read_lv_theme_line_t(data->line); break; // converting from lv_theme_line_t;
            case MP_QSTR_led: dest[0] = ptr_to_mp((void*)data->led); break; // converting from lv_style_t*;
            case MP_QSTR_bar: dest[0] = mp_read_lv_theme_bar_t(data->bar); break; // converting from lv_theme_bar_t;
            case MP_QSTR_slider: dest[0] = mp_read_lv_theme_slider_t(data->slider); break; // converting from lv_theme_slider_t;
            case MP_QSTR_lmeter: dest[0] = ptr_to_mp((void*)data->lmeter); break; // converting from lv_style_t*;
            case MP_QSTR_gauge: dest[0] = ptr_to_mp((void*)data->gauge); break; // converting from lv_style_t*;
            case MP_QSTR_arc: dest[0] = ptr_to_mp((void*)data->arc); break; // converting from lv_style_t*;
            case MP_QSTR_preload: dest[0] = ptr_to_mp((void*)data->preload); break; // converting from lv_style_t*;
            case MP_QSTR_sw: dest[0] = mp_read_lv_theme_sw_t(data->sw); break; // converting from lv_theme_sw_t;
            case MP_QSTR_chart: dest[0] = ptr_to_mp((void*)data->chart); break; // converting from lv_style_t*;
            case MP_QSTR_cb: dest[0] = mp_read_lv_theme_cb_t(data->cb); break; // converting from lv_theme_cb_t;
            case MP_QSTR_btnm: dest[0] = mp_read_lv_theme_btnm_t(data->btnm); break; // converting from lv_theme_btnm_t;
            case MP_QSTR_kb: dest[0] = mp_read_lv_theme_kb_t(data->kb); break; // converting from lv_theme_kb_t;
            case MP_QSTR_mbox: dest[0] = mp_read_lv_theme_mbox_t(data->mbox); break; // converting from lv_theme_mbox_t;
            case MP_QSTR_page: dest[0] = mp_read_lv_theme_page_t(data->page); break; // converting from lv_theme_page_t;
            case MP_QSTR_ta: dest[0] = mp_read_lv_theme_ta_t(data->ta); break; // converting from lv_theme_ta_t;
            case MP_QSTR_list: dest[0] = mp_read_lv_theme_list_t(data->list); break; // converting from lv_theme_list_t;
            case MP_QSTR_ddlist: dest[0] = mp_read_lv_theme_ddlist_t(data->ddlist); break; // converting from lv_theme_ddlist_t;
            case MP_QSTR_roller: dest[0] = mp_read_lv_theme_roller_t(data->roller); break; // converting from lv_theme_roller_t;
            case MP_QSTR_tabview: dest[0] = mp_read_lv_theme_tabview_t(data->tabview); break; // converting from lv_theme_tabview_t;
            case MP_QSTR_win: dest[0] = mp_read_lv_theme_win_t(data->win); break; // converting from lv_theme_win_t;
            default: field_not_found(MP_QSTR_lv_theme_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_bg: data->bg = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_panel: data->panel = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_cont: data->cont = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_btn: data->btn = mp_write_lv_theme_btn_t(dest[1]); break; // converting to lv_theme_btn_t;
                case MP_QSTR_label: data->label = mp_write_lv_theme_label_t(dest[1]); break; // converting to lv_theme_label_t;
                case MP_QSTR_img: data->img = mp_write_lv_theme_img_t(dest[1]); break; // converting to lv_theme_img_t;
                case MP_QSTR_line: data->line = mp_write_lv_theme_line_t(dest[1]); break; // converting to lv_theme_line_t;
                case MP_QSTR_led: data->led = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_bar: data->bar = mp_write_lv_theme_bar_t(dest[1]); break; // converting to lv_theme_bar_t;
                case MP_QSTR_slider: data->slider = mp_write_lv_theme_slider_t(dest[1]); break; // converting to lv_theme_slider_t;
                case MP_QSTR_lmeter: data->lmeter = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_gauge: data->gauge = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_arc: data->arc = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_preload: data->preload = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_sw: data->sw = mp_write_lv_theme_sw_t(dest[1]); break; // converting to lv_theme_sw_t;
                case MP_QSTR_chart: data->chart = (void*)mp_to_ptr(dest[1]); break; // converting to lv_style_t*;
                case MP_QSTR_cb: data->cb = mp_write_lv_theme_cb_t(dest[1]); break; // converting to lv_theme_cb_t;
                case MP_QSTR_btnm: data->btnm = mp_write_lv_theme_btnm_t(dest[1]); break; // converting to lv_theme_btnm_t;
                case MP_QSTR_kb: data->kb = mp_write_lv_theme_kb_t(dest[1]); break; // converting to lv_theme_kb_t;
                case MP_QSTR_mbox: data->mbox = mp_write_lv_theme_mbox_t(dest[1]); break; // converting to lv_theme_mbox_t;
                case MP_QSTR_page: data->page = mp_write_lv_theme_page_t(dest[1]); break; // converting to lv_theme_page_t;
                case MP_QSTR_ta: data->ta = mp_write_lv_theme_ta_t(dest[1]); break; // converting to lv_theme_ta_t;
                case MP_QSTR_list: data->list = mp_write_lv_theme_list_t(dest[1]); break; // converting to lv_theme_list_t;
                case MP_QSTR_ddlist: data->ddlist = mp_write_lv_theme_ddlist_t(dest[1]); break; // converting to lv_theme_ddlist_t;
                case MP_QSTR_roller: data->roller = mp_write_lv_theme_roller_t(dest[1]); break; // converting to lv_theme_roller_t;
                case MP_QSTR_tabview: data->tabview = mp_write_lv_theme_tabview_t(dest[1]); break; // converting to lv_theme_tabview_t;
                case MP_QSTR_win: data->win = mp_write_lv_theme_win_t(dest[1]); break; // converting to lv_theme_win_t;
                default: field_not_found(MP_QSTR_lv_theme_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_theme_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_theme_t");
}

STATIC const mp_rom_map_elem_t mp_lv_theme_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_theme_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_theme_t_locals_dict, mp_lv_theme_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_theme_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_theme_t,
    .print = mp_lv_theme_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_theme_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_theme_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_theme_t_type()
{
    return &mp_lv_theme_t_type;
}
    

/*
 * lvgl extension definition for:
 * void lv_theme_set_current(lv_theme_t *th)
 */
 
STATIC mp_obj_t mp_lv_theme_set_current(size_t n_args, const mp_obj_t *args)
{
    lv_theme_t *th = mp_write_ptr_lv_theme_t(args[0]);
    lv_theme_set_current(th);
    return mp_const_none;
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_set_current_obj, 1, 1, mp_lv_theme_set_current);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_current(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_current(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_current();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_current_obj, 0, 0, mp_lv_theme_get_current);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_default_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_default_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_default_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_default_init_obj, 2, 2, mp_lv_theme_default_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_default(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_default(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_default();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_default_obj, 0, 0, mp_lv_theme_get_default);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_alien_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_alien_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_alien_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_alien_init_obj, 2, 2, mp_lv_theme_alien_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_alien(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_alien(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_alien();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_alien_obj, 0, 0, mp_lv_theme_get_alien);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_night_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_night_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_night_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_night_init_obj, 2, 2, mp_lv_theme_night_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_night(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_night(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_night();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_night_obj, 0, 0, mp_lv_theme_get_night);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_zen_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_zen_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_zen_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_zen_init_obj, 2, 2, mp_lv_theme_zen_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_zen(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_zen(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_zen();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_zen_obj, 0, 0, mp_lv_theme_get_zen);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_mono_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_mono_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_mono_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_mono_init_obj, 2, 2, mp_lv_theme_mono_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_mono(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_mono(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_mono();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_mono_obj, 0, 0, mp_lv_theme_get_mono);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_material_init(uint16_t hue, lv_font_t *font)
 */
 
STATIC mp_obj_t mp_lv_theme_material_init(size_t n_args, const mp_obj_t *args)
{
    uint16_t hue = (uint16_t)mp_obj_int_get_checked(args[0]);
    lv_font_t *font = mp_to_ptr(args[1]);
    lv_theme_t* res = lv_theme_material_init(hue, font);
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_material_init_obj, 2, 2, mp_lv_theme_material_init);

 

/*
 * lvgl extension definition for:
 * lv_theme_t *lv_theme_get_material(void)
 */
 
STATIC mp_obj_t mp_lv_theme_get_material(size_t n_args, const mp_obj_t *args)
{
    
    lv_theme_t* res = lv_theme_get_material();
    return mp_read_ptr_lv_theme_t((void*)res);
}

MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_lv_theme_get_material_obj, 0, 0, mp_lv_theme_get_material);

 

/*
 *
 * Globals definitions
 *
 */


/*
 * lvgl lv_style_btn_pr global definitions
 */

STATIC void mp_lv_style_btn_pr_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_btn_pr");
}

STATIC mp_obj_t make_new_lv_style_btn_pr(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_btn_pr);
}

STATIC const mp_obj_type_t mp_lv_style_btn_pr_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_btn_pr,
    .print = mp_lv_style_btn_pr_print,
    .make_new = make_new_lv_style_btn_pr,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_transp global definitions
 */

STATIC void mp_lv_style_transp_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_transp");
}

STATIC mp_obj_t make_new_lv_style_transp(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_transp);
}

STATIC const mp_obj_type_t mp_lv_style_transp_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_transp,
    .print = mp_lv_style_transp_print,
    .make_new = make_new_lv_style_transp,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_btn_tgl_rel global definitions
 */

STATIC void mp_lv_style_btn_tgl_rel_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_btn_tgl_rel");
}

STATIC mp_obj_t make_new_lv_style_btn_tgl_rel(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_btn_tgl_rel);
}

STATIC const mp_obj_type_t mp_lv_style_btn_tgl_rel_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_btn_tgl_rel,
    .print = mp_lv_style_btn_tgl_rel_print,
    .make_new = make_new_lv_style_btn_tgl_rel,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * Struct lv_font_t
 */

STATIC inline const mp_obj_type_t *get_mp_lv_font_t_type();

STATIC inline lv_font_t* mp_write_ptr_lv_font_t(mp_obj_t self_in)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    return (lv_font_t*)self->data;
}

#define mp_write_lv_font_t(struct_obj) *mp_write_ptr_lv_font_t(struct_obj)

STATIC inline mp_obj_t mp_read_ptr_lv_font_t(lv_font_t *field)
{
    return lv_to_mp_struct(get_mp_lv_font_t_type(), field);
}

#define mp_read_lv_font_t(field) mp_read_ptr_lv_font_t(&field)

STATIC void mp_lv_font_t_attr(mp_obj_t self_in, qstr attr, mp_obj_t *dest)
{
    mp_lv_struct_t *self = MP_OBJ_TO_PTR(self_in);
    lv_font_t *data = (lv_font_t*)self->data;

    if (dest[0] == MP_OBJ_NULL) {
        // load attribute
        switch(attr)
        {
            case MP_QSTR_unicode_first: dest[0] = mp_obj_new_int_from_uint(data->unicode_first); break; // converting from uint32_t;
            case MP_QSTR_unicode_last: dest[0] = mp_obj_new_int_from_uint(data->unicode_last); break; // converting from uint32_t;
            case MP_QSTR_glyph_bitmap: dest[0] = ptr_to_mp((void*)data->glyph_bitmap); break; // converting from uint8_t*;
            case MP_QSTR_glyph_dsc: dest[0] = ptr_to_mp((void*)data->glyph_dsc); break; // converting from lv_font_glyph_dsc_t*;
            case MP_QSTR_unicode_list: dest[0] = ptr_to_mp((void*)data->unicode_list); break; // converting from uint32_t*;
            case MP_QSTR_get_bitmap: dest[0] = ptr_to_mp((void*)data->get_bitmap); break; // converting from const uint8_t *get_bitmap(const struct _lv_font_struct *, uint32_t)*;
            case MP_QSTR_get_width: dest[0] = ptr_to_mp((void*)data->get_width); break; // converting from int16_t get_width(const struct _lv_font_struct *, uint32_t)*;
            case MP_QSTR_next_page: dest[0] = ptr_to_mp((void*)data->next_page); break; // converting from struct _lv_font_struct*;
            case MP_QSTR_h_px: dest[0] = mp_obj_new_int_from_uint(data->h_px); break; // converting from uint32_t;
            case MP_QSTR_bpp: dest[0] = mp_obj_new_int_from_uint(data->bpp); break; // converting from uint32_t;
            case MP_QSTR_monospace: dest[0] = mp_obj_new_int_from_uint(data->monospace); break; // converting from uint32_t;
            case MP_QSTR_glyph_cnt: dest[0] = mp_obj_new_int_from_uint(data->glyph_cnt); break; // converting from uint16_t;
            default: field_not_found(MP_QSTR_lv_font_t, attr);
        }
    } else {
        if (dest[1])
        {
            // store attribute
            switch(attr)
            {
                case MP_QSTR_unicode_first: data->unicode_first = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_unicode_last: data->unicode_last = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_glyph_bitmap: data->glyph_bitmap = (void*)mp_to_ptr(dest[1]); break; // converting to uint8_t*;
                case MP_QSTR_glyph_dsc: data->glyph_dsc = (void*)mp_to_ptr(dest[1]); break; // converting to lv_font_glyph_dsc_t*;
                case MP_QSTR_unicode_list: data->unicode_list = (void*)mp_to_ptr(dest[1]); break; // converting to uint32_t*;
                case MP_QSTR_get_bitmap: data->get_bitmap = (void*)mp_to_ptr(dest[1]); break; // converting to const uint8_t *get_bitmap(const struct _lv_font_struct *, uint32_t)*;
                case MP_QSTR_get_width: data->get_width = (void*)mp_to_ptr(dest[1]); break; // converting to int16_t get_width(const struct _lv_font_struct *, uint32_t)*;
                case MP_QSTR_next_page: data->next_page = (void*)mp_to_ptr(dest[1]); break; // converting to struct _lv_font_struct*;
                case MP_QSTR_h_px: data->h_px = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_bpp: data->bpp = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_monospace: data->monospace = (uint32_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint32_t;
                case MP_QSTR_glyph_cnt: data->glyph_cnt = (uint16_t)mp_obj_int_get_checked(dest[1]); break; // converting to uint16_t;
                default: field_not_found(MP_QSTR_lv_font_t, attr);
            }

            dest[0] = MP_OBJ_NULL; // indicate success
        }
    }
}

STATIC void mp_lv_font_t_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl struct lv_font_t");
}

STATIC const mp_rom_map_elem_t mp_lv_font_t_locals_dict_table[] = {
    { MP_ROM_QSTR(MP_QSTR_SIZE), MP_ROM_PTR(MP_ROM_INT(sizeof(lv_font_t))) },
    { MP_OBJ_NEW_QSTR(MP_QSTR___del__), (mp_obj_t)&delete_lv_struct_obj },
};

STATIC MP_DEFINE_CONST_DICT(mp_lv_font_t_locals_dict, mp_lv_font_t_locals_dict_table);

STATIC const mp_obj_type_t mp_lv_font_t_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_font_t,
    .print = mp_lv_font_t_print,
    .make_new = make_new_lv_struct,
    .attr = mp_lv_font_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_font_t_locals_dict
};

STATIC inline const mp_obj_type_t *get_mp_lv_font_t_type()
{
    return &mp_lv_font_t_type;
}
    

/*
 * lvgl lv_font_symbol_20 global definitions
 */

STATIC void mp_lv_font_symbol_20_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_font_symbol_20");
}

STATIC mp_obj_t make_new_lv_font_symbol_20(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_font_symbol_20);
}

STATIC const mp_obj_type_t mp_lv_font_symbol_20_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_font_symbol_20,
    .print = mp_lv_font_symbol_20_print,
    .make_new = make_new_lv_font_symbol_20,
    .attr = mp_lv_font_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_font_t_locals_dict
};
    

/*
 * lvgl lv_style_transp_tight global definitions
 */

STATIC void mp_lv_style_transp_tight_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_transp_tight");
}

STATIC mp_obj_t make_new_lv_style_transp_tight(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_transp_tight);
}

STATIC const mp_obj_type_t mp_lv_style_transp_tight_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_transp_tight,
    .print = mp_lv_style_transp_tight_print,
    .make_new = make_new_lv_style_transp_tight,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_btn_ina global definitions
 */

STATIC void mp_lv_style_btn_ina_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_btn_ina");
}

STATIC mp_obj_t make_new_lv_style_btn_ina(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_btn_ina);
}

STATIC const mp_obj_type_t mp_lv_style_btn_ina_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_btn_ina,
    .print = mp_lv_style_btn_ina_print,
    .make_new = make_new_lv_style_btn_ina,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_transp_fit global definitions
 */

STATIC void mp_lv_style_transp_fit_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_transp_fit");
}

STATIC mp_obj_t make_new_lv_style_transp_fit(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_transp_fit);
}

STATIC const mp_obj_type_t mp_lv_style_transp_fit_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_transp_fit,
    .print = mp_lv_style_transp_fit_print,
    .make_new = make_new_lv_style_transp_fit,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_pretty_color global definitions
 */

STATIC void mp_lv_style_pretty_color_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_pretty_color");
}

STATIC mp_obj_t make_new_lv_style_pretty_color(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_pretty_color);
}

STATIC const mp_obj_type_t mp_lv_style_pretty_color_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_pretty_color,
    .print = mp_lv_style_pretty_color_print,
    .make_new = make_new_lv_style_pretty_color,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_btn_tgl_pr global definitions
 */

STATIC void mp_lv_style_btn_tgl_pr_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_btn_tgl_pr");
}

STATIC mp_obj_t make_new_lv_style_btn_tgl_pr(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_btn_tgl_pr);
}

STATIC const mp_obj_type_t mp_lv_style_btn_tgl_pr_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_btn_tgl_pr,
    .print = mp_lv_style_btn_tgl_pr_print,
    .make_new = make_new_lv_style_btn_tgl_pr,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_btn_rel global definitions
 */

STATIC void mp_lv_style_btn_rel_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_btn_rel");
}

STATIC mp_obj_t make_new_lv_style_btn_rel(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_btn_rel);
}

STATIC const mp_obj_type_t mp_lv_style_btn_rel_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_btn_rel,
    .print = mp_lv_style_btn_rel_print,
    .make_new = make_new_lv_style_btn_rel,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_scr global definitions
 */

STATIC void mp_lv_style_scr_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_scr");
}

STATIC mp_obj_t make_new_lv_style_scr(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_scr);
}

STATIC const mp_obj_type_t mp_lv_style_scr_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_scr,
    .print = mp_lv_style_scr_print,
    .make_new = make_new_lv_style_scr,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_pretty global definitions
 */

STATIC void mp_lv_style_pretty_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_pretty");
}

STATIC mp_obj_t make_new_lv_style_pretty(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_pretty);
}

STATIC const mp_obj_type_t mp_lv_style_pretty_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_pretty,
    .print = mp_lv_style_pretty_print,
    .make_new = make_new_lv_style_pretty,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_font_dejavu_20 global definitions
 */

STATIC void mp_lv_font_dejavu_20_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_font_dejavu_20");
}

STATIC mp_obj_t make_new_lv_font_dejavu_20(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_font_dejavu_20);
}

STATIC const mp_obj_type_t mp_lv_font_dejavu_20_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_font_dejavu_20,
    .print = mp_lv_font_dejavu_20_print,
    .make_new = make_new_lv_font_dejavu_20,
    .attr = mp_lv_font_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_font_t_locals_dict
};
    

/*
 * lvgl lv_style_plain_color global definitions
 */

STATIC void mp_lv_style_plain_color_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_plain_color");
}

STATIC mp_obj_t make_new_lv_style_plain_color(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_plain_color);
}

STATIC const mp_obj_type_t mp_lv_style_plain_color_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_plain_color,
    .print = mp_lv_style_plain_color_print,
    .make_new = make_new_lv_style_plain_color,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    

/*
 * lvgl lv_style_plain global definitions
 */

STATIC void mp_lv_style_plain_print(const mp_print_t *print,
    mp_obj_t self_in,
    mp_print_kind_t kind)
{
    mp_printf(print, "lvgl global lv_style_plain");
}

STATIC mp_obj_t make_new_lv_style_plain(
    const mp_obj_type_t *type,
    size_t n_args,
    size_t n_kw,
    const mp_obj_t *args)
{
    mp_arg_check_num(n_args, n_kw, 0, 0, false);
    return lv_to_mp_struct(type, &lv_style_plain);
}

STATIC const mp_obj_type_t mp_lv_style_plain_type = {
    { &mp_type_type },
    .name = MP_QSTR_lv_style_plain,
    .print = mp_lv_style_plain_print,
    .make_new = make_new_lv_style_plain,
    .attr = mp_lv_style_t_attr,
    .locals_dict = (mp_obj_dict_t*)&mp_lv_style_t_locals_dict
};
    


/*
 * lvgl module definitions
 * User should implement lv_mp_init. Display can be initialized there, if needed.
 */

extern void lv_mp_init();

STATIC mp_obj_t _lv_mp_init()
{
    lv_mp_init();
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_0(lv_mp_init_obj, _lv_mp_init);

STATIC const mp_rom_map_elem_t lvgl_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_lvgl) },
    { MP_ROM_QSTR(MP_QSTR___init__), MP_ROM_PTR(&lv_mp_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_obj), MP_ROM_PTR(&mp_obj_type) },
    { MP_ROM_QSTR(MP_QSTR_arc), MP_ROM_PTR(&mp_arc_type) },
    { MP_ROM_QSTR(MP_QSTR_cont), MP_ROM_PTR(&mp_cont_type) },
    { MP_ROM_QSTR(MP_QSTR_btn), MP_ROM_PTR(&mp_btn_type) },
    { MP_ROM_QSTR(MP_QSTR_label), MP_ROM_PTR(&mp_label_type) },
    { MP_ROM_QSTR(MP_QSTR_bar), MP_ROM_PTR(&mp_bar_type) },
    { MP_ROM_QSTR(MP_QSTR_btnm), MP_ROM_PTR(&mp_btnm_type) },
    { MP_ROM_QSTR(MP_QSTR_cb), MP_ROM_PTR(&mp_cb_type) },
    { MP_ROM_QSTR(MP_QSTR_line), MP_ROM_PTR(&mp_line_type) },
    { MP_ROM_QSTR(MP_QSTR_chart), MP_ROM_PTR(&mp_chart_type) },
    { MP_ROM_QSTR(MP_QSTR_page), MP_ROM_PTR(&mp_page_type) },
    { MP_ROM_QSTR(MP_QSTR_ddlist), MP_ROM_PTR(&mp_ddlist_type) },
    { MP_ROM_QSTR(MP_QSTR_lmeter), MP_ROM_PTR(&mp_lmeter_type) },
    { MP_ROM_QSTR(MP_QSTR_gauge), MP_ROM_PTR(&mp_gauge_type) },
    { MP_ROM_QSTR(MP_QSTR_img), MP_ROM_PTR(&mp_img_type) },
    { MP_ROM_QSTR(MP_QSTR_kb), MP_ROM_PTR(&mp_kb_type) },
    { MP_ROM_QSTR(MP_QSTR_led), MP_ROM_PTR(&mp_led_type) },
    { MP_ROM_QSTR(MP_QSTR_list), MP_ROM_PTR(&mp_list_type) },
    { MP_ROM_QSTR(MP_QSTR_mbox), MP_ROM_PTR(&mp_mbox_type) },
    { MP_ROM_QSTR(MP_QSTR_preload), MP_ROM_PTR(&mp_preload_type) },
    { MP_ROM_QSTR(MP_QSTR_roller), MP_ROM_PTR(&mp_roller_type) },
    { MP_ROM_QSTR(MP_QSTR_slider), MP_ROM_PTR(&mp_slider_type) },
    { MP_ROM_QSTR(MP_QSTR_sw), MP_ROM_PTR(&mp_sw_type) },
    { MP_ROM_QSTR(MP_QSTR_win), MP_ROM_PTR(&mp_win_type) },
    { MP_ROM_QSTR(MP_QSTR_tabview), MP_ROM_PTR(&mp_tabview_type) },
    { MP_ROM_QSTR(MP_QSTR_ta), MP_ROM_PTR(&mp_ta_type) },
    { MP_ROM_QSTR(MP_QSTR_color_to1), MP_ROM_PTR(&mp_lv_color_to1_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_to8), MP_ROM_PTR(&mp_lv_color_to8_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_to16), MP_ROM_PTR(&mp_lv_color_to16_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_to32), MP_ROM_PTR(&mp_lv_color_to32_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_mix), MP_ROM_PTR(&mp_lv_color_mix_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_brightness), MP_ROM_PTR(&mp_lv_color_brightness_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_copy), MP_ROM_PTR(&mp_lv_area_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_get_width), MP_ROM_PTR(&mp_lv_area_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_get_height), MP_ROM_PTR(&mp_lv_area_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_height), MP_ROM_PTR(&mp_lv_font_get_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_hsv_to_rgb), MP_ROM_PTR(&mp_lv_color_hsv_to_rgb_obj) },
    { MP_ROM_QSTR(MP_QSTR_color_rgb_to_hsv), MP_ROM_PTR(&mp_lv_color_rgb_to_hsv_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_set), MP_ROM_PTR(&mp_lv_area_set_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_set_width), MP_ROM_PTR(&mp_lv_area_set_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_set_height), MP_ROM_PTR(&mp_lv_area_set_height_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_set_pos), MP_ROM_PTR(&mp_lv_area_set_pos_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_get_size), MP_ROM_PTR(&mp_lv_area_get_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_intersect), MP_ROM_PTR(&mp_lv_area_intersect_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_join), MP_ROM_PTR(&mp_lv_area_join_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_is_point_on), MP_ROM_PTR(&mp_lv_area_is_point_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_is_on), MP_ROM_PTR(&mp_lv_area_is_on_obj) },
    { MP_ROM_QSTR(MP_QSTR_area_is_in), MP_ROM_PTR(&mp_lv_area_is_in_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_init), MP_ROM_PTR(&mp_lv_font_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_add), MP_ROM_PTR(&mp_lv_font_add_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_is_monospace), MP_ROM_PTR(&mp_lv_font_is_monospace_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_width), MP_ROM_PTR(&mp_lv_font_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_real_width), MP_ROM_PTR(&mp_lv_font_get_real_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_bpp), MP_ROM_PTR(&mp_lv_font_get_bpp_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_width_continuous), MP_ROM_PTR(&mp_lv_font_get_width_continuous_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_get_width_sparse), MP_ROM_PTR(&mp_lv_font_get_width_sparse_obj) },
    { MP_ROM_QSTR(MP_QSTR_font_builtin_init), MP_ROM_PTR(&mp_lv_font_builtin_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_init), MP_ROM_PTR(&mp_lv_anim_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_create), MP_ROM_PTR(&mp_lv_anim_create_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_del), MP_ROM_PTR(&mp_lv_anim_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_speed_to_time), MP_ROM_PTR(&mp_lv_anim_speed_to_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_path_linear), MP_ROM_PTR(&mp_lv_anim_path_linear_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_path_ease_in_out), MP_ROM_PTR(&mp_lv_anim_path_ease_in_out_obj) },
    { MP_ROM_QSTR(MP_QSTR_anim_path_step), MP_ROM_PTR(&mp_lv_anim_path_step_obj) },
    { MP_ROM_QSTR(MP_QSTR_style_init), MP_ROM_PTR(&mp_lv_style_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_style_copy), MP_ROM_PTR(&mp_lv_style_copy_obj) },
    { MP_ROM_QSTR(MP_QSTR_style_mix), MP_ROM_PTR(&mp_lv_style_mix_obj) },
    { MP_ROM_QSTR(MP_QSTR_style_anim_create), MP_ROM_PTR(&mp_lv_style_anim_create_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_init), MP_ROM_PTR(&mp_lv_mem_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_alloc), MP_ROM_PTR(&mp_lv_mem_alloc_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_free), MP_ROM_PTR(&mp_lv_mem_free_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_realloc), MP_ROM_PTR(&mp_lv_mem_realloc_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_defrag), MP_ROM_PTR(&mp_lv_mem_defrag_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_monitor), MP_ROM_PTR(&mp_lv_mem_monitor_obj) },
    { MP_ROM_QSTR(MP_QSTR_mem_get_size), MP_ROM_PTR(&mp_lv_mem_get_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_init), MP_ROM_PTR(&mp_lv_ll_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_ins_head), MP_ROM_PTR(&mp_lv_ll_ins_head_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_ins_prev), MP_ROM_PTR(&mp_lv_ll_ins_prev_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_ins_tail), MP_ROM_PTR(&mp_lv_ll_ins_tail_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_rem), MP_ROM_PTR(&mp_lv_ll_rem_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_clear), MP_ROM_PTR(&mp_lv_ll_clear_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_chg_list), MP_ROM_PTR(&mp_lv_ll_chg_list_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_get_head), MP_ROM_PTR(&mp_lv_ll_get_head_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_get_tail), MP_ROM_PTR(&mp_lv_ll_get_tail_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_get_next), MP_ROM_PTR(&mp_lv_ll_get_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_get_prev), MP_ROM_PTR(&mp_lv_ll_get_prev_obj) },
    { MP_ROM_QSTR(MP_QSTR_ll_move_before), MP_ROM_PTR(&mp_lv_ll_move_before_obj) },
    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&mp_lv_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_scr_load), MP_ROM_PTR(&mp_lv_scr_load_obj) },
    { MP_ROM_QSTR(MP_QSTR_scr_act), MP_ROM_PTR(&mp_lv_scr_act_obj) },
    { MP_ROM_QSTR(MP_QSTR_layer_top), MP_ROM_PTR(&mp_lv_layer_top_obj) },
    { MP_ROM_QSTR(MP_QSTR_layer_sys), MP_ROM_PTR(&mp_lv_layer_sys_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_drv_init), MP_ROM_PTR(&mp_lv_disp_drv_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_drv_register), MP_ROM_PTR(&mp_lv_disp_drv_register_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_set_active), MP_ROM_PTR(&mp_lv_disp_set_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_get_active), MP_ROM_PTR(&mp_lv_disp_get_active_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_next), MP_ROM_PTR(&mp_lv_disp_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_flush), MP_ROM_PTR(&mp_lv_disp_flush_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_fill), MP_ROM_PTR(&mp_lv_disp_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_map), MP_ROM_PTR(&mp_lv_disp_map_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_mem_blend), MP_ROM_PTR(&mp_lv_disp_mem_blend_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_mem_fill), MP_ROM_PTR(&mp_lv_disp_mem_fill_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_is_mem_blend_supported), MP_ROM_PTR(&mp_lv_disp_is_mem_blend_supported_obj) },
    { MP_ROM_QSTR(MP_QSTR_disp_is_mem_fill_supported), MP_ROM_PTR(&mp_lv_disp_is_mem_fill_supported_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick_inc), MP_ROM_PTR(&mp_lv_tick_inc_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick_get), MP_ROM_PTR(&mp_lv_tick_get_obj) },
    { MP_ROM_QSTR(MP_QSTR_tick_elaps), MP_ROM_PTR(&mp_lv_tick_elaps_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_drv_init), MP_ROM_PTR(&mp_lv_indev_drv_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_drv_register), MP_ROM_PTR(&mp_lv_indev_drv_register_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_next), MP_ROM_PTR(&mp_lv_indev_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_read), MP_ROM_PTR(&mp_lv_indev_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_create), MP_ROM_PTR(&mp_lv_group_create_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_del), MP_ROM_PTR(&mp_lv_group_del_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_add_obj), MP_ROM_PTR(&mp_lv_group_add_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_remove_obj), MP_ROM_PTR(&mp_lv_group_remove_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_focus_obj), MP_ROM_PTR(&mp_lv_group_focus_obj_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_focus_next), MP_ROM_PTR(&mp_lv_group_focus_next_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_focus_prev), MP_ROM_PTR(&mp_lv_group_focus_prev_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_focus_freeze), MP_ROM_PTR(&mp_lv_group_focus_freeze_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_send_data), MP_ROM_PTR(&mp_lv_group_send_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_set_style_mod_cb), MP_ROM_PTR(&mp_lv_group_set_style_mod_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_set_style_mod_edit_cb), MP_ROM_PTR(&mp_lv_group_set_style_mod_edit_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_set_focus_cb), MP_ROM_PTR(&mp_lv_group_set_focus_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_set_editing), MP_ROM_PTR(&mp_lv_group_set_editing_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_set_click_focus), MP_ROM_PTR(&mp_lv_group_set_click_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_mod_style), MP_ROM_PTR(&mp_lv_group_mod_style_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_focused), MP_ROM_PTR(&mp_lv_group_get_focused_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_style_mod_cb), MP_ROM_PTR(&mp_lv_group_get_style_mod_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_style_mod_edit_cb), MP_ROM_PTR(&mp_lv_group_get_style_mod_edit_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_focus_cb), MP_ROM_PTR(&mp_lv_group_get_focus_cb_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_editing), MP_ROM_PTR(&mp_lv_group_get_editing_obj) },
    { MP_ROM_QSTR(MP_QSTR_group_get_click_focus), MP_ROM_PTR(&mp_lv_group_get_click_focus_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_init), MP_ROM_PTR(&mp_lv_indev_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_act), MP_ROM_PTR(&mp_lv_indev_get_act_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_type), MP_ROM_PTR(&mp_lv_indev_get_type_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_reset), MP_ROM_PTR(&mp_lv_indev_reset_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_reset_lpr), MP_ROM_PTR(&mp_lv_indev_reset_lpr_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_enable), MP_ROM_PTR(&mp_lv_indev_enable_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_set_cursor), MP_ROM_PTR(&mp_lv_indev_set_cursor_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_set_group), MP_ROM_PTR(&mp_lv_indev_set_group_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_set_button_points), MP_ROM_PTR(&mp_lv_indev_set_button_points_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_point), MP_ROM_PTR(&mp_lv_indev_get_point_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_key), MP_ROM_PTR(&mp_lv_indev_get_key_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_is_dragging), MP_ROM_PTR(&mp_lv_indev_is_dragging_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_vect), MP_ROM_PTR(&mp_lv_indev_get_vect_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_get_inactive_time), MP_ROM_PTR(&mp_lv_indev_get_inactive_time_obj) },
    { MP_ROM_QSTR(MP_QSTR_indev_wait_release), MP_ROM_PTR(&mp_lv_indev_wait_release_obj) },
    { MP_ROM_QSTR(MP_QSTR_txt_get_size), MP_ROM_PTR(&mp_lv_txt_get_size_obj) },
    { MP_ROM_QSTR(MP_QSTR_txt_get_next_line), MP_ROM_PTR(&mp_lv_txt_get_next_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_txt_get_width), MP_ROM_PTR(&mp_lv_txt_get_width_obj) },
    { MP_ROM_QSTR(MP_QSTR_txt_ins), MP_ROM_PTR(&mp_lv_txt_ins_obj) },
    { MP_ROM_QSTR(MP_QSTR_txt_cut), MP_ROM_PTR(&mp_lv_txt_cut_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_init), MP_ROM_PTR(&mp_lv_fs_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_open), MP_ROM_PTR(&mp_lv_fs_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_close), MP_ROM_PTR(&mp_lv_fs_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_remove), MP_ROM_PTR(&mp_lv_fs_remove_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_seek), MP_ROM_PTR(&mp_lv_fs_seek_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_trunc), MP_ROM_PTR(&mp_lv_fs_trunc_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_rename), MP_ROM_PTR(&mp_lv_fs_rename_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_dir_open), MP_ROM_PTR(&mp_lv_fs_dir_open_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_dir_read), MP_ROM_PTR(&mp_lv_fs_dir_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_dir_close), MP_ROM_PTR(&mp_lv_fs_dir_close_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_get_letters), MP_ROM_PTR(&mp_lv_fs_get_letters_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_get_ext), MP_ROM_PTR(&mp_lv_fs_get_ext_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_up), MP_ROM_PTR(&mp_lv_fs_up_obj) },
    { MP_ROM_QSTR(MP_QSTR_fs_get_last), MP_ROM_PTR(&mp_lv_fs_get_last_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_aa_get_opa), MP_ROM_PTR(&mp_lv_draw_aa_get_opa_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_aa_ver_seg), MP_ROM_PTR(&mp_lv_draw_aa_ver_seg_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_aa_hor_seg), MP_ROM_PTR(&mp_lv_draw_aa_hor_seg_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_rect), MP_ROM_PTR(&mp_lv_draw_rect_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_label), MP_ROM_PTR(&mp_lv_draw_label_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_img), MP_ROM_PTR(&mp_lv_draw_img_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_line), MP_ROM_PTR(&mp_lv_draw_line_obj) },
    { MP_ROM_QSTR(MP_QSTR_draw_triangle), MP_ROM_PTR(&mp_lv_draw_triangle_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_set_current), MP_ROM_PTR(&mp_lv_theme_set_current_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_current), MP_ROM_PTR(&mp_lv_theme_get_current_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_default_init), MP_ROM_PTR(&mp_lv_theme_default_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_default), MP_ROM_PTR(&mp_lv_theme_get_default_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_alien_init), MP_ROM_PTR(&mp_lv_theme_alien_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_alien), MP_ROM_PTR(&mp_lv_theme_get_alien_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_night_init), MP_ROM_PTR(&mp_lv_theme_night_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_night), MP_ROM_PTR(&mp_lv_theme_get_night_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_zen_init), MP_ROM_PTR(&mp_lv_theme_zen_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_zen), MP_ROM_PTR(&mp_lv_theme_get_zen_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_mono_init), MP_ROM_PTR(&mp_lv_theme_mono_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_mono), MP_ROM_PTR(&mp_lv_theme_get_mono_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_material_init), MP_ROM_PTR(&mp_lv_theme_material_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_theme_get_material), MP_ROM_PTR(&mp_lv_theme_get_material_obj) },
    { MP_ROM_QSTR(MP_QSTR_BORDER), MP_ROM_PTR(&mp_LV_BORDER_type) },
    { MP_ROM_QSTR(MP_QSTR_ANIM), MP_ROM_PTR(&mp_LV_ANIM_type) },
    { MP_ROM_QSTR(MP_QSTR_LAYOUT), MP_ROM_PTR(&mp_LV_LAYOUT_type) },
    { MP_ROM_QSTR(MP_QSTR_INDEV_TYPE), MP_ROM_PTR(&mp_LV_INDEV_TYPE_type) },
    { MP_ROM_QSTR(MP_QSTR_FS_RES), MP_ROM_PTR(&mp_LV_FS_RES_type) },
    { MP_ROM_QSTR(MP_QSTR_SB_MODE), MP_ROM_PTR(&mp_LV_SB_MODE_type) },
    { MP_ROM_QSTR(MP_QSTR_SIGNAL), MP_ROM_PTR(&mp_LV_SIGNAL_type) },
    { MP_ROM_QSTR(MP_QSTR_ALIGN), MP_ROM_PTR(&mp_LV_ALIGN_type) },
    { MP_ROM_QSTR(MP_QSTR_INDEV_STATE), MP_ROM_PTR(&mp_LV_INDEV_STATE_type) },
    { MP_ROM_QSTR(MP_QSTR_CURSOR), MP_ROM_PTR(&mp_LV_CURSOR_type) },
    { MP_ROM_QSTR(MP_QSTR_SHADOW), MP_ROM_PTR(&mp_LV_SHADOW_type) },
    { MP_ROM_QSTR(MP_QSTR_TXT_FLAG), MP_ROM_PTR(&mp_LV_TXT_FLAG_type) },
    { MP_ROM_QSTR(MP_QSTR_RES), MP_ROM_PTR(&mp_LV_RES_type) },
    { MP_ROM_QSTR(MP_QSTR_FS_MODE), MP_ROM_PTR(&mp_LV_FS_MODE_type) },
    { MP_ROM_QSTR(MP_QSTR_TXT_CMD_STATE), MP_ROM_PTR(&mp_LV_TXT_CMD_STATE_type) },
    { MP_ROM_QSTR(MP_QSTR_PROTECT), MP_ROM_PTR(&mp_LV_PROTECT_type) },
    { MP_ROM_QSTR(MP_QSTR_DESIGN), MP_ROM_PTR(&mp_LV_DESIGN_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_color_hsv_t), MP_ROM_PTR(&mp_lv_color_hsv_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_img_header_t), MP_ROM_PTR(&mp_lv_img_header_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_anim_t), MP_ROM_PTR(&mp_lv_style_anim_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_font_t), MP_ROM_PTR(&mp_lv_font_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_disp_drv_t), MP_ROM_PTR(&mp_lv_disp_drv_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_disp_t), MP_ROM_PTR(&mp_lv_disp_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_chart_series_t), MP_ROM_PTR(&mp_lv_chart_series_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_theme_t), MP_ROM_PTR(&mp_lv_theme_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_fs_file_t), MP_ROM_PTR(&mp_lv_fs_file_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_indev_data_t), MP_ROM_PTR(&mp_lv_indev_data_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_ll_t), MP_ROM_PTR(&mp_lv_ll_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_mem_monitor_t), MP_ROM_PTR(&mp_lv_mem_monitor_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_group_t), MP_ROM_PTR(&mp_lv_group_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_point_t), MP_ROM_PTR(&mp_lv_point_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_t), MP_ROM_PTR(&mp_lv_style_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_indev_proc_t), MP_ROM_PTR(&mp_lv_indev_proc_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_area_t), MP_ROM_PTR(&mp_lv_area_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_anim_t), MP_ROM_PTR(&mp_lv_anim_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_indev_drv_t), MP_ROM_PTR(&mp_lv_indev_drv_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_color16_t), MP_ROM_PTR(&mp_lv_color16_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_fs_dir_t), MP_ROM_PTR(&mp_lv_fs_dir_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_indev_t), MP_ROM_PTR(&mp_lv_indev_t_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_btn_pr), MP_ROM_PTR(&mp_lv_style_btn_pr_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_transp), MP_ROM_PTR(&mp_lv_style_transp_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_btn_tgl_rel), MP_ROM_PTR(&mp_lv_style_btn_tgl_rel_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_font_symbol_20), MP_ROM_PTR(&mp_lv_font_symbol_20_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_transp_tight), MP_ROM_PTR(&mp_lv_style_transp_tight_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_btn_ina), MP_ROM_PTR(&mp_lv_style_btn_ina_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_transp_fit), MP_ROM_PTR(&mp_lv_style_transp_fit_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_pretty_color), MP_ROM_PTR(&mp_lv_style_pretty_color_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_btn_tgl_pr), MP_ROM_PTR(&mp_lv_style_btn_tgl_pr_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_btn_rel), MP_ROM_PTR(&mp_lv_style_btn_rel_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_scr), MP_ROM_PTR(&mp_lv_style_scr_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_pretty), MP_ROM_PTR(&mp_lv_style_pretty_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_font_dejavu_20), MP_ROM_PTR(&mp_lv_font_dejavu_20_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_plain_color), MP_ROM_PTR(&mp_lv_style_plain_color_type) },
    { MP_ROM_QSTR(MP_QSTR_lv_style_plain), MP_ROM_PTR(&mp_lv_style_plain_type) }
};


STATIC MP_DEFINE_CONST_DICT (
    mp_module_lvgl_globals,
    lvgl_globals_table
);

const mp_obj_module_t mp_module_lvgl = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&mp_module_lvgl_globals
};

