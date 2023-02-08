find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_program(AWK awk mawk gawk)

set(LV_BINDINGS_DIR ${CMAKE_CURRENT_LIST_DIR})

# Common function for creating LV bindings

function(lv_bindings)
    set(_options)
    set(_one_value_args OUTPUT)
    set(_multi_value_args INPUT DEPENDS COMPILE_OPTIONS PP_OPTIONS GEN_OPTIONS FILTER)
    cmake_parse_arguments(
        PARSE_ARGV 0 LV
        "${_options}"
        "${_one_value_args}"
        "${_multi_value_args}"
    )

    set(LV_PP ${LV_OUTPUT}.pp)
    set(LV_MPY_METADATA ${LV_OUTPUT}.json)

    add_custom_command(
        OUTPUT
            ${LV_PP}
        COMMAND
        ${CMAKE_C_COMPILER} -E -DPYCPARSER ${LV_COMPILE_OPTIONS} ${LV_PP_OPTIONS} "${LV_CFLAGS}" -I ${LV_BINDINGS_DIR}/pycparser/utils/fake_libc_include ${MICROPY_CPP_FLAGS} ${LV_INPUT} > ${LV_PP}
        DEPENDS
            ${LV_INPUT}
            ${LV_DEPENDS}
            ${LV_BINDINGS_DIR}/pycparser/utils/fake_libc_include
        IMPLICIT_DEPENDS
            C ${LV_INPUT}
        VERBATIM
        COMMAND_EXPAND_LISTS
    )

    if(ESP_PLATFORM)
        target_compile_options(${COMPONENT_LIB} PRIVATE ${LV_COMPILE_OPTIONS})
    else()
        target_compile_options(usermod_lv_bindings INTERFACE ${LV_COMPILE_OPTIONS})
    endif()

    if (DEFINED LV_FILTER)

        set(LV_PP_FILTERED ${LV_PP}.filtered)
        set(LV_AWK_CONDITION)
        foreach(_f ${LV_FILTER})
            string(APPEND LV_AWK_CONDITION "\$3!~\"${_f}\" && ")
        endforeach()
        string(APPEND LV_AWK_COMMAND "\$1==\"#\"{p=(${LV_AWK_CONDITION} 1)} p{print}")

        # message("AWK COMMAND: ${LV_AWK_COMMAND}")

        add_custom_command(
            OUTPUT
                ${LV_PP_FILTERED}
            COMMAND
                ${AWK} ${LV_AWK_COMMAND} ${LV_PP} > ${LV_PP_FILTERED}
            DEPENDS
                ${LV_PP}
            VERBATIM
            COMMAND_EXPAND_LISTS
        )
    else()
        set(LV_PP_FILTERED ${LV_PP})
    endif()

    message("-- Generating MicroPython code: ${LV_OUTPUT}")

    add_custom_command(
        OUTPUT
            ${LV_OUTPUT}
        COMMAND
            ${Python3_EXECUTABLE} ${LV_BINDINGS_DIR}/gen/gen_mpy.py ${LV_GEN_OPTIONS} -MD ${LV_MPY_METADATA} -E ${LV_PP_FILTERED} ${LV_INPUT} > ${LV_OUTPUT} || (rm -f ${LV_OUTPUT} && /bin/false)
        DEPENDS
            ${LV_BINDINGS_DIR}/gen/gen_mpy.py
            ${LV_PP_FILTERED}
        COMMAND_EXPAND_LISTS
    )

endfunction()

# Definitions for specific bindings

set(LVGL_DIR ${LV_BINDINGS_DIR}/lvgl)

set(LV_MP ${CMAKE_BINARY_DIR}/lv_mp.c)
if(ESP_PLATFORM)
    set(LV_ESPIDF ${CMAKE_BINARY_DIR}/lv_espidf.c)
endif()

if(ESP_PLATFORM)
    set(LVGL_IDF_COMPONENTS)

    if(IDF_TARGET STREQUAL "esp32c3")
        list(APPEND LVGL_IDF_COMPONENTS esp_lcd)
        list(APPEND LVGL_IDF_COMPONENTS usb)
    elseif(IDF_TARGET STREQUAL "esp32s2")
        list(APPEND LVGL_IDF_COMPONENTS esp_lcd)
        list(APPEND LVGL_IDF_COMPONENTS usb)
    elseif(IDF_TARGET STREQUAL "esp32s3")
        list(APPEND LVGL_IDF_COMPONENTS esp_lcd)
        list(APPEND LVGL_IDF_COMPONENTS usb)
    endif()

    foreach(comp ${LVGL_IDF_COMPONENTS})
        list(APPEND IDF_COMPONENTS ${comp})
        micropy_gather_target_properties(__idf_${comp})
    endforeach()

endif()

# Function for creating all specific bindings

function(all_lv_bindings)

    # LVGL bindings

    file(GLOB_RECURSE LVGL_HEADERS ${LVGL_DIR}/src/*.h ${LV_BINDINGS_DIR}/lv_conf.h)
    lv_bindings(
        OUTPUT
            ${LV_MP}
        INPUT
            ${LVGL_DIR}/lvgl.h
        DEPENDS
            ${LVGL_HEADERS}
        GEN_OPTIONS
            -M lvgl -MP lv
    )

    # ESPIDF bindings

    if(ESP_PLATFORM)
        set(GEN_MPY_OPTIONS
           -M espidf
        )

        if(IDF_TARGET STREQUAL "esp32c3")
            LIST(APPEND GEN_MPY_OPTIONS
                -N gpio_force_hold_all
                -N gpio_force_unhold_all
                -N esp_eth_phy_new_lan87xx
                -N esp_eth_phy_new_lan8720
                -N xt_clock_freq
                -N lldesc_build_chain
                -N esp_eth_phy_new_ksz8081
                -N esp_eth_phy_new_ksz8041
                -N esp_eth_phy_new_dp83848
                -N esp_eth_phy_new_rtl8201
                -N esp_eth_phy_new_ip101
                -N lldesc_set_owner
                -N lldesc_num2link
            )
        elseif(IDF_TARGET STREQUAL "esp32s2")
            LIST(APPEND GEN_MPY_OPTIONS
                -N gpio_force_hold_all
                -N gpio_force_unhold_all
                -N esp_eth_phy_new_lan87xx
                -N esp_eth_phy_new_lan8720
                -N xt_clock_freq
                -N lldesc_build_chain
                -N esp_eth_phy_new_ksz8081
                -N esp_eth_phy_new_ksz8041
                -N esp_eth_phy_new_dp83848
                -N esp_eth_phy_new_rtl8201
                -N esp_eth_phy_new_ip101
                -N lldesc_set_owner
                -N lldesc_num2link
            )
        elseif(IDF_TARGET STREQUAL "esp32s3")
            LIST(APPEND GEN_MPY_OPTIONS
                -N gpio_force_hold_all
                -N gpio_force_unhold_all
                -N esp_eth_phy_new_lan87xx
                -N esp_eth_phy_new_lan8720
                -N xt_clock_freq
                -N lldesc_build_chain
                -N esp_eth_phy_new_ksz8081
                -N esp_eth_phy_new_ksz8041
                -N esp_eth_phy_new_dp83848
                -N esp_eth_phy_new_rtl8201
                -N esp_eth_phy_new_ip101
                -N lldesc_set_owner
                -N lldesc_num2link
            )
        endif()

        file(GLOB_RECURSE LV_ESPIDF_HEADERS ${IDF_PATH}/components/*.h ${LV_BINDINGS_DIR}/driver/esp32/*.h)
        lv_bindings(
            OUTPUT
                ${LV_ESPIDF}
            INPUT
                ${LV_BINDINGS_DIR}/driver/esp32/espidf.h
            DEPENDS
                ${LV_ESPIDF_HEADERS}
            GEN_OPTIONS
                 ${GEN_MPY_OPTIONS}
            FILTER
                i2s_ll.h
                i2s_hal.h
                esp_intr_alloc.h
                soc/spi_periph.h
                rom/ets_sys.h
                soc/sens_struct.h
                soc/rtc.h
                driver/periph_ctrl.h
        )
    endif(ESP_PLATFORM)

endfunction()

# Add includes to CMake component

set(LV_INCLUDE
    ${LV_BINDINGS_DIR}
)

# Add sources to CMake component

set(LV_SRC
    ${LV_MP}
)

if(ESP_PLATFORM)
    LIST(APPEND LV_SRC
        ${LV_BINDINGS_DIR}/driver/esp32/espidf.c
        ${LV_BINDINGS_DIR}/driver/esp32/modrtch.c
        ${LV_BINDINGS_DIR}/driver/esp32/sh2lib.c
        ${LV_ESPIDF}
    )
endif(ESP_PLATFORM)
