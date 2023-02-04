/**
 * This file defines the Micorpython API to ESP-IDF
 * It is used as input to gen_mpy.py to create a micropython module
 **/
#if __has_include("esp_idf_version.h")
#   include "esp_idf_version.h"
#endif

// Disable some macros and includes that make pycparser choke

#ifdef PYCPARSER
#define __attribute__(x)
#define _Static_assert(x,y)
#define __extension__
#define _SOC_IO_MUX_REG_H_
#define _SYS_REENT_H_
#define PORTMACRO_H
#define PORTABLE_H
#define INC_FREERTOS_H
#define QUEUE_H
#define SEMAPHORE_H
#define XTENSA_HAL_H
#define _SOC_I2S_STRUCT_H_
#define XTRUNTIME_H
#define _SOC_SPI_STRUCT_H_
#define _SOC_RTC_CNTL_STRUCT_H_
#define __XTENSA_API_H__
#define _SOC_GPIO_STRUCT_H_
#define _SOC_RTC_IO_STRUCT_H_
#define _SOC_PCNT_STRUCT_H_
#define _SYS_FCNTL_H_
#define __SYS_ARCH_H__
#define LIST_H
#define INC_TASK_H
#define LWIP_HDR_NETIF_H
#define ESP_EVENT_H_
#define __SNTP_H__
#define XTENSA_CONFIG_CORE_H
#define _SOC_SPI_MEM_STRUCT_H_

typedef int	BaseType_t;
typedef unsigned int	UBaseType_t;
typedef void* system_event_t;
typedef void *intr_handle_t;

// Exclude SOC just because it contains large structs that don't interest the user
#define _SOC_SPI_PERIPH_H_
typedef void *spi_dev_t;

// TODO: Check why lldesc_t causes inifinite recursion on gen_mpy.py
#define _ROM_LLDESC_H_
typedef void *lldesc_t;

// FreeRTOS definitions we want available on Micropython
#include <stdint.h>
typedef uint32_t TickType_t;
typedef void * TaskHandle_t;
static inline uint32_t xPortGetCoreID();
UBaseType_t uxTaskPriorityGet( TaskHandle_t xTask );

// Micropython specific types
typedef void *mp_obj_t;

static inline void SPH0645_WORKAROUND(int i2s_num);
static inline void get_ccount(int *ccount);

// Memory management helper functions
void * memcpy ( void * destination, const void * source, size_t num );
void * memset ( void * ptr, int value, size_t num );


#else // PYCPARSER


/////////////////////////////////////////////////////////////////////////////////////////////
// A workaround for SPH0645 I2S, see:
// - https://hackaday.io/project/162059-street-sense/log/160705-new-i2s-microphone/discussion-124677
// - https://www.esp32.com/viewtopic.php?t=4997#p45366
// Since reg access is based on macros, this cannot currently be directly implemented in Micropython

#include "soc/i2s_reg.h" // for SPH0645_WORKAROUND

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
    #define I2S_TIMING_REG I2S_TX_TIMING_REG
    #define I2S_CONF_REG I2S_TX_CONF_REG
#endif


static inline void SPH0645_WORKAROUND(int i2s_num)
{
    REG_SET_BIT( I2S_TIMING_REG(i2s_num), BIT(9));
    REG_SET_BIT( I2S_CONF_REG(i2s_num), I2S_RX_MSB_SHIFT);
}

/////////////////////////////////////////////////////////////////////////////////////////////
// Helper function to measure CPU cycles
//
static inline void get_ccount(int *ccount)
{
    asm volatile("rsr.ccount %0" : "=a"(*ccount));
}


#endif //PYCPARSER

// The following includes are the source of the esp-idf micropython module.
// All included files are API we want to include in the module

#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 4
#   if CONFIG_IDF_TARGET_ESP32
#   include "esp32/clk.h"
#   elif CONFIG_IDF_TARGET_ESP32S2
#   include "esp32s2/clk.h"
#   elif CONFIG_IDF_TARGET_ESP32S3
#   include "esp32s3/clk.h"
#   elif CONFIG_IDF_TARGET_ESP32C3
#   include "esp32c3/clk.h"
#   elif CONFIG_IDF_TARGET_ESP32H2
#   include "esp32h2/clk.h"
#   else // CONFIG_IDF_TARGET_* not defined
#   include "esp32/clk.h"
#   endif
#else
#   include "esp_clk.h"
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32H2
    #ifdef SOC_GPIO_SUPPORT_FORCE_HOLD
        #undef SOC_GPIO_SUPPORT_FORCE_HOLD
    #endif
#endif

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "driver/i2s.h"
#include "driver/pcnt.h"
#include "mdns.h"
#include "esp_http_client.h"
#include "sh2lib.h"


// This will allow for writing keyboard drivers and/or mouse drivers in Python
#if CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C6
#include "usb/usb_helpers.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "usb/usb_types_stack.h"



//ENUM_USB_HOST_LIB_EVENT_FLAGS
enum {
    ENUM_USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS = USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS,
    ENUM_USB_HOST_LIB_EVENT_FLAGS_ALL_FREE = USB_HOST_LIB_EVENT_FLAGS_ALL_FREE
};

//USB_EP_DESC_SIZE
enum {
    ENUM_USB_EP_DESC_SIZE = USB_EP_DESC_SIZE,
};

//USB_INTF_DESC_SIZE
enum {
    ENUM_USB_INTF_DESC_SIZE = USB_INTF_DESC_SIZE,
};

//USB_IAD_DESC_SIZE
enum {
    ENUM_USB_IAD_DESC_SIZE = USB_IAD_DESC_SIZE,
};

//USB_STR_DESC_SIZE
enum {
    ENUM_USB_STR_DESC_SIZE = USB_STR_DESC_SIZE,
};

//ENUM_USB_CLASS
enum {
    ENUM_USB_CLASS_PER_INTERFACE = USB_CLASS_PER_INTERFACE,
    ENUM_USB_CLASS_AUDIO = USB_CLASS_AUDIO,
    ENUM_USB_CLASS_COMM = USB_CLASS_COMM,
    ENUM_USB_CLASS_HID = USB_CLASS_HID,
    ENUM_USB_CLASS_PHYSICAL = USB_CLASS_PHYSICAL,
    ENUM_USB_CLASS_STILL_IMAGE = USB_CLASS_STILL_IMAGE,
    ENUM_USB_CLASS_PRINTER = USB_CLASS_PRINTER,
    ENUM_USB_CLASS_MASS_STORAGE = USB_CLASS_MASS_STORAGE,
    ENUM_USB_CLASS_HUB = USB_CLASS_HUB,
    ENUM_USB_CLASS_CDC_DATA = USB_CLASS_CDC_DATA,
    ENUM_USB_CLASS_CSCID = USB_CLASS_CSCID,
    ENUM_USB_CLASS_CONTENT_SEC = USB_CLASS_CONTENT_SEC,
    ENUM_USB_CLASS_VIDEO = USB_CLASS_VIDEO,
    ENUM_USB_CLASS_WIRELESS_CONTROLLER = USB_CLASS_WIRELESS_CONTROLLER,
    ENUM_USB_CLASS_PERSONAL_HEALTHCARE = USB_CLASS_PERSONAL_HEALTHCARE,
    ENUM_USB_CLASS_AUDIO_VIDEO = USB_CLASS_AUDIO_VIDEO,
    ENUM_USB_CLASS_BILLBOARD = USB_CLASS_BILLBOARD,
    ENUM_USB_CLASS_USB_TYPE_C_BRIDGE = USB_CLASS_USB_TYPE_C_BRIDGE,
    ENUM_USB_CLASS_MISC = USB_CLASS_MISC,
    ENUM_USB_CLASS_APP_SPEC = USB_CLASS_APP_SPEC,
    ENUM_USB_CLASS_VENDOR_SPEC = USB_CLASS_VENDOR_SPEC
};

//USB_SUBCLASS_VENDOR_SPEC
enum {
    ENUM_USB_SUBCLASS_VENDOR_SPEC = USB_SUBCLASS_VENDOR_SPEC
};

//USB_CONFIG_DESC_SIZE
enum {
    ENUM_USB_CONFIG_DESC_SIZE = USB_CONFIG_DESC_SIZE
};

//USB_B_ENDPOINT_ADDRESS_EP
enum {
    ENUM_USB_B_ENDPOINT_ADDRESS_EP_NUM_MASK = USB_B_ENDPOINT_ADDRESS_EP_NUM_MASK,
    ENUM_USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK = USB_B_ENDPOINT_ADDRESS_EP_DIR_MASK
};

//USB_B_DESCRIPTOR_TYPE
enum {
    ENUM_USB_B_DESCRIPTOR_TYPE_DEVICE = USB_B_DESCRIPTOR_TYPE_DEVICE,
    ENUM_USB_B_DESCRIPTOR_TYPE_CONFIGURATION = USB_B_DESCRIPTOR_TYPE_CONFIGURATION,
    ENUM_USB_B_DESCRIPTOR_TYPE_STRING = USB_B_DESCRIPTOR_TYPE_STRING,
    ENUM_USB_B_DESCRIPTOR_TYPE_INTERFACE = USB_B_DESCRIPTOR_TYPE_INTERFACE,
    ENUM_USB_B_DESCRIPTOR_TYPE_ENDPOINT = USB_B_DESCRIPTOR_TYPE_ENDPOINT,
    ENUM_USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER = USB_B_DESCRIPTOR_TYPE_DEVICE_QUALIFIER,
    ENUM_USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION = USB_B_DESCRIPTOR_TYPE_OTHER_SPEED_CONFIGURATION,
    ENUM_USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER = USB_B_DESCRIPTOR_TYPE_INTERFACE_POWER,
    ENUM_USB_B_DESCRIPTOR_TYPE_OTG = USB_B_DESCRIPTOR_TYPE_OTG,
    ENUM_USB_B_DESCRIPTOR_TYPE_DEBUG = USB_B_DESCRIPTOR_TYPE_DEBUG,
    ENUM_USB_B_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION = USB_B_DESCRIPTOR_TYPE_INTERFACE_ASSOCIATION,
    ENUM_USB_B_DESCRIPTOR_TYPE_SECURITY = USB_B_DESCRIPTOR_TYPE_SECURITY,
    ENUM_USB_B_DESCRIPTOR_TYPE_KEY = USB_B_DESCRIPTOR_TYPE_KEY,
    ENUM_USB_B_DESCRIPTOR_TYPE_ENCRYPTION_TYPE = USB_B_DESCRIPTOR_TYPE_ENCRYPTION_TYPE,
    ENUM_USB_B_DESCRIPTOR_TYPE_BOS = USB_B_DESCRIPTOR_TYPE_BOS,
    ENUM_USB_B_DESCRIPTOR_TYPE_DEVICE_CAPABILITY = USB_B_DESCRIPTOR_TYPE_DEVICE_CAPABILITY,
    ENUM_USB_B_DESCRIPTOR_TYPE_WIRELESS_ENDPOINT_COMP = USB_B_DESCRIPTOR_TYPE_WIRELESS_ENDPOINT_COMP,
    ENUM_USB_B_DESCRIPTOR_TYPE_WIRE_ADAPTER = USB_B_DESCRIPTOR_TYPE_WIRE_ADAPTER,
    ENUM_USB_B_DESCRIPTOR_TYPE_RPIPE = USB_B_DESCRIPTOR_TYPE_RPIPE,
    ENUM_USB_B_DESCRIPTOR_TYPE_CS_RADIO_CONTROL = USB_B_DESCRIPTOR_TYPE_CS_RADIO_CONTROL,
    ENUM_USB_B_DESCRIPTOR_TYPE_PIPE_USAGE = USB_B_DESCRIPTOR_TYPE_PIPE_USAGE,
};

//USB_B_REQUEST
enum {
    ENUM_USB_B_REQUEST_GET_STATUS = USB_B_REQUEST_GET_STATUS,
    ENUM_USB_B_REQUEST_CLEAR_FEATURE = USB_B_REQUEST_CLEAR_FEATURE,
    ENUM_USB_B_REQUEST_SET_FEATURE = USB_B_REQUEST_SET_FEATURE,
    ENUM_USB_B_REQUEST_SET_ADDRESS = USB_B_REQUEST_SET_ADDRESS,
    ENUM_USB_B_REQUEST_GET_DESCRIPTOR = USB_B_REQUEST_GET_DESCRIPTOR,
    ENUM_USB_B_REQUEST_SET_DESCRIPTOR = USB_B_REQUEST_SET_DESCRIPTOR,
    ENUM_USB_B_REQUEST_GET_CONFIGURATION = USB_B_REQUEST_GET_CONFIGURATION,
    ENUM_USB_B_REQUEST_SET_CONFIGURATION = USB_B_REQUEST_SET_CONFIGURATION,
    ENUM_USB_B_REQUEST_GET_INTERFACE = USB_B_REQUEST_GET_INTERFACE,
    ENUM_USB_B_REQUEST_SET_INTERFACE = USB_B_REQUEST_SET_INTERFACE,
    ENUM_USB_B_REQUEST_SYNCH_FRAME = USB_B_REQUEST_SYNCH_FRAME
};

//USB_SETUP_PACKET_SIZE
enum {
    ENUM_USB_SETUP_PACKET_SIZE = USB_SETUP_PACKET_SIZE
};

//USB_BM_ATTRIBUTES
enum {
    ENUM_USB_BM_ATTRIBUTES_ONE = USB_BM_ATTRIBUTES_ONE,
    ENUM_USB_BM_ATTRIBUTES_SELFPOWER = USB_BM_ATTRIBUTES_SELFPOWER,
    ENUM_USB_BM_ATTRIBUTES_WAKEUP = USB_BM_ATTRIBUTES_WAKEUP,
    ENUM_USB_BM_ATTRIBUTES_BATTERY = USB_BM_ATTRIBUTES_BATTERY
};

//USB_BM_ATTRIBUTES_XFER
enum {
    ENUM_USB_BM_ATTRIBUTES_XFER_TYPE_MASK = USB_BM_ATTRIBUTES_XFERTYPE_MASK,
    ENUM_USB_BM_ATTRIBUTES_XFER_CONTROL = USB_BM_ATTRIBUTES_XFER_CONTROL,
    ENUM_USB_BM_ATTRIBUTES_XFER_ISOC = USB_BM_ATTRIBUTES_XFER_ISOC,
    ENUM_USB_BM_ATTRIBUTES_XFER_BULK = USB_BM_ATTRIBUTES_XFER_BULK,
    ENUM_USB_BM_ATTRIBUTES_XFER_INT = USB_BM_ATTRIBUTES_XFER_INT
};

//USB_BM_ATTRIBUTES_SYNC
enum {
    ENUM_USB_BM_ATTRIBUTES_SYNC_TYPE_MASK = USB_BM_ATTRIBUTES_SYNCTYPE_MASK,
    ENUM_USB_BM_ATTRIBUTES_SYNC_NONE = USB_BM_ATTRIBUTES_SYNC_NONE ,
    ENUM_USB_BM_ATTRIBUTES_SYNC_ASYNC = USB_BM_ATTRIBUTES_SYNC_ASYNC,
    ENUM_USB_BM_ATTRIBUTES_SYNC_ADAPTIVE = USB_BM_ATTRIBUTES_SYNC_ADAPTIVE,
    ENUM_USB_BM_ATTRIBUTES_SYNC_SYNC = USB_BM_ATTRIBUTES_SYNC_SYNC
};

//USB_BM_ATTRIBUTES_USAGE
enum {
    ENUM_USB_BM_ATTRIBUTES_USAGE_TYPE_MASK = USB_BM_ATTRIBUTES_USAGETYPE_MASK,
    ENUM_USB_BM_ATTRIBUTES_USAGE_DATA = USB_BM_ATTRIBUTES_USAGE_DATA,
    ENUM_USB_BM_ATTRIBUTES_USAGE_FEEDBACK = USB_BM_ATTRIBUTES_USAGE_FEEDBACK,
    ENUM_USB_BM_ATTRIBUTES_USAGE_IMPLICIT_FB = USB_BM_ATTRIBUTES_USAGE_IMPLICIT_FB
};

//USB_BM_REQUEST_TYPE
enum {
    ENUM_USB_BM_REQUEST_TYPE_DIR_OUT = USB_BM_REQUEST_TYPE_DIR_OUT,
    ENUM_USB_BM_REQUEST_TYPE_DIR_IN = USB_BM_REQUEST_TYPE_DIR_IN,
    ENUM_USB_BM_REQUEST_TYPE_TYPE_STANDARD = USB_BM_REQUEST_TYPE_TYPE_STANDARD,
    ENUM_USB_BM_REQUEST_TYPE_TYPE_CLASS = USB_BM_REQUEST_TYPE_TYPE_CLASS,
    ENUM_USB_BM_REQUEST_TYPE_TYPE_VENDOR = USB_BM_REQUEST_TYPE_TYPE_VENDOR,
    ENUM_USB_BM_REQUEST_TYPE_TYPE_RESERVED = USB_BM_REQUEST_TYPE_TYPE_RESERVED,
    ENUM_USB_BM_REQUEST_TYPE_TYPE_MASK = USB_BM_REQUEST_TYPE_TYPE_MASK,
    ENUM_USB_BM_REQUEST_TYPE_RECIP_DEVICE = USB_BM_REQUEST_TYPE_RECIP_DEVICE,
    ENUM_USB_BM_REQUEST_TYPE_RECIP_INTERFACE = USB_BM_REQUEST_TYPE_RECIP_INTERFACE,
    ENUM_USB_BM_REQUEST_TYPE_RECIP_ENDPOINT = USB_BM_REQUEST_TYPE_RECIP_ENDPOINT,
    ENUM_USB_BM_REQUEST_TYPE_RECIP_OTHER = USB_BM_REQUEST_TYPE_RECIP_OTHER,
    ENUM_USB_BM_REQUEST_TYPE_RECIP_MASK = USB_BM_REQUEST_TYPE_RECIP_MASK
};

//USB_W_VALUE_DT
enum {
    ENUM_USB_W_VALUE_DT_DEVICE = USB_W_VALUE_DT_DEVICE,
    ENUM_USB_W_VALUE_DT_CONFIG = USB_W_VALUE_DT_CONFIG,
    ENUM_USB_W_VALUE_DT_STRING = USB_W_VALUE_DT_STRING,
    ENUM_USB_W_VALUE_DT_INTERFACE = USB_W_VALUE_DT_INTERFACE,
    ENUM_USB_W_VALUE_DT_ENDPOINT = USB_W_VALUE_DT_ENDPOINT,
    ENUM_USB_W_VALUE_DT_DEVICE_QUALIFIER = USB_W_VALUE_DT_DEVICE_QUALIFIER,
    ENUM_USB_W_VALUE_DT_OTHER_SPEED_CONFIG = USB_W_VALUE_DT_OTHER_SPEED_CONFIG,
    ENUM_USB_W_VALUE_DT_INTERFACE_POWER = USB_W_VALUE_DT_INTERFACE_POWER
};

//USB_STANDARD_DESC_SIZE
enum {
    ENUM_USB_STANDARD_DESC_SIZE = USB_STANDARD_DESC_SIZE
};

//USB_DEVICE_DESC_SIZE
enum {
    ENUM_USB_DEVICE_DESC_SIZE = USB_DEVICE_DESC_SIZE
};

//USB_TRANSFER_FLAG_ZERO_PACK
enum {
    ENUM_USB_TRANSFER_FLAG_ZERO_PACK = USB_TRANSFER_FLAG_ZERO_PACK,
};

enum {
#if CONFIG_IDF_TARGET_ESP32C3
    ENUM_USB_DATA_PIN_MINUS = 18,
    ENUM_USB_DATA_PIN_PLUS = 19
#elif CONFIG_IDF_TARGET_ESP32C6
    ENUM_USB_DATA_PIN_MINUS = 12,
    ENUM_USB_DATA_PIN_PLUS = 13
#else // CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3
    ENUM_USB_DATA_PIN_MINUS = 19,
    ENUM_USB_DATA_PIN_PLUS = 20
#endif
};


typedef struct {
    uint8_t *const data_buffer;                     //< Pointer to data buffer
    const size_t data_buffer_size;                  //< Size of the data buffer in bytes
    int num_bytes;                                  //< Number of bytes to transfer.
                                                    //     Control transfers should include the size of the setup packet.
                                                    //     Isochronous transfer should be the total transfer size of all packets.
                                                    //     For non-control IN transfers, num_bytes should be an integer multiple of MPS.
    int actual_num_bytes;                           //< Actual number of bytes transferred
    uint32_t flags;                                 //< Transfer flags
    usb_device_handle_t device_handle;              //< Device handle
    uint8_t bEndpointAddress;                       //< Endpoint Address
    usb_transfer_status_t status;                   //< Status of the transfer
    uint32_t timeout_ms;                            //< Timeout (in milliseconds) of the packet (currently not supported yet)
    usb_transfer_cb_t callback;                     //< Transfer callback
    void *context;                                  //< Context variable for transfer to associate transfer with something
    const int num_isoc_packets;                     //< Only relevant to Isochronous. Number of service periods (i.e., intervals) to transfer data buffer over.
    usb_isoc_packet_desc_t isoc_packet_desc[];      //< Descriptors for each Isochronous packet
} usb_transfer_s;


/*
This is a dummy function in order to get gen_mpy to populate different
structures and enumerations. The only way it works is they have to be added to
a function.
*/
static inline void usb_dummy_func(usb_iad_desc_t *param1, usb_transfer_s *param5){}


#endif

//this exposes the esp_lcd functions, structures and enumerations to Micropython

#if defined(SOC_LCD_RGB_SUPPORTED) && SOC_LCD_RGB_SUPPORTED
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_rgb.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_interface.h"
#include "esp_private/gdma.h"
#include "esp_pm.h"
#include "hal/dma_types.h"
#include "hal/lcd_hal.h"
#include "hal/lcd_types.h"

typedef struct {
    esp_lcd_panel_t base;  // Base class of generic lcd panel
    int panel_id;          // LCD panel ID
    lcd_hal_context_t hal; // Hal layer object
    size_t data_width;     // Number of data lines (e.g. for RGB565, the data width is 16)
    size_t sram_trans_align;  // Alignment for framebuffer that allocated in SRAM
    size_t psram_trans_align; // Alignment for framebuffer that allocated in PSRAM
    int disp_gpio_num;     // Display control GPIO, which is used to perform action like "disp_off"
    intr_handle_t intr;    // LCD peripheral interrupt handle
    esp_pm_lock_handle_t pm_lock; // Power management lock
    size_t num_dma_nodes;  // Number of DMA descriptors that used to carry the frame buffer
    uint8_t *fb;           // Frame buffer
    size_t fb_size;        // Size of frame buffer
    int data_gpio_nums[SOC_LCD_RGB_DATA_WIDTH]; // GPIOs used for data lines, we keep these GPIOs for action like "invert_color"
    uint32_t src_clk_hz;   // Peripheral source clock resolution
    esp_lcd_rgb_timing_t timings;   // RGB timing parameters (e.g. pclk, sync pulse, porch width)
    gdma_channel_handle_t dma_chan; // DMA channel handle
    esp_lcd_rgb_panel_frame_trans_done_cb_t on_frame_trans_done; // Callback, invoked after frame trans done
    void *user_ctx;                // Reserved user's data of callback functions
    int x_gap;                      // Extra gap in x coordinate, it's used when calculate the flush window
    int y_gap;                      // Extra gap in y coordinate, it's used when calculate the flush window
    int lcd_clk_flags;              // LCD clock calculation flags
    struct {
        unsigned int disp_en_level: 1; // The level which can turn on the screen by `disp_gpio_num`
        unsigned int stream_mode: 1;   // If set, the LCD transfers data continuously, otherwise, it stops refreshing the LCD when transaction done
        unsigned int fb_in_psram: 1;   // Whether the frame buffer is in PSRAM
    } flags;
    dma_descriptor_t dma_nodes[]; // DMA descriptor pool of size `num_dma_nodes`
} esp_rgb_panel_t;


esp_rgb_panel_t *esp_rgb_panel_container(esp_lcd_panel_handle_t panel_handle);

#endif

/////////////////////////////////////////////////////////////////////////////////////////////
// Helper function to register HTTP event handler
// Needed to fulfill gen_mpy.py callback conventions
//
static inline void esp_http_client_register_event_handler(esp_http_client_config_t *config, http_event_handle_cb http_event_handler, void *user_data)
{
    config->event_handler = http_event_handler;
    config->user_data = user_data;
}

// We don't want the whole FreeRTOS, only selected functions

void task_delay_ms(int ms);

// The binding only publishes structs that are used in some function. We need spi_transaction_ext_t
// TOOD: Find some way to mark structs for binding export instead of new function.
static inline void set_spi_transaction_ext(
        spi_transaction_ext_t *ext_trans,
        spi_transaction_t *trans,
        uint8_t command_bits,
        uint8_t address_bits){
    ext_trans->base = *trans;
    ext_trans->command_bits = command_bits;
    ext_trans->address_bits = address_bits;
}

// Wrapper for safe ISR callbacks from micropython
// Need to call both spi_transaction_set_cb and set spi_pre/post_cb_isr!

// Use this to set pre/post callbacks for spi transaction.
// pre_cb/post_cb should be either a callable object or "None".
// Result should be set to spi_transaction_t user field.
// Allocates RAM.

void *spi_transaction_set_cb(mp_obj_t pre_cb, mp_obj_t post_cb);

// These functions can be set into pre_cb/post_cb of spi_device_interface_config_t

void ex_spi_pre_cb_isr(spi_transaction_t *trans);
void ex_spi_post_cb_isr(spi_transaction_t *trans);

// Useful constants

#define EXPORT_CONST_INT(int_value) enum {ENUM_##int_value = int_value}

//#if defined(ESP_IDF_VERSION_MAJOR) && ESP_IDF_VERSION_MAJOR >= 4

// SPI HOST enum was changed to macros on v4
// ESP

enum {
  ENUM_ESP_OK = 0,
  ENUM_ESP_BAD_DATA_LEN = 0xC0,
  ENUM_ESP_BAD_DATA_CHECKSUM,
  ENUM_ESP_BAD_BLOCKSIZE,
  ENUM_ESP_INVALID_COMMAND,
  ENUM_ESP_FAILED_SPI_OP,
  ENUM_ESP_FAILED_SPI_UNLOCK,
  ENUM_ESP_NOT_IN_FLASH_MODE,
  ENUM_ESP_INFLATE_ERROR,
  ENUM_ESP_NOT_ENOUGH_DATA,
  ENUM_ESP_TOO_MUCH_DATA,
  ENUM_ESP_CMD_NOT_IMPLEMENTED = 0xFF
};


enum {
    ENUM_SPI_HOST = SPI1_HOST,
#ifdef CONFIG_IDF_TARGET_ESP32
    ENUM_HSPI_HOST = SPI2_HOST,
    ENUM_VSPI_HOST = SPI3_HOST
#else
    ENUM_VSPI_HOST = SPI2_HOST,
    ENUM_HSPI_HOST = SPI3_HOST
#endif
};
//#endif



enum {
    ENUM_portMAX_DELAY = portMAX_DELAY
};

enum {
    ENUM_I2S_PIN_NO_CHANGE = I2S_PIN_NO_CHANGE
};

enum {
    ENUM_SPI_DEVICE_TXBIT_LSBFIRST = SPI_DEVICE_TXBIT_LSBFIRST,
    ENUM_SPI_DEVICE_RXBIT_LSBFIRST = SPI_DEVICE_RXBIT_LSBFIRST,
    ENUM_SPI_DEVICE_BIT_LSBFIRST = SPI_DEVICE_BIT_LSBFIRST,
    ENUM_SPI_DEVICE_3WIRE = SPI_DEVICE_3WIRE,
    ENUM_SPI_DEVICE_POSITIVE_CS = SPI_DEVICE_POSITIVE_CS,
    ENUM_SPI_DEVICE_HALFDUPLEX = SPI_DEVICE_HALFDUPLEX,
    ENUM_SPI_DEVICE_NO_DUMMY = SPI_DEVICE_NO_DUMMY,
    ENUM_SPI_DEVICE_CLK_AS_CS = SPI_DEVICE_CLK_AS_CS,
};

enum {
    ENUM_SPI_TRANS_MODE_DIO = SPI_TRANS_MODE_DIO,
    ENUM_SPI_TRANS_MODE_QIO = SPI_TRANS_MODE_QIO,
    ENUM_SPI_TRANS_MODE_DIOQIO_ADDR = SPI_TRANS_MODE_DIOQIO_ADDR,
    ENUM_SPI_TRANS_USE_RXDATA = SPI_TRANS_USE_RXDATA,
    ENUM_SPI_TRANS_USE_TXDATA = SPI_TRANS_USE_TXDATA,
    ENUM_SPI_TRANS_VARIABLE_CMD = SPI_TRANS_VARIABLE_CMD,
    ENUM_SPI_TRANS_VARIABLE_ADDR = SPI_TRANS_VARIABLE_ADDR,
};

enum {
    ENUM_MALLOC_CAP_EXEC = MALLOC_CAP_EXEC,
    ENUM_MALLOC_CAP_32BIT = MALLOC_CAP_32BIT,
    ENUM_MALLOC_CAP_8BIT = MALLOC_CAP_8BIT,
    ENUM_MALLOC_CAP_DMA = MALLOC_CAP_DMA,
    ENUM_MALLOC_CAP_SPIRAM = MALLOC_CAP_SPIRAM,
    ENUM_MALLOC_CAP_INTERNAL = MALLOC_CAP_INTERNAL,
    ENUM_MALLOC_CAP_DEFAULT = MALLOC_CAP_DEFAULT,
    // Missing on espidf v4.02:
    // ENUM_MALLOC_CAP_IRAM_8BIT = MALLOC_CAP_IRAM_8BIT,
    ENUM_MALLOC_CAP_INVALID = MALLOC_CAP_INVALID,
};

enum {
    ENUM_ESP_TASK_PRIO_MAX = ESP_TASK_PRIO_MAX,
    ENUM_ESP_TASK_PRIO_MIN = ESP_TASK_PRIO_MIN,
};

/////////////////////////////////////////////////////////////////////////////////////////////
// ili9xxx flush and ISR in C
//
// disp_drv->user_data should be a dict that contains dc and spi, setup by micropython.
// like this: "self.disp_drv.user_data = {'dc': self.dc, 'spi': self.spi, 'dt': display_type}"


void ili9xxx_post_cb_isr(spi_transaction_t *trans);

void ili9xxx_flush(void *disp_drv, const void *area, void *color_p);
