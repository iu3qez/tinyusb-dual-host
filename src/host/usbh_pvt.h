/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#ifndef TUSB_USBH_PVT_H_
#define TUSB_USBH_PVT_H_

#include "usbh.h"
#include "hcd.h"
#include "osal/osal.h"
#include "common/tusb_private.h"

#ifdef __cplusplus
 extern "C" {
#endif

#define TU_LOG_USBH(...)      TU_LOG(CFG_TUH_LOG_LEVEL, __VA_ARGS__)
#define TU_LOG_MEM_USBH(...)  TU_LOG_MEM(CFG_TUH_LOG_LEVEL, __VA_ARGS__)
#define TU_LOG_BUF_USBH(...)  TU_LOG_BUF(CFG_TUH_LOG_LEVEL, __VA_ARGS__)
#define TU_LOG_INT_USBH(...)  TU_LOG_INT(CFG_TUH_LOG_LEVEL, __VA_ARGS__)
#define TU_LOG_HEX_USBH(...)  TU_LOG_HEX(CFG_TUH_LOG_LEVEL, __VA_ARGS__)

//--------------------------------------------------------------------+
// Class Driver API
//--------------------------------------------------------------------+

typedef struct {
  char const* name;
  bool (* const init       )(void);
  bool (* const deinit     )(void);
  bool (* const open       )(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const * itf_desc, uint16_t max_len);
  bool (* const set_config )(uint8_t dev_addr, uint8_t itf_num);
  bool (* const xfer_cb    )(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);
  void (* const close      )(uint8_t dev_addr);
} usbh_class_driver_t;

// Invoked when initializing host stack to get additional class drivers.
// Can be implemented by application to extend/overwrite class driver support.
// Note: The drivers array must be accessible at all time when stack is active
usbh_class_driver_t const* usbh_app_driver_get_cb(uint8_t* driver_count);

//--------------------------------------------------------------------+
// Instance Structure Definition
//--------------------------------------------------------------------+

#define TOTAL_DEVICES (CFG_TUH_DEVICE_MAX + CFG_TUH_HUB)

// Device structure
typedef struct {
  tuh_bus_info_t bus_info;  // Bus information (rhport, hub_addr, hub_port, speed)

  // Device Descriptor
  uint16_t bcdUSB;
  uint8_t bDeviceClass;
  uint8_t bDeviceSubClass;
  uint8_t bDeviceProtocol;
  uint8_t bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t iManufacturer;
  uint8_t iProduct;
  uint8_t iSerialNumber;
  uint8_t bNumConfigurations;

  // Device State
  struct TU_ATTR_PACKED {
    volatile uint8_t connected  : 1; // After 1st transfer
    volatile uint8_t addressed  : 1; // After SET_ADDR
    volatile uint8_t configured : 1; // After SET_CONFIG and all drivers are configured
    volatile uint8_t suspended  : 1; // Bus suspended
  };

  // Endpoint & Interface mapping
  uint8_t itf2drv[CFG_TUH_INTERFACE_MAX];  // map interface number to driver (0xff is invalid)
  uint8_t ep2drv[CFG_TUH_ENDPOINT_MAX][2]; // map endpoint to driver (0xff is invalid)

  // Endpoint status tracking
  tu_edpt_state_t ep_status[CFG_TUH_ENDPOINT_MAX][2];

#if CFG_TUH_API_EDPT_XFER
  // Endpoint transfer callbacks
  struct {
    tuh_xfer_cb_t complete_cb;
    uintptr_t user_data;
  } ep_callback[CFG_TUH_ENDPOINT_MAX][2];
#endif
} usbh_device_t;

// Control transfer info
typedef struct {
  uint8_t* buffer;
  tuh_xfer_cb_t complete_cb;
  uintptr_t user_data;

  volatile uint8_t stage;
  uint8_t daddr;
  volatile uint16_t actual_len;
  uint8_t failed_count;
} usbh_ctrl_xfer_info_t;

// Enumeration buffer
typedef struct {
  TUH_EPBUF_TYPE_DEF(tusb_control_request_t, request);
  TUH_EPBUF_DEF(ctrl, CFG_TUH_ENUMERATION_BUFSIZE);
} usbh_epbuf_t;

// USB Host Instance Structure
struct usbh_instance {
  uint8_t rhport;
  uint8_t controller_id;
  uint8_t enumerating_daddr;
  uint8_t attach_debouncing_bm;
  tuh_bus_info_t dev0_bus;
  usbh_ctrl_xfer_info_t ctrl_xfer;

  // Event queue and spinlock are defined at file scope (not embedded in struct)
  // Instance stores only pointers/references to them
  osal_queue_t event_queue;
  osal_spinlock_t* spin;

#if OSAL_MUTEX_REQUIRED
  osal_mutex_def_t mutex_def;
  osal_mutex_t mutex;
#endif

  usbh_device_t devices[TOTAL_DEVICES];
  CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN usbh_epbuf_t epbuf;

  bool initialized;
  bool running;
};

typedef struct usbh_instance usbh_instance_t;

//--------------------------------------------------------------------+
// Internal Helper Functions
//--------------------------------------------------------------------+

// Get rhport from device address
uint8_t usbh_get_rhport(uint8_t daddr);

// Get enumeration buffer
uint8_t* usbh_get_enum_buf(void);

// Interrupt control
void usbh_int_set(bool enabled);

// Spinlock control
void usbh_spin_lock(bool in_isr);
void usbh_spin_unlock(bool in_isr);

// Defer a function to be called later
void usbh_defer_func(osal_task_func_t func, void *param, bool in_isr);

// Driver enumeration complete callback
void usbh_driver_set_config_complete(uint8_t dev_addr, uint8_t itf_num);

// Endpoint transfer functions
#if CFG_TUH_API_EDPT_XFER
bool usbh_edpt_xfer_with_callback(uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes,
                                  tuh_xfer_cb_t complete_cb, uintptr_t user_data);
#endif

bool usbh_edpt_xfer(uint8_t dev_addr, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes);
bool usbh_edpt_claim(uint8_t dev_addr, uint8_t ep_addr);
bool usbh_edpt_release(uint8_t dev_addr, uint8_t ep_addr);
bool usbh_edpt_busy(uint8_t dev_addr, uint8_t ep_addr);

#ifdef __cplusplus
 }
#endif

#endif
