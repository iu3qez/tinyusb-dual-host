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

#ifdef __cplusplus
 extern "C" {
#endif

//--------------------------------------------------------------------+
// Instance Structure Definition
//--------------------------------------------------------------------+

#define TOTAL_DEVICES (CFG_TUH_DEVICE_MAX + CFG_TUH_HUB)

// Device structure
typedef struct {
  uint8_t rhport;
  uint8_t hub_addr;
  uint8_t hub_port;
  uint8_t speed;

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

  volatile uint8_t state;
  uint8_t itf_count;
  uint8_t ep_count;

  uint8_t itf2drv[CFG_TUH_INTERFACE_MAX];
  uint8_t ep2drv[CFG_TUH_ENDPOINT_MAX][2];

  struct TU_ATTR_PACKED {
    volatile bool connected : 1;
    volatile bool configured : 1;
    volatile bool suspended : 1;
  };
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

  OSAL_QUEUE_DEF(event_queue_def, CFG_TUH_TASK_QUEUE_SZ, hcd_event_t);
  osal_queue_t event_queue;

#if OSAL_MUTEX_REQUIRED
  osal_mutex_def_t mutex_def;
  osal_mutex_t mutex;
#endif

  OSAL_SPINLOCK_DEF(spin_def, usbh_int_set);
  osal_spinlock_t spin;

  usbh_device_t devices[TOTAL_DEVICES];
  CFG_TUH_MEM_SECTION CFG_TUH_MEM_ALIGN usbh_epbuf_t epbuf;

  bool initialized;
  bool running;
};

typedef struct usbh_instance usbh_instance_t;

#ifdef __cplusplus
 }
#endif

#endif
