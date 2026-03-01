/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
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

#include "tusb_option.h"

#if CFG_TUD_ENABLED

#include "device/dcd.h"
#include "tusb.h"
#include "common/tusb_private.h"

#include "device/usbd.h"
#include "device/usbd_pvt.h"

//--------------------------------------------------------------------+
// USBD Configuration
//--------------------------------------------------------------------+
#ifndef CFG_TUD_TASK_QUEUE_SZ
  #define CFG_TUD_TASK_QUEUE_SZ   16
#endif

//--------------------------------------------------------------------+
// Weak stubs: invoked if no strong implementation is available
//--------------------------------------------------------------------+
TU_ATTR_WEAK void tud_event_hook_cb(uint8_t rhport, uint32_t eventid, bool in_isr) {
  (void) rhport; (void) eventid; (void) in_isr;
}

TU_ATTR_WEAK void tud_sof_cb(uint32_t frame_count) {
  (void) frame_count;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_bos_cb(void) {
  return NULL;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_device_qualifier_cb(void) {
  return NULL;
}

TU_ATTR_WEAK uint8_t const* tud_descriptor_other_speed_configuration_cb(uint8_t index) {
  (void) index;
  return NULL;
}

TU_ATTR_WEAK void tud_mount_cb(void) {
}

TU_ATTR_WEAK void tud_umount_cb(void) {
}

TU_ATTR_WEAK void tud_suspend_cb(bool remote_wakeup_en) {
  (void) remote_wakeup_en;
}

TU_ATTR_WEAK void tud_resume_cb(void) {
}

TU_ATTR_WEAK bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const* request) {
  (void) rhport; (void) stage; (void) request;
  return false;
}

TU_ATTR_WEAK bool dcd_deinit(uint8_t rhport) {
  (void) rhport;
  return false;
}

TU_ATTR_WEAK void dcd_connect(uint8_t rhport) {
  (void) rhport;
}

TU_ATTR_WEAK void dcd_disconnect(uint8_t rhport) {
  (void) rhport;
}

TU_ATTR_WEAK bool dcd_dcache_clean(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

TU_ATTR_WEAK bool dcd_dcache_invalidate(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

TU_ATTR_WEAK bool dcd_dcache_clean_invalidate(const void* addr, uint32_t data_size) {
  (void) addr; (void) data_size;
  return true;
}

//--------------------------------------------------------------------+
// INSTANCE STORAGE
//--------------------------------------------------------------------+

// DRVID_INVALID kept as local alias for backward compat within this file
enum { DRVID_INVALID = USBD_DRVID_INVALID };

// Static storage for all instances
static usbd_instance_t _usbd_instances[CFG_TUD_MAX_RHPORT];

// Track if class drivers have been initialized (global, once only)
static bool _class_drivers_initialized = false;

// Current rhport being processed for descriptor callbacks
static uint8_t _current_processing_rhport = 0;

//--------------------------------------------------------------------+
// Per-Instance OSAL Resources (Queue and Spinlock)
// These must be defined at file scope because OSAL macros create
// buffers and cannot be embedded in structs
//--------------------------------------------------------------------+

// Interrupt control functions for each rhport (referenced by OSAL macros)
TU_ATTR_UNUSED static void usbd_int_set_0(bool enabled) {
  if (enabled) {
    dcd_int_enable(0);
  } else {
    dcd_int_disable(0);
  }
}

// Event queue and spinlock for rhport 0
OSAL_QUEUE_DEF(usbd_int_set_0, _usbd_qdef_0, CFG_TUD_TASK_QUEUE_SZ, dcd_event_t);
static osal_queue_t _usbd_q_0 = NULL;

OSAL_SPINLOCK_DEF(_usbd_spin_0, usbd_int_set_0);
static osal_spinlock_t* _usbd_spinlock_0 = &_usbd_spin_0;

#if CFG_TUD_MAX_RHPORT >= 2
// Interrupt control function for rhport 1 (referenced by OSAL macros)
TU_ATTR_UNUSED static void usbd_int_set_1(bool enabled) {
  if (enabled) {
    dcd_int_enable(1);
  } else {
    dcd_int_disable(1);
  }
}

// Event queue and spinlock for rhport 1
OSAL_QUEUE_DEF(usbd_int_set_1, _usbd_qdef_1, CFG_TUD_TASK_QUEUE_SZ, dcd_event_t);
static osal_queue_t _usbd_q_1 = NULL;

OSAL_SPINLOCK_DEF(_usbd_spin_1, usbd_int_set_1);
static osal_spinlock_t* _usbd_spinlock_1 = &_usbd_spin_1;
#endif

//--------------------------------------------------------------------+
// Instance Helper Functions
//--------------------------------------------------------------------+

// Get instance from rhport
static inline usbd_instance_t* get_instance(uint8_t rhport) {
  if (rhport >= CFG_TUD_MAX_RHPORT) return NULL;
  usbd_instance_t* inst = &_usbd_instances[rhport];
  return inst->initialized ? inst : NULL;
}

// Public API for usbd_control.c
usbd_instance_t* usbd_get_instance(uint8_t rhport) {
  return get_instance(rhport);
}

// Get the current processing rhport for descriptor callbacks
uint8_t tud_get_current_rhport(void) {
  return _current_processing_rhport;
}

// Get default (first initialized) instance for backward-compat API
static inline usbd_instance_t* get_default_instance(void) {
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    if (_usbd_instances[i].initialized) return &_usbd_instances[i];
  }
  return NULL;
}

//--------------------------------------------------------------------+
// Class Driver
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
  #define DRIVER_NAME(_name)  _name
#else
  #define DRIVER_NAME(_name)  NULL
#endif

// Built-in class drivers
tu_static usbd_class_driver_t const _usbd_driver[] = {
    #if CFG_TUD_CDC
    {
        .name             = DRIVER_NAME("CDC"),
        .init             = cdcd_init,
        .deinit           = cdcd_deinit,
        .reset            = cdcd_reset,
        .open             = cdcd_open,
        .control_xfer_cb  = cdcd_control_xfer_cb,
        .xfer_cb          = cdcd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_MSC
    {
        .name             = DRIVER_NAME("MSC"),
        .init             = mscd_init,
        .deinit           = NULL,
        .reset            = mscd_reset,
        .open             = mscd_open,
        .control_xfer_cb  = mscd_control_xfer_cb,
        .xfer_cb          = mscd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_HID
    {
        .name             = DRIVER_NAME("HID"),
        .init             = hidd_init,
        .deinit           = hidd_deinit,
        .reset            = hidd_reset,
        .open             = hidd_open,
        .control_xfer_cb  = hidd_control_xfer_cb,
        .xfer_cb          = hidd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_AUDIO
    {
        .name             = DRIVER_NAME("AUDIO"),
        .init             = audiod_init,
        .deinit           = audiod_deinit,
        .reset            = audiod_reset,
        .open             = audiod_open,
        .control_xfer_cb  = audiod_control_xfer_cb,
        .xfer_cb          = audiod_xfer_cb,
        .xfer_isr         = audiod_xfer_isr,
        .sof              = audiod_sof_isr
    },
    #endif

    #if CFG_TUD_VIDEO
    {
        .name             = DRIVER_NAME("VIDEO"),
        .init             = videod_init,
        .deinit           = videod_deinit,
        .reset            = videod_reset,
        .open             = videod_open,
        .control_xfer_cb  = videod_control_xfer_cb,
        .xfer_cb          = videod_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_MIDI
    {
        .name             = DRIVER_NAME("MIDI"),
        .init             = midid_init,
        .deinit           = midid_deinit,
        .reset            = midid_reset,
        .open             = midid_open,
        .control_xfer_cb  = midid_control_xfer_cb,
        .xfer_cb          = midid_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_VENDOR
    {
        .name             = DRIVER_NAME("VENDOR"),
        .init             = vendord_init,
        .deinit           = vendord_deinit,
        .reset            = vendord_reset,
        .open             = vendord_open,
        .control_xfer_cb  = NULL,
        .xfer_cb          = vendord_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_USBTMC
    {
        .name             = DRIVER_NAME("TMC"),
        .init             = usbtmcd_init_cb,
        .deinit           = usbtmcd_deinit,
        .reset            = usbtmcd_reset_cb,
        .open             = usbtmcd_open_cb,
        .control_xfer_cb  = usbtmcd_control_xfer_cb,
        .xfer_cb          = usbtmcd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_DFU_RUNTIME
    {
        .name             = DRIVER_NAME("DFU-RT"),
        .init             = dfu_rtd_init,
        .deinit           = dfu_rtd_deinit,
        .reset            = dfu_rtd_reset,
        .open             = dfu_rtd_open,
        .control_xfer_cb  = dfu_rtd_control_xfer_cb,
        .xfer_cb          = NULL,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_DFU
    {
        .name             = DRIVER_NAME("DFU"),
        .init             = dfu_moded_init,
        .deinit           = dfu_moded_deinit,
        .reset            = dfu_moded_reset,
        .open             = dfu_moded_open,
        .control_xfer_cb  = dfu_moded_control_xfer_cb,
        .xfer_cb          = NULL,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_ECM_RNDIS || CFG_TUD_NCM
    {
        .name             = DRIVER_NAME("NET"),
        .init             = netd_init,
        .deinit           = netd_deinit,
        .reset            = netd_reset,
        .open             = netd_open,
        .control_xfer_cb  = netd_control_xfer_cb,
        .xfer_cb          = netd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif

    #if CFG_TUD_BTH
    {
        .name             = DRIVER_NAME("BTH"),
        .init             = btd_init,
        .deinit           = btd_deinit,
        .reset            = btd_reset,
        .open             = btd_open,
        .control_xfer_cb  = btd_control_xfer_cb,
        .xfer_cb          = btd_xfer_cb,
        .xfer_isr         = NULL,
        .sof              = NULL
    },
    #endif
};

enum { BUILTIN_DRIVER_COUNT = TU_ARRAY_SIZE(_usbd_driver) };

tu_static usbd_class_driver_t const * _app_driver = NULL;
tu_static uint8_t _app_driver_count = 0;

#define TOTAL_DRIVER_COUNT    (uint8_t)(BUILTIN_DRIVER_COUNT + _app_driver_count)

static inline usbd_class_driver_t const * get_driver(uint8_t drvid) {
  usbd_class_driver_t const * driver = NULL;

  if ( drvid < BUILTIN_DRIVER_COUNT ) {
    driver = &_usbd_driver[drvid];
  } else if ( _app_driver ) {
    driver = &_app_driver[drvid - BUILTIN_DRIVER_COUNT];
  }

  return driver;
}

//--------------------------------------------------------------------+
// DCD Event Queue
//--------------------------------------------------------------------+

// Route event to correct instance's queue
TU_ATTR_ALWAYS_INLINE static inline bool queue_event(dcd_event_t const * event, bool in_isr) {
  uint8_t const rh = event->rhport;
  usbd_instance_t* inst = (rh < CFG_TUD_MAX_RHPORT) ? &_usbd_instances[rh] : NULL;
  TU_ASSERT(inst && inst->event_queue);
  TU_ASSERT(osal_queue_send(inst->event_queue, event, in_isr));
  tud_event_hook_cb(event->rhport, event->event_id, in_isr);
  return true;
}

//--------------------------------------------------------------------+
// Prototypes
//--------------------------------------------------------------------+
static bool process_control_request(uint8_t rhport, tusb_control_request_t const * p_request);
static bool process_set_config(uint8_t rhport, uint8_t cfg_num);
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request);

#if CFG_TUD_TEST_MODE
static bool process_test_mode_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request) {
  TU_VERIFY(CONTROL_STAGE_ACK == stage);
  uint8_t const selector = tu_u16_high(request->wIndex);
  TU_LOG_USBD("    Enter Test Mode (test selector index: %d)\r\n", selector);
  dcd_enter_test_mode(rhport, (tusb_feature_test_mode_t) selector);
  return true;
}
#endif

// from usbd_control.c
void usbd_control_reset(uint8_t rhport);
void usbd_control_set_request(uint8_t rhport, tusb_control_request_t const *request);
void usbd_control_set_complete_callback(uint8_t rhport, usbd_control_xfer_cb_t fp);
bool usbd_control_xfer_cb (uint8_t rhport, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes);

//--------------------------------------------------------------------+
// Weak stubs: invoked if no strong implementation is available
//--------------------------------------------------------------------+
TU_ATTR_WEAK usbd_class_driver_t const* usbd_app_driver_get_cb(uint8_t* driver_count) {
  *driver_count = 0;
  return NULL;
}

TU_ATTR_WEAK bool dcd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t * ff, uint16_t total_bytes, bool is_isr) {
  (void) rhport; (void) ep_addr; (void) ff; (void) total_bytes; (void) is_isr;
  return false;
}

//--------------------------------------------------------------------+
// Debug
//--------------------------------------------------------------------+
#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
tu_static char const* const _usbd_event_str[DCD_EVENT_COUNT] = {
    "Invalid",
    "Bus Reset",
    "Unplugged",
    "SOF",
    "Suspend",
    "Resume",
    "Setup Received",
    "Xfer Complete",
    "Func Call"
};

// for usbd_control to print the name of control complete driver
void usbd_driver_print_control_complete_name(usbd_control_xfer_cb_t callback) {
  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    if (driver && driver->control_xfer_cb == callback) {
      TU_LOG_USBD("%s control complete\r\n", driver->name);
      return;
    }
  }
}

#endif

//--------------------------------------------------------------------+
// Application API (backward compatible — uses default instance)
//--------------------------------------------------------------------+
tusb_speed_t tud_speed_get(void) {
  usbd_instance_t* inst = get_default_instance();
  return inst ? (tusb_speed_t) inst->dev.speed : TUSB_SPEED_FULL;
}

bool tud_connected(void) {
  usbd_instance_t* inst = get_default_instance();
  return inst ? inst->dev.connected : false;
}

bool tud_mounted(void) {
  usbd_instance_t* inst = get_default_instance();
  return inst ? (inst->dev.cfg_num ? true : false) : false;
}

bool tud_suspended(void) {
  usbd_instance_t* inst = get_default_instance();
  return inst ? inst->dev.suspended : false;
}

bool tud_remote_wakeup(void) {
  usbd_instance_t* inst = get_default_instance();
  TU_VERIFY(inst);
  TU_VERIFY(inst->dev.suspended && inst->dev.remote_wakeup_support && inst->dev.remote_wakeup_en);
  dcd_remote_wakeup(inst->rhport);
  return true;
}

bool tud_disconnect(void) {
  usbd_instance_t* inst = get_default_instance();
  TU_VERIFY(inst);
  dcd_disconnect(inst->rhport);
  return true;
}

bool tud_connect(void) {
  usbd_instance_t* inst = get_default_instance();
  TU_VERIFY(inst);
  dcd_connect(inst->rhport);
  return true;
}

void tud_sof_cb_enable(bool en) {
  usbd_instance_t* inst = get_default_instance();
  if (inst) {
    usbd_sof_enable(inst->rhport, SOF_CONSUMER_USER, en);
  }
}

//--------------------------------------------------------------------+
// USBD Task
//--------------------------------------------------------------------+
bool tud_inited(void) {
  return get_default_instance() != NULL;
}

bool tud_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  TU_ASSERT(rhport < CFG_TUD_MAX_RHPORT);
  TU_ASSERT(rh_init);

  usbd_instance_t* inst = &_usbd_instances[rhport];

  // Return if this rhport is already initialized
  if (inst->initialized) {
    TU_LOG_USBD("USBD rhport %u already initialized\r\n", rhport);
    return true;
  }

#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
  char const* speed_str = 0;
            switch (rh_init->speed) {
    case TUSB_SPEED_HIGH:
      speed_str = "High";
    break;
    case TUSB_SPEED_FULL:
      speed_str = "Full";
    break;
    case TUSB_SPEED_LOW:
      speed_str = "Low";
    break;
    case TUSB_SPEED_AUTO:
      speed_str = "Auto";
    break;
  default:
    break;
  }
  TU_LOG_USBD("USBD init on controller %u, speed = %s\r\n", rhport, speed_str);
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(usbd_device_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(dcd_event_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(tu_fifo_t));
  TU_LOG_INT(CFG_TUD_LOG_LEVEL, sizeof(tu_edpt_stream_t));
#endif

  // Clear instance
  tu_memclr(inst, sizeof(usbd_instance_t));
  inst->rhport = rhport;

  // Assign pre-created event queue and spinlock based on rhport
  if (rhport == 0) {
    if (_usbd_q_0 == NULL) {
      _usbd_q_0 = osal_queue_create(&_usbd_qdef_0);
      TU_ASSERT(_usbd_q_0);
    }
    inst->event_queue = _usbd_q_0;
    inst->spin = _usbd_spinlock_0;
  }
#if CFG_TUD_MAX_RHPORT >= 2
  else if (rhport == 1) {
    if (_usbd_q_1 == NULL) {
      _usbd_q_1 = osal_queue_create(&_usbd_qdef_1);
      TU_ASSERT(_usbd_q_1);
    }
    inst->event_queue = _usbd_q_1;
    inst->spin = _usbd_spinlock_1;
  }
#endif
  else {
    TU_LOG_USBD("Invalid rhport %u (max %u)\r\n", rhport, CFG_TUD_MAX_RHPORT);
    return false;
  }

  osal_spin_init(inst->spin);

#if OSAL_MUTEX_REQUIRED
  inst->mutex = osal_mutex_create(&inst->mutex_def);
  TU_ASSERT(inst->mutex);
#endif

  // Initialize class drivers (global init, once only)
  if (!_class_drivers_initialized) {
    _app_driver = usbd_app_driver_get_cb(&_app_driver_count);
    TU_ASSERT(_app_driver_count + BUILTIN_DRIVER_COUNT <= UINT8_MAX);

    for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
      usbd_class_driver_t const* driver = get_driver(i);
      TU_ASSERT(driver && driver->init);
      TU_LOG_USBD("%s init\r\n", driver->name);
      driver->init();
    }

    _class_drivers_initialized = true;
  }

  // Mark initialized before dcd_init (ISR may fire immediately)
  inst->initialized = true;

  // Init device controller driver
  TU_ASSERT(dcd_init(rhport, rh_init));
  dcd_int_enable(rhport);

  return true;
}

bool tud_deinit(uint8_t rhport) {
  usbd_instance_t* inst = get_instance(rhport);
  if (!inst) {
    return true; // not initialized, nothing to do
  }

  TU_LOG_USBD("USBD deinit on controller %u\r\n", rhport);

  // Deinit device controller driver
  dcd_int_disable(rhport);
  dcd_disconnect(rhport);
  TU_VERIFY(dcd_deinit(rhport));

  // Deinit class drivers (only if no other instances remain)
  bool others_active = false;
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    if (i != rhport && _usbd_instances[i].initialized) {
      others_active = true;
      break;
    }
  }

  if (!others_active) {
    for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
      usbd_class_driver_t const* driver = get_driver(i);
      if(driver && driver->deinit) {
        TU_LOG_USBD("%s deinit\r\n", driver->name);
        driver->deinit();
      }
    }
    _class_drivers_initialized = false;
  }

  // Deinit device queue
  osal_queue_delete(inst->event_queue);
  if (rhport == 0) {
    _usbd_q_0 = NULL;
  }
#if CFG_TUD_MAX_RHPORT >= 2
  else if (rhport == 1) {
    _usbd_q_1 = NULL;
  }
#endif

#if OSAL_MUTEX_REQUIRED
  osal_mutex_delete(inst->mutex);
  inst->mutex = NULL;
#endif

  inst->initialized = false;
  return true;
}

static void configuration_reset(uint8_t rhport) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst,);

  for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
    usbd_class_driver_t const* driver = get_driver(i);
    TU_ASSERT(driver,);
    driver->reset(rhport);
  }

  tu_varclr(&inst->dev);
  (void) memset(inst->dev.itf2drv, DRVID_INVALID, sizeof(inst->dev.itf2drv));
  (void) memset(inst->dev.ep2drv, DRVID_INVALID, sizeof(inst->dev.ep2drv));
}

static void usbd_reset(uint8_t rhport) {
  configuration_reset(rhport);
  usbd_control_reset(rhport);
}

bool tud_task_event_ready(void) {
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    usbd_instance_t* inst = &_usbd_instances[i];
    if (inst->initialized && !osal_queue_empty(inst->event_queue)) {
      return true;
    }
  }
  return false;
}

/* USB Device Driver task
 * This top level thread manages all device controller event and delegates events to class-specific drivers.
 * This should be called periodically within the mainloop or rtos thread.
 *
    int main(void) {
      application_init();
      tusb_init(0, TUSB_ROLE_DEVICE);

      while(1) { // the mainloop
        application_code();
        tud_task(); // tinyusb device task
      }
    }
 */
void tud_task_ext(uint32_t timeout_ms, bool in_isr) {
  (void) in_isr; // not implemented yet

  // For multi-instance: use non-blocking queue receives so we round-robin
  // all controllers instead of blocking forever on the first one's queue.
#if CFG_TUD_MAX_RHPORT > 1
  const uint32_t recv_timeout = 0;
#else
  const uint32_t recv_timeout = timeout_ms;
#endif

  // Process all initialized instances
  for (uint8_t rh = 0; rh < CFG_TUD_MAX_RHPORT; rh++) {
    usbd_instance_t* inst = &_usbd_instances[rh];
    if (!inst->initialized) continue;

    // Loop until there is no more events in the queue for this instance
    while (1) {
      dcd_event_t event;
      if (!osal_queue_receive(inst->event_queue, &event, recv_timeout)) {
        break; // no more events for this instance
      }

#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
      if (event.event_id == DCD_EVENT_SETUP_RECEIVED) TU_LOG_USBD("\r\n"); // extra line for setup
      TU_LOG_USBD("USBD[%u] %s ", rh, event.event_id < DCD_EVENT_COUNT ? _usbd_event_str[event.event_id] : "CORRUPTED");
#endif

      switch (event.event_id) {
        case DCD_EVENT_BUS_RESET:
          TU_LOG_USBD(": %s Speed\r\n", tu_str_speed[event.bus_reset.speed]);
          usbd_reset(event.rhport);
          inst->dev.speed = event.bus_reset.speed;
          break;

        case DCD_EVENT_UNPLUGGED:
          TU_LOG_USBD("\r\n");
          usbd_reset(event.rhport);
          tud_umount_cb();
          break;

        case DCD_EVENT_SETUP_RECEIVED:
          TU_ASSERT(inst->queued_setup > 0,);
          inst->queued_setup--;
          TU_LOG_BUF(CFG_TUD_LOG_LEVEL, &event.setup_received, 8);
          if (inst->queued_setup != 0) {
            TU_LOG_USBD("  Skipped since there is other SETUP in queue\r\n");
            break;
          }

          // Mark as connected after receiving 1st setup packet.
          inst->dev.connected = 1;

          // mark both in & out control as free
          inst->dev.ep_status[0][TUSB_DIR_OUT].busy = 0;
          inst->dev.ep_status[0][TUSB_DIR_OUT].claimed = 0;
          inst->dev.ep_status[0][TUSB_DIR_IN].busy = 0;
          inst->dev.ep_status[0][TUSB_DIR_IN].claimed = 0;

          // Process control request
          if (!process_control_request(event.rhport, &event.setup_received)) {
            TU_LOG_USBD("  Stall EP0\r\n");
            // Failed -> stall both control endpoint IN and OUT
            dcd_edpt_stall(event.rhport, 0);
            dcd_edpt_stall(event.rhport, 0 | TUSB_DIR_IN_MASK);
          }
          break;

        case DCD_EVENT_XFER_COMPLETE: {
          // Invoke the class callback associated with the endpoint address
          uint8_t const ep_addr = event.xfer_complete.ep_addr;
          uint8_t const epnum = tu_edpt_number(ep_addr);
          uint8_t const ep_dir = tu_edpt_dir(ep_addr);

          TU_LOG_USBD("on EP %02X with %u bytes\r\n", ep_addr, (unsigned int) event.xfer_complete.len);

          inst->dev.ep_status[epnum][ep_dir].busy = 0;
          inst->dev.ep_status[epnum][ep_dir].claimed = 0;

          if (0 == epnum) {
            usbd_control_xfer_cb(event.rhport, ep_addr, (xfer_result_t) event.xfer_complete.result, event.xfer_complete.len);
          } else {
            usbd_class_driver_t const* driver = get_driver(inst->dev.ep2drv[epnum][ep_dir]);
            TU_ASSERT(driver,);

            TU_LOG_USBD("  %s xfer callback\r\n", driver->name);
            driver->xfer_cb(event.rhport, ep_addr, (xfer_result_t) event.xfer_complete.result, event.xfer_complete.len);
          }
          break;
        }

        case DCD_EVENT_SUSPEND:
          if (inst->dev.connected) {
            TU_LOG_USBD(": Remote Wakeup = %u\r\n", inst->dev.remote_wakeup_en);
            tud_suspend_cb(inst->dev.remote_wakeup_en);
          } else {
            TU_LOG_USBD(" Skipped\r\n");
          }
          break;

        case DCD_EVENT_RESUME:
          if (inst->dev.connected) {
            TU_LOG_USBD("\r\n");
            tud_resume_cb();
          } else {
            TU_LOG_USBD(" Skipped\r\n");
          }
          break;

        case USBD_EVENT_FUNC_CALL:
          TU_LOG_USBD("\r\n");
          if (event.func_call.func != NULL) {
            event.func_call.func(event.func_call.param);
          }
          break;

        case DCD_EVENT_SOF:
          if (tu_bit_test(inst->dev.sof_consumer, SOF_CONSUMER_USER)) {
            TU_LOG_USBD("\r\n");
            tud_sof_cb(event.sof.frame_count);
          }
        break;

        default:
          TU_BREAKPOINT();
          break;
      }

#if CFG_TUSB_OS != OPT_OS_NONE && CFG_TUSB_OS != OPT_OS_PICO
      // return if there is no more events, for application to run other background
      if (osal_queue_empty(inst->event_queue)) { break; }
#endif
    }
  }

#if CFG_TUD_MAX_RHPORT > 1
  // Multi-instance: we used non-blocking receives to round-robin all ports.
  // Yield briefly to prevent busy-loop when no events are pending.
  (void) timeout_ms;
  osal_task_delay(1);
#endif
}

//--------------------------------------------------------------------+
// Control Request Parser & Handling
//--------------------------------------------------------------------+

// Helper to invoke class driver control request handler
static bool invoke_class_control(uint8_t rhport, usbd_class_driver_t const * driver, tusb_control_request_t const * request) {
  usbd_control_set_complete_callback(rhport, driver->control_xfer_cb);
  TU_LOG_USBD("  %s control request\r\n", driver->name);
  return driver->control_xfer_cb(rhport, CONTROL_STAGE_SETUP, request);
}

// This handles the actual request and its response.
// Returns false if unable to complete the request, causing caller to stall control endpoints.
static bool process_control_request(uint8_t rhport, tusb_control_request_t const * p_request) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  usbd_control_set_complete_callback(rhport, NULL);
  TU_ASSERT(p_request->bmRequestType_bit.type < TUSB_REQ_TYPE_INVALID);

  // Vendor request
  if ( p_request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR ) {
    usbd_control_set_complete_callback(rhport, tud_vendor_control_xfer_cb);
    return tud_vendor_control_xfer_cb(rhport, CONTROL_STAGE_SETUP, p_request);
  }

#if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
  if (TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type && p_request->bRequest <= TUSB_REQ_SYNCH_FRAME) {
    TU_LOG_USBD("  %s", tu_str_std_request[p_request->bRequest]);
    if (TUSB_REQ_GET_DESCRIPTOR != p_request->bRequest) TU_LOG_USBD("\r\n");
  }
#endif

  switch (p_request->bmRequestType_bit.recipient) { //-V2520
    //------------- Device Requests e.g in enumeration -------------//
    case TUSB_REQ_RCPT_DEVICE:
      if ( TUSB_REQ_TYPE_CLASS == p_request->bmRequestType_bit.type ) {
        uint8_t const itf = tu_u16_low(p_request->wIndex);
        TU_VERIFY(itf < TU_ARRAY_SIZE(inst->dev.itf2drv));

        usbd_class_driver_t const * driver = get_driver(inst->dev.itf2drv[itf]);
        TU_VERIFY(driver);

        // forward to class driver: "non-STD request to Interface"
        return invoke_class_control(rhport, driver, p_request);
      }

      if (TUSB_REQ_TYPE_STANDARD != p_request->bmRequestType_bit.type) {
        // Non-standard request is not supported
        TU_BREAKPOINT();
        return false;
      }

      switch (p_request->bRequest) { //-V2520
        case TUSB_REQ_SET_ADDRESS:
          usbd_control_set_request(rhport, p_request);
          dcd_set_address(rhport, (uint8_t) p_request->wValue);
          inst->dev.addressed = 1;
        break;

        case TUSB_REQ_GET_CONFIGURATION: {
          uint8_t cfg_num = inst->dev.cfg_num;
          tud_control_xfer(rhport, p_request, &cfg_num, 1);
        }
        break;

        case TUSB_REQ_SET_CONFIGURATION: {
          uint8_t const cfg_num = (uint8_t) p_request->wValue;

          // Only process if new configure is different
          if (inst->dev.cfg_num != cfg_num) {
            if (inst->dev.cfg_num != 0) {
              TU_LOG_USBD("  Clear current Configuration (%u) before switching\r\n", inst->dev.cfg_num);

              dcd_sof_enable(rhport, false);
              dcd_edpt_close_all(rhport);

              const uint8_t speed = inst->dev.speed;
              configuration_reset(rhport);

              inst->dev.speed = speed; // restore speed
            }

            inst->dev.cfg_num = cfg_num;

            // Handle the new configuration
            if (cfg_num == 0) {
              tud_umount_cb();
            } else {
              if (!process_set_config(rhport, cfg_num)) {
                inst->dev.cfg_num = 0;
                TU_ASSERT(false);
              }
              tud_mount_cb();
            }
          }

          tud_control_status(rhport, p_request);
        }
        break;

        case TUSB_REQ_GET_DESCRIPTOR:
          TU_VERIFY(process_get_descriptor(rhport, p_request));
        break;

        case TUSB_REQ_SET_FEATURE:
          switch(p_request->wValue) { //-V2520
            case TUSB_REQ_FEATURE_REMOTE_WAKEUP:
              TU_LOG_USBD("    Enable Remote Wakeup\r\n");
              inst->dev.remote_wakeup_en = true;
              tud_control_status(rhport, p_request);
              break;

            #if CFG_TUD_TEST_MODE
            case TUSB_REQ_FEATURE_TEST_MODE: {
              TU_VERIFY(0 == tu_u16_low(p_request->wIndex));

              uint8_t const selector = tu_u16_high(p_request->wIndex);
              TU_VERIFY(TUSB_FEATURE_TEST_J <= selector && selector <= TUSB_FEATURE_TEST_FORCE_ENABLE);

              usbd_control_set_complete_callback(rhport, process_test_mode_cb);
              tud_control_status(rhport, p_request);
              break;
            }
            #endif

            // Stall unsupported feature selector
            default: return false;
          }
        break;

        case TUSB_REQ_CLEAR_FEATURE:
          TU_VERIFY(TUSB_REQ_FEATURE_REMOTE_WAKEUP == p_request->wValue);
          TU_LOG_USBD("    Disable Remote Wakeup\r\n");

          inst->dev.remote_wakeup_en = false;
          tud_control_status(rhport, p_request);
          break;

        case TUSB_REQ_GET_STATUS: {
          uint16_t status = (uint16_t) ((inst->dev.self_powered ? 1u : 0u) | (inst->dev.remote_wakeup_en ? 2u : 0u));
          tud_control_xfer(rhport, p_request, &status, 2);
          break;
        }

        // Unknown/Unsupported request
        default: TU_BREAKPOINT(); return false;
      }
    break;

    //------------- Class/Interface Specific Request -------------//
    case TUSB_REQ_RCPT_INTERFACE: {
      uint8_t const itf = tu_u16_low(p_request->wIndex);
      TU_VERIFY(itf < TU_ARRAY_SIZE(inst->dev.itf2drv));

      usbd_class_driver_t const * driver = get_driver(inst->dev.itf2drv[itf]);
      TU_VERIFY(driver);

      if (!invoke_class_control(rhport, driver, p_request)) {
        TU_VERIFY(TUSB_REQ_TYPE_STANDARD == p_request->bmRequestType_bit.type);

        usbd_control_set_complete_callback(rhport, NULL);

        switch (p_request->bRequest) { //-V2520
          case TUSB_REQ_GET_INTERFACE: {
            uint8_t alternate = 0;
            tud_control_xfer(rhport, p_request, &alternate, 1);
            break;
          }

          case TUSB_REQ_SET_INTERFACE:
            tud_control_status(rhport, p_request);
            break;

          default: return false;
        }
      }
      break;
    }

    //------------- Endpoint Request -------------//
    case TUSB_REQ_RCPT_ENDPOINT: {
      uint8_t const ep_addr = tu_u16_low(p_request->wIndex);
      uint8_t const ep_num  = tu_edpt_number(ep_addr);
      uint8_t const ep_dir  = tu_edpt_dir(ep_addr);

      TU_ASSERT(ep_num < TU_ARRAY_SIZE(inst->dev.ep2drv) );
      usbd_class_driver_t const * driver = get_driver(inst->dev.ep2drv[ep_num][ep_dir]);

      if (TUSB_REQ_TYPE_STANDARD != p_request->bmRequestType_bit.type) {
        TU_VERIFY(driver);
        return invoke_class_control(rhport, driver, p_request);
      } else {
        switch (p_request->bRequest) { //-V2520
          case TUSB_REQ_GET_STATUS: {
            uint16_t status = usbd_edpt_stalled(rhport, ep_addr) ? 0x0001u : 0x0000u;
            tud_control_xfer(rhport, p_request, &status, 2);
          }
          break;

          case TUSB_REQ_CLEAR_FEATURE:
          case TUSB_REQ_SET_FEATURE: {
            if ( TUSB_REQ_FEATURE_EDPT_HALT == p_request->wValue ) {
              if ( TUSB_REQ_CLEAR_FEATURE ==  p_request->bRequest ) {
                usbd_edpt_clear_stall(rhport, ep_addr);
              }else {
                usbd_edpt_stall(rhport, ep_addr);
              }
            }

            if (driver != NULL) {
              (void) invoke_class_control(rhport, driver, p_request);
              usbd_control_set_complete_callback(rhport, NULL);

              // skip ZLP status if driver already did that
              if (!inst->dev.ep_status[0][TUSB_DIR_IN].busy) {
                tud_control_status(rhport, p_request);
              }
            }
          }
          break;

          // Unknown/Unsupported request
          default:
            TU_BREAKPOINT();
            return false;
        }
      }
      break;
    }

    // Unknown recipient
    default:
      TU_BREAKPOINT();
      return false;
  }

  return true;
}

// Process Set Configure Request
// This function parse configuration descriptor & open drivers accordingly
static bool process_set_config(uint8_t rhport, uint8_t cfg_num)
{
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  // Set current processing rhport for descriptor callbacks
  _current_processing_rhport = rhport;

  // index is cfg_num-1
  tusb_desc_configuration_t const * desc_cfg = (tusb_desc_configuration_t const *) tud_descriptor_configuration_cb(cfg_num-1);
  TU_ASSERT(desc_cfg != NULL && desc_cfg->bDescriptorType == TUSB_DESC_CONFIGURATION);

  // Parse configuration descriptor
  inst->dev.remote_wakeup_support = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP) ? 1u : 0u;
  inst->dev.self_powered          = (desc_cfg->bmAttributes & TUSB_DESC_CONFIG_ATT_SELF_POWERED ) ? 1u : 0u;

  // Parse interface descriptor
  uint8_t const * p_desc   = ((uint8_t const*) desc_cfg) + sizeof(tusb_desc_configuration_t);
  uint8_t const * desc_end = ((uint8_t const*) desc_cfg) + tu_le16toh(desc_cfg->wTotalLength);

  while( p_desc < desc_end )
  {
    uint8_t assoc_itf_count = 1;

    // Class will always starts with Interface Association (if any) and then Interface descriptor
    if ( TUSB_DESC_INTERFACE_ASSOCIATION == tu_desc_type(p_desc) )
    {
      tusb_desc_interface_assoc_t const * desc_iad = (tusb_desc_interface_assoc_t const *) p_desc;
      assoc_itf_count = desc_iad->bInterfaceCount;

      p_desc = tu_desc_next(p_desc); // next to Interface
    }

    TU_ASSERT( TUSB_DESC_INTERFACE == tu_desc_type(p_desc) );
    tusb_desc_interface_t const * desc_itf = (tusb_desc_interface_t const*) p_desc;

    // Find driver for this interface
    uint16_t const remaining_len = (uint16_t) (desc_end-p_desc);
    uint8_t drv_id;
    for (drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++)
    {
      usbd_class_driver_t const *driver = get_driver(drv_id);
      TU_ASSERT(driver);
      uint16_t const drv_len = driver->open(rhport, desc_itf, remaining_len);

      if ( (sizeof(tusb_desc_interface_t) <= drv_len)  && (drv_len <= remaining_len) )
      {
        // Open successfully
        TU_LOG_USBD("  %s opened\r\n", driver->name);

        if (assoc_itf_count == 1) {
          #if CFG_TUD_CDC
          if ( driver->open == cdcd_open ) {
            assoc_itf_count = 2;
          }
          #endif

          #if CFG_TUD_MIDI
          if (driver->open == midid_open) {
            if (TUSB_CLASS_AUDIO               == desc_itf->bInterfaceClass    &&
                AUDIO_SUBCLASS_CONTROL         == desc_itf->bInterfaceSubClass &&
                AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_itf->bInterfaceProtocol) {
              assoc_itf_count = 2;
            }
          }
          #endif

          #if CFG_TUD_BTH && CFG_TUD_BTH_ISO_ALT_COUNT
          if ( driver->open == btd_open ) assoc_itf_count = 2;
          #endif

          #if CFG_TUD_AUDIO
          if (driver->open == audiod_open) {
            if (TUSB_CLASS_AUDIO               == desc_itf->bInterfaceClass    &&
                AUDIO_SUBCLASS_CONTROL         == desc_itf->bInterfaceSubClass &&
                AUDIO_FUNC_PROTOCOL_CODE_UNDEF == desc_itf->bInterfaceProtocol) {
              uint8_t const* p = tu_desc_next(p_desc);
              uint8_t const* const itf_end = p_desc + remaining_len;
              while (p < itf_end) {
                if (TUSB_DESC_CS_INTERFACE == tu_desc_type(p) &&
                    AUDIO10_CS_AC_INTERFACE_HEADER == ((audio10_desc_cs_ac_interface_1_t const *) p)->bDescriptorSubType) {
                  audio10_desc_cs_ac_interface_1_t const * p_header = (audio10_desc_cs_ac_interface_1_t const *) p;
                  assoc_itf_count = p_header->bInCollection + 1;
                  break;
                }
                p = tu_desc_next(p);
              }
            }
          }
          #endif
        }

        // bind (associated) interfaces to found driver
        for(uint8_t i=0; i<assoc_itf_count; i++)
        {
          uint8_t const itf_num = desc_itf->bInterfaceNumber+i;

          TU_ASSERT(DRVID_INVALID == inst->dev.itf2drv[itf_num]);
          inst->dev.itf2drv[itf_num] = drv_id;
        }

        // bind all endpoints to found driver
        tu_edpt_bind_driver(inst->dev.ep2drv, desc_itf, drv_len, drv_id);

        // next Interface
        p_desc += drv_len;

        break; // exit driver find loop
      }
    }

    // Failed if there is no supported drivers
    TU_ASSERT(drv_id < TOTAL_DRIVER_COUNT);
  }

  return true;
}

// return descriptor's buffer and update desc_len
static bool process_get_descriptor(uint8_t rhport, tusb_control_request_t const * p_request)
{
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  // Set current processing rhport so descriptor callbacks can query it
  _current_processing_rhport = rhport;

  tusb_desc_type_t const desc_type = (tusb_desc_type_t) tu_u16_high(p_request->wValue);
  uint8_t const desc_index = tu_u16_low( p_request->wValue );

  switch(desc_type) { //-V2520
    case TUSB_DESC_DEVICE: {
      TU_LOG_USBD(" Device\r\n");

      void* desc_device = (void*) (uintptr_t) tud_descriptor_device_cb();
      TU_ASSERT(desc_device);

      if ((CFG_TUD_ENDPOINT0_SIZE < sizeof(tusb_desc_device_t)) && !inst->dev.addressed &&
          ((tusb_control_request_t const*) p_request)->wLength > sizeof(tusb_desc_device_t)) {
        tusb_control_request_t mod_request = *p_request;
        mod_request.wLength = CFG_TUD_ENDPOINT0_SIZE;

        return tud_control_xfer(rhport, &mod_request, desc_device, CFG_TUD_ENDPOINT0_SIZE);
      }else {
        return tud_control_xfer(rhport, p_request, desc_device, sizeof(tusb_desc_device_t));
      }
    }
    // break; // unreachable

    case TUSB_DESC_BOS: {
      TU_LOG_USBD(" BOS\r\n");

      uintptr_t desc_bos = (uintptr_t) tud_descriptor_bos_cb();
      TU_VERIFY(desc_bos != 0);

      uint16_t const total_len = tu_le16toh( tu_unaligned_read16((const void*) (desc_bos + offsetof(tusb_desc_bos_t, wTotalLength))) );

      return tud_control_xfer(rhport, p_request, (void*) desc_bos, total_len);
    }
    // break; // unreachable

    case TUSB_DESC_CONFIGURATION:
    case TUSB_DESC_OTHER_SPEED_CONFIG: {
      uintptr_t desc_config;

      if ( desc_type == TUSB_DESC_CONFIGURATION ) {
        TU_LOG_USBD(" Configuration[%u]\r\n", desc_index);
        desc_config = (uintptr_t) tud_descriptor_configuration_cb(desc_index);
        TU_ASSERT(desc_config != 0);
      }else {
        TU_LOG_USBD(" Other Speed Configuration\r\n");
        desc_config = (uintptr_t) tud_descriptor_other_speed_configuration_cb(desc_index);
        TU_VERIFY(desc_config != 0);
      }

      uint16_t const total_len = tu_le16toh( tu_unaligned_read16((const void*) (desc_config + offsetof(tusb_desc_configuration_t, wTotalLength))) );

      return tud_control_xfer(rhport, p_request, (void*) desc_config, total_len);
    }
    // break; // unreachable

    case TUSB_DESC_STRING: {
      TU_LOG_USBD(" String[%u]\r\n", desc_index);

      uint8_t const* desc_str = (uint8_t const*) tud_descriptor_string_cb(desc_index, tu_le16toh(p_request->wIndex));
      TU_VERIFY(desc_str);

      return tud_control_xfer(rhport, p_request, (void*) (uintptr_t) desc_str, tu_desc_len(desc_str));
    }
    // break; // unreachable

    case TUSB_DESC_DEVICE_QUALIFIER: {
      TU_LOG_USBD(" Device Qualifier\r\n");
      uint8_t const* desc_qualifier = tud_descriptor_device_qualifier_cb();
      TU_VERIFY(desc_qualifier);
      return tud_control_xfer(rhport, p_request, (void*) (uintptr_t) desc_qualifier, tu_desc_len(desc_qualifier));
    }
    // break; // unreachable

    default: return false;
  }
}

//--------------------------------------------------------------------+
// DCD Event Handler
//--------------------------------------------------------------------+
TU_ATTR_FAST_FUNC void dcd_event_handler(dcd_event_t const* event, bool in_isr) {
  uint8_t const rh = event->rhport;
  usbd_instance_t* inst = (rh < CFG_TUD_MAX_RHPORT) ? &_usbd_instances[rh] : NULL;

  bool send = false;
  switch (event->event_id) {
    case DCD_EVENT_UNPLUGGED:
      if (inst) {
        inst->dev.connected = 0;
        inst->dev.addressed = 0;
        inst->dev.cfg_num = 0;
        inst->dev.suspended = 0;
      }
      send = true;
      break;

    case DCD_EVENT_SUSPEND:
      if (inst && inst->dev.connected) {
        inst->dev.suspended = 1;
        send = true;
      }
      break;

    case DCD_EVENT_RESUME:
      if (inst && inst->dev.connected) {
        inst->dev.suspended = 0;
        send = true;
      }
      break;

    case DCD_EVENT_SOF:
      // SOF driver handler in ISR context
      for (uint8_t i = 0; i < TOTAL_DRIVER_COUNT; i++) {
        usbd_class_driver_t const* driver = get_driver(i);
        if (driver && driver->sof) {
          driver->sof(event->rhport, event->sof.frame_count);
        }
      }

      if (inst && inst->dev.suspended) {
        inst->dev.suspended = 0;

        dcd_event_t const event_resume = {.rhport = event->rhport, .event_id = DCD_EVENT_RESUME};
        queue_event(&event_resume, in_isr);
      }

      if (inst && tu_bit_test(inst->dev.sof_consumer, SOF_CONSUMER_USER)) {
        dcd_event_t const event_sof = {.rhport = event->rhport, .event_id = DCD_EVENT_SOF, .sof.frame_count = event->sof.frame_count};
        queue_event(&event_sof, in_isr);
      }
      break;

    case DCD_EVENT_SETUP_RECEIVED:
      if (inst) {
        inst->queued_setup++;
      }
      send = true;
      break;

    case DCD_EVENT_XFER_COMPLETE: {
      uint8_t const ep_addr = event->xfer_complete.ep_addr;
      uint8_t const epnum = tu_edpt_number(ep_addr);
      uint8_t const ep_dir = tu_edpt_dir(ep_addr);

      send = true;
      if(epnum > 0 && inst) {
        usbd_class_driver_t const* driver = get_driver(inst->dev.ep2drv[epnum][ep_dir]);

        if (driver && driver->xfer_isr) {
          inst->dev.ep_status[epnum][ep_dir].busy = 0;
          inst->dev.ep_status[epnum][ep_dir].claimed = 0;

          send = !driver->xfer_isr(event->rhport, ep_addr, (xfer_result_t) event->xfer_complete.result, event->xfer_complete.len);

          // xfer_isr() is deferred to xfer_cb(), revert busy/claimed status
          if (send) {
            inst->dev.ep_status[epnum][ep_dir].busy = 1;
            inst->dev.ep_status[epnum][ep_dir].claimed = 1;
          }
        }
      }
      break;
    }

    default:
      send = true;
      break;
  }

  if (send) {
    queue_event(event, in_isr);
  }
}

//--------------------------------------------------------------------+
// USBD API For Class Driver
//--------------------------------------------------------------------+

void usbd_int_set(bool enabled) {
  // Enable/disable for all initialized instances
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    if (_usbd_instances[i].initialized) {
      if (enabled) {
        dcd_int_enable(_usbd_instances[i].rhport);
      } else {
        dcd_int_disable(_usbd_instances[i].rhport);
      }
    }
  }
}

void usbd_spin_lock(bool in_isr) {
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    if (_usbd_instances[i].initialized && _usbd_instances[i].spin) {
      osal_spin_lock(_usbd_instances[i].spin, in_isr);
    }
  }
}

void usbd_spin_unlock(bool in_isr) {
  for (uint8_t i = 0; i < CFG_TUD_MAX_RHPORT; i++) {
    if (_usbd_instances[i].initialized && _usbd_instances[i].spin) {
      osal_spin_unlock(_usbd_instances[i].spin, in_isr);
    }
  }
}

// Parse consecutive endpoint descriptors (IN & OUT)
bool usbd_open_edpt_pair(uint8_t rhport, uint8_t const* p_desc, uint8_t ep_count, uint8_t xfer_type, uint8_t* ep_out, uint8_t* ep_in)
{
  for(int i=0; i<ep_count; i++)
  {
    tusb_desc_endpoint_t const * desc_ep = (tusb_desc_endpoint_t const *) p_desc;

    TU_ASSERT(TUSB_DESC_ENDPOINT == desc_ep->bDescriptorType && xfer_type == desc_ep->bmAttributes.xfer);
    TU_ASSERT(usbd_edpt_open(rhport, desc_ep));

    if ( tu_edpt_dir(desc_ep->bEndpointAddress) == TUSB_DIR_IN )
    {
      (*ep_in) = desc_ep->bEndpointAddress;
    }else
    {
      (*ep_out) = desc_ep->bEndpointAddress;
    }

    p_desc = tu_desc_next(p_desc);
  }

  return true;
}

// Helper to defer an isr function
void usbd_defer_func(osal_task_func_t func, void* param, bool in_isr) {
  dcd_event_t event = {
      .rhport   = 0,
      .event_id = USBD_EVENT_FUNC_CALL,
  };
  event.func_call.func  = func;
  event.func_call.param = param;

  queue_event(&event, in_isr);
}

//--------------------------------------------------------------------+
// USBD Endpoint API
// rhport parameter from class drivers is used directly (no override)
//--------------------------------------------------------------------+

bool usbd_edpt_open(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  TU_ASSERT(tu_edpt_number(desc_ep->bEndpointAddress) < CFG_TUD_ENDPPOINT_MAX);
  TU_ASSERT(tu_edpt_validate(desc_ep, (tusb_speed_t) inst->dev.speed, false));

  return dcd_edpt_open(rhport, desc_ep);
}

bool usbd_edpt_claim(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  tu_edpt_state_t* ep_state = &inst->dev.ep_status[epnum][dir];

#if OSAL_MUTEX_REQUIRED
  return tu_edpt_claim(ep_state, inst->mutex);
#else
  return tu_edpt_claim(ep_state, NULL);
#endif
}

bool usbd_edpt_release(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);
  tu_edpt_state_t* ep_state = &inst->dev.ep_status[epnum][dir];

#if OSAL_MUTEX_REQUIRED
  return tu_edpt_release(ep_state, inst->mutex);
#else
  return tu_edpt_release(ep_state, NULL);
#endif
}

bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t* buffer, uint16_t total_bytes, bool is_isr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  TU_LOG_USBD("  Queue EP %02X with %u bytes ...\r\n", ep_addr, total_bytes);
#if CFG_TUD_LOG_LEVEL >= 3
  if(dir == TUSB_DIR_IN) {
    TU_LOG_MEM(CFG_TUD_LOG_LEVEL, buffer, total_bytes, 2);
  }
#endif

  // Attempt to transfer on a busy endpoint, sound like an race condition !
  TU_ASSERT(inst->dev.ep_status[epnum][dir].busy == 0);

  // Set busy first since the actual transfer can be complete before dcd_edpt_xfer()
  // could return and USBD task can preempt and clear the busy
  inst->dev.ep_status[epnum][dir].busy = 1;

  if (dcd_edpt_xfer(rhport, ep_addr, buffer, total_bytes, is_isr)) {
    return true;
  } else {
    // DCD error, mark endpoint as ready to allow next transfer
    inst->dev.ep_status[epnum][dir].busy = 0;
    inst->dev.ep_status[epnum][dir].claimed = 0;
    TU_LOG_USBD("FAILED\r\n");
    TU_BREAKPOINT();
    return false;
  }
}

bool usbd_edpt_xfer_fifo(uint8_t rhport, uint8_t ep_addr, tu_fifo_t* ff, uint16_t total_bytes, bool is_isr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  TU_LOG_USBD("  Queue ISO EP %02X with %u bytes ... ", ep_addr, total_bytes);

  // Attempt to transfer on a busy endpoint, sound like an race condition !
  TU_ASSERT(inst->dev.ep_status[epnum][dir].busy == 0);

  // Set busy first since the actual transfer can be complete before dcd_edpt_xfer() could return
  // and usbd task can preempt and clear the busy
  inst->dev.ep_status[epnum][dir].busy = 1;

  if (dcd_edpt_xfer_fifo(rhport, ep_addr, ff, total_bytes, is_isr)) {
    TU_LOG_USBD("OK\r\n");
    return true;
  } else {
    // DCD error, mark endpoint as ready to allow next transfer
    inst->dev.ep_status[epnum][dir].busy = 0;
    inst->dev.ep_status[epnum][dir].claimed = 0;
    TU_LOG_USBD("failed\r\n");
    TU_BREAKPOINT();
    return false;
  }
}

bool usbd_edpt_busy(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst, false);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  return inst->dev.ep_status[epnum][dir].busy;
}

void usbd_edpt_stall(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst,);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  TU_LOG_USBD("    Stall EP %02X\r\n", ep_addr);
  dcd_edpt_stall(rhport, ep_addr);
  inst->dev.ep_status[epnum][dir].stalled = 1;
  inst->dev.ep_status[epnum][dir].busy = 1;
}

void usbd_edpt_clear_stall(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst,);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  TU_LOG_USBD("    Clear Stall EP %02X\r\n", ep_addr);
  dcd_edpt_clear_stall(rhport, ep_addr);
  inst->dev.ep_status[epnum][dir].stalled = 0;
  inst->dev.ep_status[epnum][dir].busy = 0;
}

bool usbd_edpt_stalled(uint8_t rhport, uint8_t ep_addr) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst, false);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  return inst->dev.ep_status[epnum][dir].stalled;
}

/**
 * usbd_edpt_close will disable an endpoint.
 * In progress transfers on this EP may be delivered after this call.
 */
void usbd_edpt_close(uint8_t rhport, uint8_t ep_addr) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  (void) rhport; (void) ep_addr;
  // ISO alloc/activate Should be used instead
#else
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst,);

  TU_LOG_USBD("  CLOSING Endpoint: 0x%02X\r\n", ep_addr);

  uint8_t const epnum = tu_edpt_number(ep_addr);
  uint8_t const dir = tu_edpt_dir(ep_addr);

  dcd_edpt_close(rhport, ep_addr);
  inst->dev.ep_status[epnum][dir].stalled = 0;
  inst->dev.ep_status[epnum][dir].busy = 0;
  inst->dev.ep_status[epnum][dir].claimed = 0;
#endif

  return;
}

void usbd_sof_enable(uint8_t rhport, sof_consumer_t consumer, bool en) {
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst,);

  uint8_t consumer_old = inst->dev.sof_consumer;
  if (en) {
    inst->dev.sof_consumer |= (uint8_t)(1 << consumer);
  } else {
    inst->dev.sof_consumer &= (uint8_t)(~(1 << consumer));
  }

  // Test logically unequal
  if(!inst->dev.sof_consumer != !consumer_old) {
    dcd_sof_enable(rhport, inst->dev.sof_consumer);
  }
}

bool usbd_edpt_iso_alloc(uint8_t rhport, uint8_t ep_addr, uint16_t largest_packet_size) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  TU_ASSERT(tu_edpt_number(ep_addr) < CFG_TUD_ENDPPOINT_MAX);
  return dcd_edpt_iso_alloc(rhport, ep_addr, largest_packet_size);
#else
  (void) rhport; (void) ep_addr; (void) largest_packet_size;
  return false;
#endif
}

bool usbd_edpt_iso_activate(uint8_t rhport, tusb_desc_endpoint_t const* desc_ep) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
  usbd_instance_t* inst = get_instance(rhport);
  TU_ASSERT(inst);

  uint8_t const epnum = tu_edpt_number(desc_ep->bEndpointAddress);
  uint8_t const dir = tu_edpt_dir(desc_ep->bEndpointAddress);

  TU_ASSERT(epnum < CFG_TUD_ENDPPOINT_MAX);
  TU_ASSERT(tu_edpt_validate(desc_ep, (tusb_speed_t) inst->dev.speed, false));

  inst->dev.ep_status[epnum][dir].stalled = 0;
  inst->dev.ep_status[epnum][dir].busy = 0;
  inst->dev.ep_status[epnum][dir].claimed = 0;
  return dcd_edpt_iso_activate(rhport, desc_ep);
#else
  (void) rhport; (void) desc_ep;
  return false;
#endif
}

#endif
