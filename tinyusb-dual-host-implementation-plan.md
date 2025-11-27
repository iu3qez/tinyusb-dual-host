# TinyUSB Dual Host Implementation Plan - Option C (Instance Handle API)

## Overview

Implement a proper instance-based API for TinyUSB host stack to support multiple USB controllers simultaneously. This approach provides the cleanest abstraction and is most future-proof.

---

## Phase 1: Core Data Structures

### 1.1 Define Instance Handle Type

**File**: `src/host/usbh.h`

```c
// Forward declare the internal structure
typedef struct usbh_instance usbh_instance_t;

// Opaque handle for users
typedef usbh_instance_t* tuh_instance_t;
```

### 1.2 Define Per-Instance Structure

**File**: `src/host/usbh_pvt.h` (private header)

```c
typedef struct usbh_instance {
  // Controller identification
  uint8_t rhport;
  uint8_t controller_id;
  uint8_t enumerating_daddr;
  uint8_t attach_debouncing_bm;

  // Bus info for device 0 during enumeration
  tuh_bus_info_t dev0_bus;

  // Control transfer state
  usbh_ctrl_xfer_info_t ctrl_xfer;

  // Event queue for this controller
  osal_queue_def_t queue_def;
  osal_queue_t event_queue;

  // Mutex for endpoint claiming
#if OSAL_MUTEX_REQUIRED
  osal_mutex_def_t mutex_def;
  osal_mutex_t mutex;
#endif

  // Spinlock for interrupt handling
  osal_spinlock_def_t spin_def;
  osal_spinlock_t spin;

  // Devices managed by this controller
  usbh_device_t devices[TOTAL_DEVICES];

  // Enumeration buffer
  usbh_epbuf_t epbuf;

  // Class driver instances (per-controller)
  void* class_data[TOTAL_DRIVER_COUNT];

  // Initialization state
  bool initialized;
  bool running;
} usbh_instance_t;
```

### 1.3 Instance Storage

**File**: `src/host/usbh.c`

```c
// Static storage for instances
#ifndef CFG_TUH_MAX_RHPORT
  #define CFG_TUH_MAX_RHPORT 2
#endif

static usbh_instance_t _usbh_instances[CFG_TUH_MAX_RHPORT];

// Global default instance (for backward compatibility)
static tuh_instance_t _default_instance = NULL;
```

---

## Phase 2: Instance Management API

### 2.1 Instance Initialization

**File**: `src/host/usbh.c`

```c
tuh_instance_t tuh_instance_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  TU_VERIFY(rhport < CFG_TUH_MAX_RHPORT, NULL);

  usbh_instance_t* inst = &_usbh_instances[rhport];

  // Return existing if already initialized
  if (inst->initialized && inst->rhport == rhport) {
    TU_LOG_USBH("Instance for rhport %u already initialized\r\n", rhport);
    return inst;
  }

  // Clear the instance
  tu_memclr(inst, sizeof(usbh_instance_t));
  inst->rhport = rhport;
  inst->controller_id = rhport;
  inst->enumerating_daddr = TUSB_INDEX_INVALID_8;

  // Initialize spinlock
  osal_spin_init(&inst->spin);

  // Create event queue
  inst->event_queue = osal_queue_create(&inst->queue_def, CFG_TUH_TASK_QUEUE_SZ, sizeof(hcd_event_t));
  TU_VERIFY(inst->event_queue != NULL, NULL);

#if OSAL_MUTEX_REQUIRED
  // Create mutex
  inst->mutex = osal_mutex_create(&inst->mutex_def);
  TU_VERIFY(inst->mutex != NULL, NULL);
#endif

  // Initialize devices
  for (uint8_t i = 0; i < TOTAL_DEVICES; i++) {
    clear_device(&inst->devices[i]);
  }

  // Initialize class drivers for this instance
  for (uint8_t drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++) {
    usbh_class_driver_t const* driver = get_driver(drv_id);
    if (driver && driver->init_instance) {
      TU_LOG_USBH("%s init for rhport %u\r\n", driver->name, rhport);
      inst->class_data[drv_id] = driver->init_instance(inst);
    }
  }

  // Initialize HCD
  TU_VERIFY(hcd_init(rhport, rh_init), NULL);
  hcd_int_enable(rhport);

  inst->initialized = true;
  inst->running = true;

  // Set as default if first instance
  if (_default_instance == NULL) {
    _default_instance = inst;
  }

  return inst;
}

// Backward compatibility wrapper
bool tuh_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  tuh_instance_t inst = tuh_instance_init(rhport, rh_init);
  if (inst != NULL && _default_instance == NULL) {
    _default_instance = inst;
  }
  return inst != NULL;
}

// Get default instance
tuh_instance_t tuh_get_default_instance(void) {
  return _default_instance;
}

// Get instance by rhport
tuh_instance_t tuh_get_instance(uint8_t rhport) {
  TU_VERIFY(rhport < CFG_TUH_MAX_RHPORT, NULL);
  usbh_instance_t* inst = &_usbh_instances[rhport];
  return inst->initialized ? inst : NULL;
}
```

### 2.2 Instance Deinitialization

```c
bool tuh_instance_deinit(tuh_instance_t handle) {
  TU_VERIFY(handle != NULL, false);
  usbh_instance_t* inst = (usbh_instance_t*)handle;

  if (!inst->initialized) {
    return true;
  }

  uint8_t rhport = inst->rhport;

  // Disable interrupts
  hcd_int_disable(rhport);

  // Deinitialize HCD
  hcd_deinit(rhport);

  // Remove all devices
  process_removed_device(inst, rhport, 0, 0);

  // Deinitialize class drivers
  for (uint8_t drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++) {
    usbh_class_driver_t const* driver = get_driver(drv_id);
    if (driver && driver->deinit_instance && inst->class_data[drv_id]) {
      TU_LOG_USBH("%s deinit for rhport %u\r\n", driver->name, rhport);
      driver->deinit_instance(inst, inst->class_data[drv_id]);
      inst->class_data[drv_id] = NULL;
    }
  }

  // Clean up OSAL resources
  osal_queue_delete(inst->event_queue);
  inst->event_queue = NULL;

#if OSAL_MUTEX_REQUIRED
  osal_mutex_delete(inst->mutex);
  inst->mutex = NULL;
#endif

  inst->initialized = false;
  inst->running = false;

  // Clear default if it was this instance
  if (_default_instance == inst) {
    _default_instance = NULL;
    // Find next available instance to be default
    for (uint8_t i = 0; i < CFG_TUH_MAX_RHPORT; i++) {
      if (_usbh_instances[i].initialized) {
        _default_instance = &_usbh_instances[i];
        break;
      }
    }
  }

  return true;
}

// Backward compatibility wrapper
bool tuh_deinit(uint8_t rhport) {
  tuh_instance_t inst = tuh_get_instance(rhport);
  return tuh_instance_deinit(inst);
}
```

---

## Phase 3: Update Core USBH Functions

### 3.1 Task Function

```c
void tuh_task_instance(tuh_instance_t handle) {
  TU_VERIFY(handle != NULL, );
  usbh_instance_t* inst = (usbh_instance_t*)handle;

  if (!inst->running) {
    return;
  }

  // Process events from this instance's queue
  hcd_event_t event;
  while (osal_queue_receive(inst->event_queue, &event)) {
    TU_ASSERT(event.rhport == inst->rhport, );

    switch (event.event_id) {
      case HCD_EVENT_DEVICE_ATTACH:
        process_device_attach(inst, event.rhport, event.connection.hub_addr,
                             event.connection.hub_port, event.connection.speed);
        break;

      case HCD_EVENT_DEVICE_REMOVE:
        process_device_remove(inst, event.rhport, event.connection.hub_addr,
                             event.connection.hub_port);
        break;

      case HCD_EVENT_XFER_COMPLETE:
        process_xfer_complete(inst, event.rhport, event.dev_addr,
                            event.xfer_complete.ep_addr,
                            event.xfer_complete.result,
                            event.xfer_complete.len);
        break;

      case USBH_EVENT_FUNC_CALL:
        if (event.func_call.func) {
          event.func_call.func(event.func_call.param);
        }
        break;

      default:
        break;
    }
  }
}

// Backward compatibility: process default instance
void tuh_task(void) {
  tuh_task_instance(_default_instance);
}

// Process ALL instances (useful for applications managing multiple controllers)
void tuh_task_all(void) {
  for (uint8_t i = 0; i < CFG_TUH_MAX_RHPORT; i++) {
    if (_usbh_instances[i].initialized) {
      tuh_task_instance(&_usbh_instances[i]);
    }
  }
}
```

### 3.2 Device Information Functions

```c
bool tuh_instance_mounted(tuh_instance_t handle, uint8_t dev_addr) {
  TU_VERIFY(handle != NULL, false);
  usbh_instance_t* inst = (usbh_instance_t*)handle;
  TU_VERIFY(dev_addr < TOTAL_DEVICES, false);

  return inst->devices[dev_addr].connected && inst->devices[dev_addr].configured;
}

bool tuh_instance_device_get_info(tuh_instance_t handle, uint8_t dev_addr, tuh_device_info_t* info) {
  TU_VERIFY(handle != NULL && info != NULL, false);
  usbh_instance_t* inst = (usbh_instance_t*)handle;
  TU_VERIFY(dev_addr < TOTAL_DEVICES, false);

  usbh_device_t const* dev = &inst->devices[dev_addr];
  TU_VERIFY(dev->connected, false);

  info->rhport = inst->rhport;
  info->hub_addr = dev->hub_addr;
  info->hub_port = dev->hub_port;
  info->speed = dev->speed;

  return true;
}

// Backward compatibility: use default instance
bool tuh_mounted(uint8_t dev_addr) {
  return tuh_instance_mounted(_default_instance, dev_addr);
}
```

---

## Phase 4: Update HCD Layer

### 4.1 Per-Instance HCD Data

**File**: `src/portable/synopsys/dwc2/hcd_dwc2.c`

```c
typedef struct {
  hcd_xfer_t xfer[DWC2_CHANNEL_COUNT_MAX];
  hcd_endpoint_t edpt[CFG_TUH_DWC2_ENDPOINT_MAX];
} hcd_data_t;

// Per-rhport data
static hcd_data_t _hcd_data[CFG_TUH_MAX_RHPORT];

// Helper to get HCD data for rhport
TU_ATTR_ALWAYS_INLINE static inline hcd_data_t* get_hcd_data(uint8_t rhport) {
  TU_ASSERT(rhport < CFG_TUH_MAX_RHPORT, NULL);
  return &_hcd_data[rhport];
}
```

### 4.2 Update HCD Functions

Update all HCD functions to use `get_hcd_data(rhport)`:

```c
// Example: channel_alloc
static uint8_t channel_alloc(dwc2_regs_t* dwc2, uint8_t rhport) {
  hcd_data_t* hcd_data = get_hcd_data(rhport);
  const uint8_t ch_count = dwc2_channel_count(dwc2);

  for (uint8_t ch_id = 0; ch_id < ch_count; ch_id++) {
    if (hcd_data->xfer[ch_id].ep_id == 0) {
      return ch_id;
    }
  }

  return TUSB_INDEX_INVALID_8;
}

// Example: edpt_open
bool hcd_edpt_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_endpoint_t const* ep_desc) {
  hcd_data_t* hcd_data = get_hcd_data(rhport);
  dwc2_regs_t* dwc2 = DWC2_REG(rhport);

  uint8_t const ep_id = edpt_allocate(hcd_data, dev_addr, ep_desc);
  TU_VERIFY(ep_id != TUSB_INDEX_INVALID_8);

  hcd_endpoint_t* edpt = &hcd_data->edpt[ep_id];
  // ... rest of implementation using hcd_data

  return true;
}
```

---

## Phase 5: Update Class Drivers

### 5.1 Class Driver Interface Changes

**File**: `src/host/usbh.h`

```c
typedef struct {
  char const* name;

  // Instance-based lifecycle (new)
  void* (*init_instance)(tuh_instance_t inst);
  void (*deinit_instance)(tuh_instance_t inst, void* class_data);

  // Legacy lifecycle (for backward compatibility)
  void (*init)(void);
  void (*deinit)(void);

  // Device management (add inst parameter)
  bool (*open)(tuh_instance_t inst, uint8_t rhport, uint8_t dev_addr,
               tusb_desc_interface_t const* desc_itf, uint16_t max_len);
  bool (*set_config)(tuh_instance_t inst, uint8_t dev_addr, uint8_t itf_num);
  bool (*xfer_cb)(tuh_instance_t inst, uint8_t dev_addr, uint8_t ep_addr,
                  xfer_result_t result, uint32_t xferred_bytes);
  void (*close)(tuh_instance_t inst, uint8_t dev_addr);
} usbh_class_driver_t;
```

### 5.2 Example: CDC Host Class Driver

**File**: `src/class/cdc/cdc_host.c`

```c
typedef struct {
  uint8_t rhport;
  uint8_t daddr;
  uint8_t bInterfaceNumber;
  // ... rest of CDC interface state
} cdch_interface_t;

// Per-instance storage
typedef struct {
  cdch_interface_t interfaces[CFG_TUH_CDC];
  uint8_t interface_count;
} cdch_instance_data_t;

// Instance initialization
static void* cdch_init_instance(tuh_instance_t inst) {
  cdch_instance_data_t* data = tu_malloc(sizeof(cdch_instance_data_t));
  if (data) {
    tu_memclr(data, sizeof(cdch_instance_data_t));
  }
  return data;
}

// Instance deinitialization
static void cdch_deinit_instance(tuh_instance_t inst, void* class_data) {
  if (class_data) {
    tu_free(class_data);
  }
}

// Open interface
static bool cdch_open(tuh_instance_t inst, uint8_t rhport, uint8_t dev_addr,
                     tusb_desc_interface_t const* desc_itf, uint16_t max_len) {
  // Get instance-specific data
  cdch_instance_data_t* data = (cdch_instance_data_t*)tuh_instance_get_class_data(inst, USBH_CLASS_DRIVER_CDC);
  TU_VERIFY(data != NULL);

  // Find free interface slot
  cdch_interface_t* p_cdc = NULL;
  for (uint8_t i = 0; i < CFG_TUH_CDC; i++) {
    if (data->interfaces[i].daddr == 0) {
      p_cdc = &data->interfaces[i];
      break;
    }
  }
  TU_VERIFY(p_cdc != NULL);

  // Initialize interface
  p_cdc->rhport = rhport;
  p_cdc->daddr = dev_addr;
  p_cdc->bInterfaceNumber = desc_itf->bInterfaceNumber;
  // ... rest of open logic

  return true;
}

// Update driver table
usbh_class_driver_t const cdch_driver = {
  .name = "CDC",
  .init_instance = cdch_init_instance,
  .deinit_instance = cdch_deinit_instance,
  .init = NULL,  // Legacy - NULL if using instance API
  .deinit = NULL,
  .open = cdch_open,
  .set_config = cdch_set_config,
  .xfer_cb = cdch_xfer_cb,
  .close = cdch_close
};
```

---

## Phase 6: Backward Compatibility Layer

### 6.1 Compatibility Macros

**File**: `src/host/usbh.h`

```c
// Enable backward compatibility by default
#ifndef CFG_TUH_BACKWARD_COMPATIBLE_API
  #define CFG_TUH_BACKWARD_COMPATIBLE_API 1
#endif

#if CFG_TUH_BACKWARD_COMPATIBLE_API

// Map old API to new instance-based API using default instance
#define tuh_inited()           (tuh_get_default_instance() != NULL)
#define tuh_task()             tuh_task_instance(tuh_get_default_instance())
#define tuh_mounted(addr)      tuh_instance_mounted(tuh_get_default_instance(), addr)
#define tuh_device_get_info(addr, info) \
  tuh_instance_device_get_info(tuh_get_default_instance(), addr, info)

// Transfer functions
#define tuh_control_xfer(addr, request, buffer, cb, user_data) \
  tuh_instance_control_xfer(tuh_get_default_instance(), addr, request, buffer, cb, user_data)

#define tuh_edpt_xfer(addr, ep_addr, buffer, len) \
  tuh_instance_edpt_xfer(tuh_get_default_instance(), addr, ep_addr, buffer, len)

#endif // CFG_TUH_BACKWARD_COMPATIBLE_API
```

---

## Phase 7: Configuration

### 7.1 Add Configuration Options

**File**: `src/tusb_option.h`

```c
//--------------------------------------------------------------------
// HOST CONFIGURATION
//--------------------------------------------------------------------

// Maximum number of USB root hub ports (controllers) that can operate simultaneously
// - ESP32-P4 has 2 (Full-Speed and High-Speed)
// - Most MCUs have 1
#ifndef CFG_TUH_MAX_RHPORT
  #if TU_CHECK_MCU(OPT_MCU_ESP32P4)
    #define CFG_TUH_MAX_RHPORT 2
  #else
    #define CFG_TUH_MAX_RHPORT 1
  #endif
#endif

// Enable multi-instance (dual host) support
// When disabled, code size is reduced by eliminating per-instance overhead
#ifndef CFG_TUH_MULTI_INSTANCE
  #define CFG_TUH_MULTI_INSTANCE (CFG_TUH_MAX_RHPORT > 1)
#endif

// Enable backward compatible API (single instance via macros)
// Recommended for existing applications
#ifndef CFG_TUH_BACKWARD_COMPATIBLE_API
  #define CFG_TUH_BACKWARD_COMPATIBLE_API 1
#endif
```

---

## Phase 8: Testing Strategy

### 8.1 Unit Tests

Create `test/host/test_dual_host.c`:

```c
void test_dual_instance_init(void) {
  // Initialize two instances
  tuh_instance_t inst0 = tuh_instance_init(0, &rhport0_init);
  tuh_instance_t inst1 = tuh_instance_init(1, &rhport1_init);

  TEST_ASSERT_NOT_NULL(inst0);
  TEST_ASSERT_NOT_NULL(inst1);
  TEST_ASSERT_NOT_EQUAL(inst0, inst1);

  // Verify both are initialized
  TEST_ASSERT_TRUE(tuh_instance_inited(inst0));
  TEST_ASSERT_TRUE(tuh_instance_inited(inst1));

  // Clean up
  tuh_instance_deinit(inst0);
  tuh_instance_deinit(inst1);
}

void test_dual_instance_isolation(void) {
  tuh_instance_t inst0 = tuh_instance_init(0, &rhport0_init);
  tuh_instance_t inst1 = tuh_instance_init(1, &rhport1_init);

  // Attach device to port 0
  simulate_device_attach(0, 1, TUSB_SPEED_FULL);
  tuh_task_instance(inst0);

  // Verify device is only on instance 0
  TEST_ASSERT_TRUE(tuh_instance_mounted(inst0, 1));
  TEST_ASSERT_FALSE(tuh_instance_mounted(inst1, 1));

  // Attach different device to port 1
  simulate_device_attach(1, 1, TUSB_SPEED_HIGH);
  tuh_task_instance(inst1);

  // Verify both instances have their own devices
  TEST_ASSERT_TRUE(tuh_instance_mounted(inst0, 1));
  TEST_ASSERT_TRUE(tuh_instance_mounted(inst1, 1));

  // Clean up
  tuh_instance_deinit(inst0);
  tuh_instance_deinit(inst1);
}
```

### 8.2 Integration Test Example

```c
// ESP32-P4 specific test
void test_esp32p4_dual_usb_host(void) {
  // Initialize both FS and HS controllers
  tusb_rhport_init_t fs_init = {.speed = TUSB_SPEED_FULL};
  tusb_rhport_init_t hs_init = {.speed = TUSB_SPEED_HIGH};

  tuh_instance_t fs_host = tuh_instance_init(0, &fs_init);  // Full-Speed
  tuh_instance_t hs_host = tuh_instance_init(1, &hs_init);  // High-Speed

  TEST_ASSERT_NOT_NULL(fs_host);
  TEST_ASSERT_NOT_NULL(hs_host);

  // Main loop: process both hosts
  for (int i = 0; i < 1000; i++) {
    tuh_task_instance(fs_host);
    tuh_task_instance(hs_host);
    vTaskDelay(pdMS_TO_TICKS(1));
  }

  // Or use convenience function
  // tuh_task_all();
}
```

---

## Implementation Checklist

### Core Infrastructure
- [ ] Define `tuh_instance_t` handle type
- [ ] Define `usbh_instance_t` internal structure
- [ ] Implement `tuh_instance_init()`
- [ ] Implement `tuh_instance_deinit()`
- [ ] Implement `tuh_task_instance()`
- [ ] Implement `tuh_task_all()`
- [ ] Add per-instance helper functions

### HCD Layer
- [ ] Convert `_hcd_data` to array `[CFG_TUH_MAX_RHPORT]`
- [ ] Add `get_hcd_data(rhport)` helper
- [ ] Update all HCD functions to use per-rhport data
- [ ] Test HCD layer isolation

### USBH Layer
- [ ] Update device enumeration to use instance
- [ ] Update control transfer to use instance
- [ ] Update endpoint operations to use instance
- [ ] Update event processing to use instance

### Class Drivers (per driver)
- [ ] Add `init_instance()` callback
- [ ] Add `deinit_instance()` callback
- [ ] Update `open()` to accept instance
- [ ] Update `set_config()` to accept instance
- [ ] Update `xfer_cb()` to accept instance
- [ ] Update `close()` to accept instance
- [ ] Convert static globals to per-instance allocation

### Configuration
- [ ] Add `CFG_TUH_MAX_RHPORT`
- [ ] Add `CFG_TUH_MULTI_INSTANCE`
- [ ] Add `CFG_TUH_BACKWARD_COMPATIBLE_API`
- [ ] Update documentation

### Testing
- [ ] Write unit tests for dual instance
- [ ] Write integration tests for ESP32-P4
- [ ] Test backward compatibility
- [ ] Stress test concurrent operations
- [ ] Test error handling and cleanup

---

## Timeline Estimate

- **Phase 1-2 (Core)**: 2-3 days
- **Phase 3 (USBH)**: 3-4 days
- **Phase 4 (HCD)**: 2-3 days
- **Phase 5 (Class Drivers)**: 5-7 days (1 day per major driver)
- **Phase 6 (Compatibility)**: 1 day
- **Phase 7 (Config)**: 1 day
- **Phase 8 (Testing)**: 3-5 days

**Total**: 17-26 days of focused development

---

## Risk Mitigation

1. **Incremental approach**: Implement phase by phase, testing each
2. **Backward compatibility**: Keep old API working via macros
3. **Feature flag**: Make multi-instance optional via `CFG_TUH_MULTI_INSTANCE`
4. **Code review**: Each phase should be reviewed before proceeding
5. **Testing**: Continuous testing with both single and dual instance configs

---

## Success Criteria

1. ✅ Two USB Host controllers can run simultaneously on ESP32-P4
2. ✅ Each controller enumerates devices independently
3. ✅ Each controller handles transfers without interfering with the other
4. ✅ Existing single-controller code continues to work unchanged
5. ✅ Memory overhead is minimal when `CFG_TUH_MAX_RHPORT=1`
6. ✅ All class drivers support per-instance operation
7. ✅ Comprehensive test coverage for dual-instance scenarios
