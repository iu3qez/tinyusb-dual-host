# TinyUSB Dual Host - Proof of Concept Plan

## Goal
Create a minimal, testable implementation that allows ESP32-P4 to initialize and run both USB Host controllers simultaneously. Focus on getting something working first, then refine.

## Scope for POC

### What We WILL Implement
1. ✅ Per-rhport HCD data structures (DWC2 layer)
2. ✅ Per-rhport USBH instance structures
3. ✅ Basic dual initialization
4. ✅ Independent event queues per controller
5. ✅ Simple task loop that processes both controllers
6. ✅ Basic device enumeration on both ports

### What We WON'T Implement (for now)
- ❌ Full instance handle API (too much work for POC)
- ❌ Class driver per-instance refactoring (use globals, add rhport tracking)
- ❌ Backward compatibility macros
- ❌ Comprehensive testing framework
- ❌ Multiple class driver support (start with MSC or CDC only)

## Implementation Strategy

### Phase 1: Minimal HCD Changes (Day 1)

**Goal**: Make DWC2 HCD support per-rhport data without breaking existing code.

**File**: `src/portable/synopsys/dwc2/hcd_dwc2.c`

```c
// Change from:
hcd_data_t _hcd_data;

// To:
#ifndef CFG_TUH_MAX_RHPORT
  #define CFG_TUH_MAX_RHPORT 2
#endif

static hcd_data_t _hcd_data[CFG_TUH_MAX_RHPORT];

// Add helper (used throughout file)
#define HCD_DATA(rhport) (&_hcd_data[rhport])

// Example update:
static void edpt_close(dwc2_regs_t *dwc2, uint8_t rhport, uint8_t ep_id) {
  hcd_endpoint_t *edpt = &HCD_DATA(rhport)->edpt[ep_id];
  // ... rest unchanged
}
```

**Testing**: Compile and verify existing single-controller code still works.

---

### Phase 2: Minimal USBH Changes (Day 1-2)

**Goal**: Allow USBH to track multiple active controllers.

**File**: `src/host/usbh.c`

```c
// Keep most globals, just make critical parts per-rhport
typedef struct {
  uint8_t controller_id;
  uint8_t enumerating_daddr;
  uint8_t attach_debouncing_bm;
  tuh_bus_info_t dev0_bus;
  usbh_ctrl_xfer_info_t ctrl_xfer;

  // Per-rhport event queue
  OSAL_QUEUE_DEF(rhport_qdef, CFG_TUH_TASK_QUEUE_SZ, hcd_event_t);
  osal_queue_t event_queue;

  bool initialized;
} usbh_rhport_data_t;

static usbh_rhport_data_t _usbh_rhports[CFG_TUH_MAX_RHPORT];

// Shared resources (for now - will refactor later if needed)
static usbh_device_t _usbh_devices[CFG_TUH_MAX_RHPORT][TOTAL_DEVICES];
static osal_mutex_t _usbh_mutex;  // Global for now
static osal_spinlock_t _usbh_spin;

#define RHPORT_DATA(rhport) (&_usbh_rhports[rhport])
```

**Key changes to `tuh_rhport_init()`:**

```c
bool tuh_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  TU_VERIFY(rhport < CFG_TUH_MAX_RHPORT, false);

  usbh_rhport_data_t* rhport_data = RHPORT_DATA(rhport);

  if (rhport_data->initialized) {
    TU_LOG_USBH("rhport %u already initialized\r\n", rhport);
    return true;
  }

  // Initialize global resources once (first rhport only)
  static bool global_init = false;
  if (!global_init) {
    osal_spin_init(&_usbh_spin);

#if OSAL_MUTEX_REQUIRED
    _usbh_mutex = osal_mutex_create(&_usbh_mutexdef);
    TU_ASSERT(_usbh_mutex);
#endif

    // Initialize class drivers (global for now)
    for (uint8_t drv_id = 0; drv_id < TOTAL_DRIVER_COUNT; drv_id++) {
      usbh_class_driver_t const* driver = get_driver(drv_id);
      if (driver != NULL) {
        TU_LOG_USBH("%s init\r\n", driver->name);
        driver->init();
      }
    }

    global_init = true;
  }

  // Initialize per-rhport resources
  rhport_data->controller_id = rhport;
  rhport_data->enumerating_daddr = TUSB_INDEX_INVALID_8;

  // Create per-rhport event queue
  rhport_data->event_queue = osal_queue_create(&rhport_data->rhport_qdef);
  TU_ASSERT(rhport_data->event_queue != NULL);

  // Initialize devices for this rhport
  for (uint8_t i = 0; i < TOTAL_DEVICES; i++) {
    clear_device(&_usbh_devices[rhport][i]);
  }

  // Initialize HCD
  rhport_data->controller_id = rhport;
  TU_ASSERT(hcd_init(rhport, rh_init));
  hcd_int_enable(rhport);

  rhport_data->initialized = true;

  return true;
}
```

**Testing**: Verify both rhports can initialize without crashing.

---

### Phase 3: Dual Task Loop (Day 2)

**Goal**: Process events from both controllers.

```c
bool tuh_task_event_ready(void) {
  // Check if ANY rhport has events ready
  for (uint8_t rhport = 0; rhport < CFG_TUH_MAX_RHPORT; rhport++) {
    if (RHPORT_DATA(rhport)->initialized &&
        !osal_queue_empty(RHPORT_DATA(rhport)->event_queue)) {
      return true;
    }
  }
  return false;
}

void tuh_task_ext(uint8_t rhport, bool in_isr) {
  usbh_rhport_data_t* rhport_data = RHPORT_DATA(rhport);

  if (!rhport_data->initialized) {
    return;
  }

  // Process events from this rhport's queue
  hcd_event_t event;
  while (osal_queue_receive(rhport_data->event_queue, &event)) {
    // Verify event is for this rhport
    TU_ASSERT(event.rhport == rhport, );

    switch (event.event_id) {
      case HCD_EVENT_DEVICE_ATTACH:
        process_device_attach(rhport, event.connection.hub_addr,
                            event.connection.hub_port, event.connection.speed);
        break;

      case HCD_EVENT_DEVICE_REMOVE:
        process_device_remove(rhport, event.connection.hub_addr,
                             event.connection.hub_port);
        break;

      case HCD_EVENT_XFER_COMPLETE:
        process_xfer_complete(rhport, event.dev_addr,
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

// Original tuh_task() calls both rhports
void tuh_task(void) {
  for (uint8_t rhport = 0; rhport < CFG_TUH_MAX_RHPORT; rhport++) {
    tuh_task_ext(rhport, false);
  }
}
```

**Testing**: Verify task loop doesn't crash with both rhports initialized.

---

### Phase 4: Event Queue Routing (Day 2-3)

**Goal**: Ensure HCD events go to the correct rhport queue.

**File**: `src/host/usbh.c`

Update event posting to use correct queue:

```c
// Helper to get the right queue for rhport
static inline osal_queue_t usbh_get_event_queue(uint8_t rhport) {
  return RHPORT_DATA(rhport)->event_queue;
}

// Update osal_queue_send calls throughout
void hcd_event_handler(hcd_event_t const* event, bool in_isr) {
  osal_queue_t queue = usbh_get_event_queue(event->rhport);
  osal_queue_send(queue, event, in_isr);
}
```

**Testing**: Attach device to port 0, verify events only go to port 0 queue.

---

### Phase 5: Device Array Isolation (Day 3)

**Goal**: Ensure devices enumerated on port 0 don't interfere with port 1.

All device access needs rhport parameter:

```c
// Update device accessor functions
static inline usbh_device_t* get_device(uint8_t rhport, uint8_t dev_addr) {
  TU_VERIFY(rhport < CFG_TUH_MAX_RHPORT && dev_addr < TOTAL_DEVICES, NULL);
  return &_usbh_devices[rhport][dev_addr];
}

// Update all device access throughout the file
void process_device_attach(uint8_t rhport, uint8_t hub_addr, uint8_t hub_port, tusb_speed_t speed) {
  usbh_device_t* dev = get_device(rhport, 0);  // Use dev0 for enumeration
  // ... rest of logic
}
```

**Testing**:
1. Attach device to port 0, verify it enumerates
2. Attach device to port 1, verify it enumerates independently
3. Verify both devices can be accessed simultaneously

---

### Phase 6: Class Driver RHPort Tracking (Day 3-4)

**Goal**: Make class drivers aware of which rhport owns each device.

**Minimal approach**: Add rhport to existing structures.

**Example - MSC Driver** (`src/class/msc/msc_host.c`):

```c
typedef struct {
  uint8_t rhport;  // ADD THIS
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;
  // ... rest unchanged
} msch_interface_t;

// Update open function
bool msch_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len) {
  // ... existing logic ...

  // Store rhport for this interface
  msc_itf->rhport = rhport;

  // ... rest unchanged
}

// Update API functions to verify rhport
bool tuh_msc_mounted(uint8_t dev_addr) {
  // Find interface for this dev_addr
  msch_interface_t* msc_itf = find_interface_by_addr(dev_addr);
  if (msc_itf == NULL) return false;

  // Verify device is mounted on its rhport
  return tuh_mounted(dev_addr);  // Will check correct rhport internally
}
```

**Testing**:
1. Mount MSC device on port 0
2. Mount MSC device on port 1
3. Verify file operations work on both independently

---

## Test Application for ESP32-P4

Create `examples/host/dual_host_test/dual_host_test.c`:

```c
#include "tusb.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "dual_host";

void usb_host_task(void* param) {
  while (1) {
    tuh_task();  // Process both rhports
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "TinyUSB Dual Host Test");

  // Initialize Full-Speed USB (rhport 0)
  tusb_rhport_init_t fs_init = {
    .speed = TUSB_SPEED_FULL
  };

  if (tuh_rhport_init(0, &fs_init)) {
    ESP_LOGI(TAG, "USB FS (rhport 0) initialized");
  } else {
    ESP_LOGE(TAG, "USB FS (rhport 0) init failed");
  }

  // Initialize High-Speed USB (rhport 1)
  tusb_rhport_init_t hs_init = {
    .speed = TUSB_SPEED_HIGH
  };

  if (tuh_rhport_init(1, &hs_init)) {
    ESP_LOGI(TAG, "USB HS (rhport 1) initialized");
  } else {
    ESP_LOGE(TAG, "USB HS (rhport 1) init failed");
  }

  // Start USB host task
  xTaskCreate(usb_host_task, "usb_host", 4096, NULL, 5, NULL);

  ESP_LOGI(TAG, "Dual USB Host running");
  ESP_LOGI(TAG, "Connect USB devices to both ports");

  // Monitor loop
  while (1) {
    // Check for mounted devices on both ports
    for (uint8_t rhport = 0; rhport < 2; rhport++) {
      for (uint8_t dev_addr = 1; dev_addr < CFG_TUH_DEVICE_MAX; dev_addr++) {
        if (tuh_mounted(dev_addr)) {
          // Get device info
          tuh_device_info_t info;
          if (tuh_device_get_info(dev_addr, &info)) {
            if (info.rhport == rhport) {
              ESP_LOGI(TAG, "Device %u on rhport %u: VID=%04x PID=%04x Speed=%s",
                      dev_addr, rhport,
                      info.vid, info.pid,
                      info.speed == TUSB_SPEED_HIGH ? "HS" :
                      info.speed == TUSB_SPEED_FULL ? "FS" : "LS");
            }
          }
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Callbacks
void tuh_mount_cb(uint8_t dev_addr) {
  ESP_LOGI(TAG, "Device mounted: addr=%u", dev_addr);
}

void tuh_umount_cb(uint8_t dev_addr) {
  ESP_LOGI(TAG, "Device unmounted: addr=%u", dev_addr);
}
```

## Testing Plan

### Test 1: Basic Initialization
1. Flash test app to ESP32-P4
2. Check serial output for both rhport init messages
3. Verify no crashes or errors

**Expected output**:
```
USB FS (rhport 0) initialized
USB HS (rhport 1) initialized
Dual USB Host running
```

### Test 2: Single Device Enumeration
1. Connect USB device to FS port only
2. Verify device enumerates and mounts
3. Check device is reported on rhport 0

**Expected output**:
```
Device mounted: addr=1
Device 1 on rhport 0: VID=xxxx PID=xxxx Speed=FS
```

### Test 3: Dual Device Enumeration
1. Connect USB device to FS port
2. Connect different USB device to HS port
3. Verify both enumerate independently
4. Check each device reports correct rhport

**Expected output**:
```
Device mounted: addr=1
Device 1 on rhport 0: VID=xxxx PID=xxxx Speed=FS
Device mounted: addr=1
Device 1 on rhport 1: VID=yyyy PID=yyyy Speed=HS
```

### Test 4: Hotplug
1. With both devices connected, unplug FS device
2. Verify only FS device unmounts
3. Verify HS device continues working
4. Replug FS device, verify it re-enumerates

### Test 5: Mass Storage (if using MSC)
1. Connect USB flash drive to each port
2. Verify both show up as separate devices
3. Read files from both simultaneously
4. Verify no data corruption or interference

## Success Criteria

✅ **Minimal POC Success**:
- Both rhports initialize without errors
- Devices can enumerate on either port independently
- Devices on different ports don't interfere with each other
- Basic class driver (MSC or CDC) works on both ports

✅ **Full POC Success**:
- All of above, plus:
- Concurrent transfers on both ports work correctly
- Hotplug works on both ports independently
- No crashes after 1 hour of operation
- At least one class driver fully functional on both ports

## Timeline

- **Day 1**: Phase 1-2 (HCD + USBH structure changes)
- **Day 2**: Phase 3-4 (Task loop + event routing)
- **Day 3**: Phase 5-6 (Device isolation + class driver)
- **Day 4**: Test application + basic testing
- **Day 5**: Bug fixes + validation

**Total**: ~1 week for working POC

## Next Steps After POC

Once POC works:
1. Clean up code and add comments
2. Add proper error handling
3. Test with multiple class drivers (CDC, HID, MSC)
4. Performance testing and optimization
5. Consider upstream contribution strategy
