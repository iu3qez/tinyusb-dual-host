# TinyUSB Dual USB Host Support Analysis for ESP32-P4

## Executive Summary

The ESP32-P4 has **hardware support** for dual USB OTG controllers (High-Speed and Full-Speed), but **TinyUSB's software architecture currently prevents using both as USB Hosts simultaneously**. This document analyzes the limitation and proposes a solution.

---

## Current Architecture

### Hardware Layer (✅ Already Supports Multiple Controllers)

**File**: `src/portable/synopsys/dwc2/dwc2_esp32.h`

The ESP32-P4 hardware abstraction already defines **two separate USB controllers**:

```c
#elif TU_CHECK_MCU(OPT_MCU_ESP32P4)
#define DWC2_FS_REG_BASE   0x50040000UL  // Full-Speed controller
#define DWC2_HS_REG_BASE   0x50000000UL  // High-Speed controller

// Array of 2 controllers - hardware supports both!
static const dwc2_controller_t _dwc2_controller[] = {
  { .reg_base = DWC2_FS_REG_BASE, .irqnum = ETS_USB_OTG11_CH0_INTR_SOURCE,
    .ep_count = 7, .ep_in_count = 5, .ep_fifo_size = 1024 },   // Port 0: FS
  { .reg_base = DWC2_HS_REG_BASE, .irqnum = ETS_USB_OTG_INTR_SOURCE,
    .ep_count = 16, .ep_in_count = 8, .ep_fifo_size = 4096 }   // Port 1: HS
};

// Interrupt handles - one per controller
static intr_handle_t usb_ih[TU_ARRAY_SIZE(_dwc2_controller)];
```

**Key observations**:
- ✅ Hardware layer already supports arrays indexed by `rhport`
- ✅ Separate interrupt handlers per controller
- ✅ Different register bases, IRQ numbers, and capabilities per controller

---

### Host Controller Driver (HCD) Layer (❌ SINGLE GLOBAL INSTANCE)

**File**: `src/portable/synopsys/dwc2/hcd_dwc2.c`

The DWC2 HCD maintains **a single global data structure**:

```c
typedef struct {
  hcd_xfer_t xfer[DWC2_CHANNEL_COUNT_MAX];        // Transfer state per channel
  hcd_endpoint_t edpt[CFG_TUH_DWC2_ENDPOINT_MAX]; // Endpoint state
} hcd_data_t;

hcd_data_t _hcd_data;  // ❌ SINGLE GLOBAL INSTANCE - THIS IS THE PROBLEM
```

**Problem**: All HCD operations (channel allocation, endpoint management, transfers) use this single `_hcd_data` instance, preventing concurrent operation of multiple controllers.

---

### USB Host Stack (USBH) Layer (❌ SINGLE GLOBAL INSTANCE)

**File**: `src/host/usbh.c`

The high-level host stack also uses global static variables:

```c
#define TOTAL_DEVICES (CFG_TUH_DEVICE_MAX + CFG_TUH_HUB)

// ❌ All static - single instance only
static usbh_device_t _usbh_devices[TOTAL_DEVICES];
static osal_queue_t _usbh_q;                     // Event queue
static osal_mutex_t _usbh_mutex;                 // Mutex for claiming endpoints
static usbh_data_t _usbh_data = {
  .controller_id = TUSB_INDEX_INVALID_8,         // Only ONE active controller
  .enumerating_daddr = TUSB_INDEX_INVALID_8,
  .ctrl_xfer_info = { ... }                      // Single control transfer
};
```

**Critical limitation** in `tuh_rhport_init()`:

```c
bool tuh_rhport_init(uint8_t rhport, const tusb_rhport_init_t* rh_init) {
  if (tuh_rhport_is_active(rhport)) {
    return true; // skip if already initialized
  }

  // Init host stack if not already
  if (!tuh_inited()) {
    // ... initialize global structures once
  }

  // Init host controller
  _usbh_data.controller_id = rhport;  // ❌ OVERWRITES previous rhport!
  TU_ASSERT(hcd_init(rhport, rh_init));
  hcd_int_enable(rhport);

  return true;
}
```

**Problems**:
1. `_usbh_data.controller_id` can only track **ONE** active controller
2. `tuh_rhport_is_active()` returns `_usbh_data.controller_id == rhport` - only one port can be "active"
3. Device array `_usbh_devices[]` is shared across all controllers
4. Single event queue `_usbh_q` mixes events from all controllers
5. Single control transfer state - can't handle concurrent control transfers

---

## Root Cause Analysis

The issue identified in GitHub issue #15810 applies to both Device and Host stacks:

> "there is only one internal static variable `_usbd_rhport` which will be initialized after first stack initialization and it prevents to initialize stack again"
>
> "we don't have TinyUSB instance handle, so there are lots of internal variables which should be shared between instances running on two different peripherals (such as buffers for control transfer and so on)"

**For USB Host**, the equivalent problems are:
1. No instance handle - all operations reference global `_hcd_data` and `_usbh_data`
2. Single controller ID tracking in `_usbh_data.controller_id`
3. Shared device array without rhport association
4. Single event queue for all controllers
5. Single control transfer state machine

---

## Proposed Solution Architecture

### Phase 1: Make HCD Per-RHPort (Host Controller Driver Layer)

**File**: `src/portable/synopsys/dwc2/hcd_dwc2.c`

Change from single global to per-rhport array:

```c
// Before (current):
hcd_data_t _hcd_data;

// After (proposed):
#ifndef CFG_TUH_MAX_RHPORT
  #define CFG_TUH_MAX_RHPORT 2  // ESP32-P4 has 2 USB OTG controllers
#endif

hcd_data_t _hcd_data[CFG_TUH_MAX_RHPORT];
```

**Required changes**:
- Update all HCD functions to accept `rhport` and index into `_hcd_data[rhport]`
- Channel allocation becomes per-controller: `channel_alloc(_hcd_data[rhport])`
- Endpoint management becomes per-controller
- Transfer state isolated per controller

**Impact**: ~50-100 function signatures need `rhport` parameter added/used

---

### Phase 2: Make USBH Per-RHPort (USB Host Stack Layer)

**File**: `src/host/usbh.c`

Convert global structures to per-rhport:

```c
typedef struct {
  uint8_t controller_id;              // Still needed for validation
  uint8_t enumerating_daddr;
  uint8_t attach_debouncing_bm;
  tuh_bus_info_t dev0_bus;
  usbh_ctrl_xfer_info_t ctrl_xfer_info;

  // Per-controller resources
  osal_queue_t event_queue;
  osal_mutex_t mutex;
  usbh_device_t devices[TOTAL_DEVICES];
} usbh_controller_t;

static usbh_controller_t _usbh_controllers[CFG_TUH_MAX_RHPORT];
```

**Alternative approach** (less invasive):
- Keep device array global but add `rhport` field to `usbh_device_t`
- Make event queue per-rhport
- Make control transfer per-rhport
- Track multiple active `controller_id` values

---

### Phase 3: Update Class Drivers

Each class driver would need updates:

**Example - CDC Host** (`src/class/cdc/cdc_host.c`):
```c
// Current: single global array
static cdch_interface_t _cdch_itf[CFG_TUH_CDC];

// Proposed: per-rhport or add rhport tracking
static cdch_interface_t _cdch_itf[CFG_TUH_CDC];
// Add rhport field to cdch_interface_t to track which controller owns it
```

**Minimal approach**:
- Add `rhport` field to all interface/device tracking structures
- Filter operations by rhport when needed
- Allows single global array but with rhport awareness

---

## Implementation Strategy

### Option A: Full Refactor (Comprehensive but Invasive)

1. Add `CFG_TUH_MAX_RHPORT` configuration option
2. Convert all HCD static data to `[CFG_TUH_MAX_RHPORT]` arrays
3. Convert all USBH static data to per-rhport structures
4. Thread `rhport` parameter through all function calls
5. Update all class drivers

**Pros**: Clean architecture, fully supports multiple controllers
**Cons**: Large code changes, high risk of regressions

---

### Option B: Hybrid Approach (Pragmatic)

1. Make HCD per-rhport (Phase 1 only)
2. Make USBH event queue and control transfer per-rhport
3. Keep device array global but add `rhport` field to devices
4. Add rhport filtering in class drivers where needed

**Pros**: Smaller changeset, lower risk
**Cons**: Some global state remains, less clean architecture

---

### Option C: Instance Handle API (Future-Proof)

Add an opaque instance handle to the API:

```c
typedef struct tuh_instance_t* tuh_handle_t;

// New API
tuh_handle_t tuh_init_instance(uint8_t rhport, const tusb_rhport_init_t* rh_init);
void tuh_task_instance(tuh_handle_t handle);
bool tuh_xfer_instance(tuh_handle_t handle, uint8_t daddr, ...);

// Backwards compatibility: global instance
tuh_handle_t tuh_get_default_instance(void);
#define tuh_task() tuh_task_instance(tuh_get_default_instance())
```

**Pros**:
- Clean abstraction, supports unlimited controllers
- Backward compatible with defines
- Aligns with TinyUSB upstream feature request

**Cons**:
- Most invasive change
- Requires API versioning strategy
- Upstream TinyUSB coordination needed

---

## Configuration Options

Add to `tusb_option.h`:

```c
// Maximum number of root hub ports (USB controllers) that can operate simultaneously as hosts
#ifndef CFG_TUH_MAX_RHPORT
  #if TU_CHECK_MCU(OPT_MCU_ESP32P4)
    #define CFG_TUH_MAX_RHPORT 2  // ESP32-P4 has FS and HS controllers
  #else
    #define CFG_TUH_MAX_RHPORT 1  // Default: single controller
  #endif
#endif

// Enable dual host support (requires per-rhport data structures)
#ifndef CFG_TUH_DUAL_HOST_ENABLE
  #define CFG_TUH_DUAL_HOST_ENABLE (CFG_TUH_MAX_RHPORT > 1)
#endif
```

---

## Complexity Estimate

### Phase 1: HCD Layer (Moderate)
- **Files to modify**: 1 main file (`hcd_dwc2.c`)
- **Functions to update**: ~30-50 functions
- **Estimated LOC changed**: 200-300 lines
- **Risk**: Medium (isolated to HCD layer)

### Phase 2: USBH Layer (High)
- **Files to modify**: 2 main files (`usbh.c`, `usbh.h`)
- **Functions to update**: ~80-120 functions
- **Estimated LOC changed**: 400-600 lines
- **Risk**: High (core stack changes)

### Phase 3: Class Drivers (Variable)
- **Files to modify**: 6-10 class driver files
- **Functions to update**: ~20-40 per class driver
- **Estimated LOC changed**: 300-500 lines
- **Risk**: Medium (class-specific, parallel work possible)

**Total Estimate**: 900-1400 LOC changed across 10-15 files

---

## Testing Strategy

1. **Unit tests**: Per-rhport initialization, concurrent enumeration
2. **Integration tests**:
   - Single device on each controller simultaneously
   - Hub on one controller, device on another
   - Class driver operations (CDC on port 0, MSC on port 1)
3. **Stress tests**:
   - Concurrent transfers on both ports
   - Hotplug while other port active
   - Error handling with both ports

---

## Upstream Coordination

This work aligns with:
- **ESP-IDF Issue #15810**: "Support more than one TinyUSB instance on ESP32-P4"
- **TinyUSB Issue #3092**: Upstream feature request for multi-instance support

**Recommendation**: Implement Option B (Hybrid Approach) as a patch for ESP-IDF's TinyUSB fork, then propose Option C (Instance Handle API) to upstream TinyUSB for long-term solution.

---

## Conclusion

### Why It's a Software Limitation

The ESP32-P4 **hardware fully supports** dual USB Host controllers:
- ✅ Separate register bases
- ✅ Separate interrupts
- ✅ Separate PHYs
- ✅ Hardware layer already has per-controller arrays

The **software limitation** is purely architectural:
- ❌ HCD uses single global `_hcd_data`
- ❌ USBH uses single global `_usbh_data`
- ❌ No rhport tracking in device structures
- ❌ Single event queue and control transfer state machine

### Feasibility

Dual USB Host support is **feasible** but requires significant refactoring:
- **Minimal viable**: HCD + USBH event queue per-rhport (~500 LOC)
- **Full support**: All layers per-rhport (~1200 LOC)
- **Future-proof**: Instance handle API (~1500+ LOC)

### Recommendation for ESPHome

For the ESPHome USB Audio use case:
1. **Short term**: Continue using single USB Host (HS or FS, user choice)
2. **Medium term**: Monitor ESP-IDF progress on dual host support
3. **Long term**: Contribute to or adopt upstream TinyUSB multi-instance work

The limitation is **solvable but non-trivial** - requires coordinated effort between ESP-IDF and TinyUSB upstream.
