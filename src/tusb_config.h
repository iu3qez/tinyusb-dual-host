/*
 * TinyUSB Configuration for ESPHome USB Host
 *
 * This configuration enables all available USB host classes for maximum
 * device compatibility.
 */

#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// Common Configuration
//--------------------------------------------------------------------

// MCU type - defined by build system
#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

// No RTOS for now (ESPHome runs on Arduino/ESP-IDF event loop)
#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

// Debug level (0 = no debug, 3 = verbose)
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// Memory alignment for DMA transfers
#ifndef CFG_TUH_MEM_SECTION
#define CFG_TUH_MEM_SECTION
#endif

#ifndef CFG_TUH_MEM_ALIGN
#define CFG_TUH_MEM_ALIGN __attribute__((aligned(4)))
#endif

//--------------------------------------------------------------------
// Host Stack Configuration
//--------------------------------------------------------------------

// Enable USB Host stack
#define CFG_TUH_ENABLED 1

// Default max speed (let hardware decide)
#ifndef BOARD_TUH_MAX_SPEED
#define BOARD_TUH_MAX_SPEED OPT_MODE_DEFAULT_SPEED
#endif

#define CFG_TUH_MAX_SPEED BOARD_TUH_MAX_SPEED

// Root hub port (default to 0 for built-in USB controller)
#ifndef BOARD_TUH_RHPORT
#define BOARD_TUH_RHPORT 0
#endif

//--------------------------------------------------------------------
// Host Driver Configuration
//--------------------------------------------------------------------

// Buffer for enumeration and descriptor reading
#define CFG_TUH_ENUMERATION_BUFSIZE 512

// Task queue size for USB host events
#define CFG_TUH_TASK_QUEUE_SZ 16

// Maximum number of interfaces per device
#define CFG_TUH_INTERFACE_MAX 8

// Maximum number of USB hubs supported
#define CFG_TUH_HUB 1

// Maximum number of devices (excluding hubs)
// 1 hub typically has 4 ports, so allow 4 devices
#define CFG_TUH_DEVICE_MAX (4 * CFG_TUH_HUB + 1)

//--------------------------------------------------------------------
// USB Host Class Drivers
// ESPHome uses ESP-IDF's USB class drivers (esp-usb), not TinyUSB's
// Disable all TinyUSB class drivers to avoid conflicts
//--------------------------------------------------------------------

#define CFG_TUH_MSC 0        // Use esp-usb MSC driver instead
#define CFG_TUH_HID 0        // Use esp-usb HID driver instead
#define CFG_TUH_CDC 0        // Use esp-usb CDC driver instead
#define CFG_TUH_CDC_FTDI 0   // Use esp-usb CDC driver instead
#define CFG_TUH_CDC_CP210X 0 // Use esp-usb CDC driver instead
#define CFG_TUH_CDC_CH34X 0  // Use esp-usb CDC driver instead
#define CFG_TUH_CDC_PL2303 0 // Use esp-usb CDC driver instead
#define CFG_TUH_MIDI 0       // Use esp-usb MIDI driver instead
#define CFG_TUH_VENDOR 0     // Use esp-usb vendor driver instead

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H_ */
