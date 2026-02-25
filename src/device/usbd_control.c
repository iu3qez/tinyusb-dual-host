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

#include "dcd.h"
#include "tusb.h"
#include "device/usbd_pvt.h"

//--------------------------------------------------------------------+
// Callback weak stubs (called if application does not provide)
//--------------------------------------------------------------------+
TU_ATTR_WEAK void dcd_edpt0_status_complete(uint8_t rhport, const tusb_control_request_t* request) {
  (void) rhport;
  (void) request;
}

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+

enum {
  EDPT_CTRL_OUT = 0x00,
  EDPT_CTRL_IN = 0x80
};

// Control transfer state and endpoint buffer are now per-instance in usbd_instance_t
// (usbd_control_xfer_t and ctrl_epbuf defined in usbd_pvt.h)

//--------------------------------------------------------------------+
// Application API
//--------------------------------------------------------------------+

// Queue ZLP status transaction
static inline bool status_stage_xact(uint8_t rhport, const tusb_control_request_t* request) {
  // Opposite to endpoint in Data Phase
  const uint8_t ep_addr = request->bmRequestType_bit.direction ? EDPT_CTRL_OUT : EDPT_CTRL_IN;
  return usbd_edpt_xfer(rhport, ep_addr, NULL, 0, false);
}

// Status phase
bool tud_control_status(uint8_t rhport, const tusb_control_request_t* request) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  TU_ASSERT(inst);

  inst->ctrl_xfer.request = (*request);
  inst->ctrl_xfer.buffer = NULL;
  inst->ctrl_xfer.total_xferred = 0;
  inst->ctrl_xfer.data_len = 0;

  return status_stage_xact(rhport, request);
}

// Queue a transaction in Data Stage
static bool data_stage_xact(uint8_t rhport) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  TU_ASSERT(inst);

  const uint16_t xact_len = tu_min16(inst->ctrl_xfer.data_len - inst->ctrl_xfer.total_xferred, CFG_TUD_ENDPOINT0_BUFSIZE);
  uint8_t ep_addr = EDPT_CTRL_OUT;

  if (inst->ctrl_xfer.request.bmRequestType_bit.direction == TUSB_DIR_IN) {
    ep_addr = EDPT_CTRL_IN;
    if (0u != xact_len) {
      TU_VERIFY(0 == tu_memcpy_s(inst->ctrl_epbuf.buf, CFG_TUD_ENDPOINT0_BUFSIZE, inst->ctrl_xfer.buffer, xact_len));
    }
  }

  return usbd_edpt_xfer(rhport, ep_addr, xact_len ? inst->ctrl_epbuf.buf : NULL, xact_len, false);
}

// Transmit data to/from the control endpoint.
bool tud_control_xfer(uint8_t rhport, const tusb_control_request_t* request, void* buffer, uint16_t len) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  TU_ASSERT(inst);

  inst->ctrl_xfer.request = (*request);
  inst->ctrl_xfer.buffer = (uint8_t*) buffer;
  inst->ctrl_xfer.total_xferred = 0U;
  inst->ctrl_xfer.data_len = tu_min16(len, request->wLength);

  if (request->wLength > 0U) {
    if (inst->ctrl_xfer.data_len > 0U) {
      TU_ASSERT(buffer);
    }
    TU_ASSERT(data_stage_xact(rhport));
  } else {
    TU_ASSERT(status_stage_xact(rhport, request));
  }

  return true;
}

//--------------------------------------------------------------------+
// USBD API (called from usbd.c)
//--------------------------------------------------------------------+
void usbd_control_reset(uint8_t rhport);
void usbd_control_set_request(uint8_t rhport, const tusb_control_request_t* request);
void usbd_control_set_complete_callback(uint8_t rhport, usbd_control_xfer_cb_t fp);
bool usbd_control_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes);

void usbd_control_reset(uint8_t rhport) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  if (inst) {
    tu_varclr(&inst->ctrl_xfer);
  }
}

// Set complete callback
void usbd_control_set_complete_callback(uint8_t rhport, usbd_control_xfer_cb_t fp) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  if (inst) {
    inst->ctrl_xfer.complete_cb = fp;
  }
}

// for dcd_set_address where DCD is responsible for status response
void usbd_control_set_request(uint8_t rhport, const tusb_control_request_t* request) {
  usbd_instance_t* inst = usbd_get_instance(rhport);
  if (inst) {
    inst->ctrl_xfer.request = (*request);
    inst->ctrl_xfer.buffer = NULL;
    inst->ctrl_xfer.total_xferred = 0;
    inst->ctrl_xfer.data_len = 0;
  }
}

// callback when a transaction complete on
// - DATA stage of control endpoint or
// - Status stage
bool usbd_control_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes) {
  (void) result;

  usbd_instance_t* inst = usbd_get_instance(rhport);
  TU_ASSERT(inst);

  usbd_control_xfer_t* xfer = &inst->ctrl_xfer;

  // Endpoint Address is opposite to direction bit, this is Status Stage complete event
  if (tu_edpt_dir(ep_addr) != xfer->request.bmRequestType_bit.direction) {
    TU_ASSERT(0 == xferred_bytes);

    // invoke optional dcd hook if available
    dcd_edpt0_status_complete(rhport, &xfer->request);

    if (NULL != xfer->complete_cb) {
      // TODO refactor with usbd_driver_print_control_complete_name
      xfer->complete_cb(rhport, CONTROL_STAGE_ACK, &xfer->request);
    }

    return true;
  }

  if (xfer->request.bmRequestType_bit.direction == TUSB_DIR_OUT) {
    TU_VERIFY(xfer->buffer);
    memcpy(xfer->buffer, inst->ctrl_epbuf.buf, xferred_bytes);
    TU_LOG_MEM(CFG_TUD_LOG_LEVEL, xfer->buffer, xferred_bytes, 2);
  }

  xfer->total_xferred += (uint16_t) xferred_bytes;
  xfer->buffer += xferred_bytes;

  // Data Stage is complete when all request's length are transferred or
  // a short packet is sent including zero-length packet.
  if ((xfer->request.wLength == xfer->total_xferred) ||
      (xferred_bytes < CFG_TUD_ENDPOINT0_BUFSIZE)) {
    // DATA stage is complete
    bool is_ok = true;

    // invoke complete callback if set
    if (NULL != xfer->complete_cb) {
      #if CFG_TUSB_DEBUG >= CFG_TUD_LOG_LEVEL
      usbd_driver_print_control_complete_name(xfer->complete_cb);
      #endif

      is_ok = xfer->complete_cb(rhport, CONTROL_STAGE_DATA, &xfer->request);
    }

    if (is_ok) {
      TU_ASSERT(status_stage_xact(rhport, &xfer->request));
    } else {
      // Stall both IN and OUT control endpoint
      dcd_edpt_stall(rhport, EDPT_CTRL_OUT);
      dcd_edpt_stall(rhport, EDPT_CTRL_IN);
    }
  } else {
    // More data to transfer
    TU_ASSERT(data_stage_xact(rhport));
  }

  return true;
}

#endif
