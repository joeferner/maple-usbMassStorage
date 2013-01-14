
#include "usb_mass.h"

#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"

void usb_mass_init_usblib(usblib_dev *dev, void (**ep_int_in)(void), void (**ep_int_out)(void));

/*
 * Endpoint callbacks
 */

static void (*ep_mass_int_in[7])(void) = {
  NOP_Process, // TODO vcomDataTxCb,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process
};

static void (*ep_mass_int_out[7])(void) = {
  NOP_Process,
  NOP_Process,
  NOP_Process, // TODO vcomDataRxCb,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process
};

void usb_mass_enable(gpio_dev *disc_dev, uint8 disc_bit) {
  /* Present ourselves to the host. Writing 0 to "disc" pin must
   * pull USB_DP pin up while leaving USB_DM pulled down by the
   * transceiver. See USB 2.0 spec, section 7.1.7.3. */
  gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
  gpio_write_bit(disc_dev, disc_bit, 0);

  /* Initialize the USB peripheral. */
  usb_mass_init_usblib(USBLIB, ep_mass_int_in, ep_mass_int_out);
}

void usb_mass_disable(gpio_dev *disc_dev, uint8 disc_bit) {
  /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
   * spec, section 7.1.7.3). */
  nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
  gpio_write_bit(disc_dev, disc_bit, 1);
}

/*
 * User hooks
 */

static void (*read_hook)(unsigned, void*) = 0;
static void (*write_hook)(unsigned, void*) = 0;

void usb_mass_set_hooks(unsigned hook_flags, void (*hook)(unsigned, void*)) {
  if (hook_flags & USB_MASS_HOOK_READ) {
    read_hook = hook;
  }
  if (hook_flags & USB_MASS_HOOK_WRITE) {
    write_hook = hook;
  }
}