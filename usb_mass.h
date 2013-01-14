
#ifndef _LIBMAPLE_USB_MASS_H_
#define _LIBMAPLE_USB_MASS_H_

#include <libmaple/libmaple_types.h>
#include <libmaple/gpio.h>
#include <libmaple/usb.h>

#ifdef __cplusplus
extern "C" {
#endif

#define USB_MASS_HOOK_READ 0x1
#define USB_MASS_HOOK_WRITE 0x2

  void usb_mass_enable(gpio_dev *disc_dev, uint8 disc_bit);
  void usb_mass_disable(gpio_dev *disc_dev, uint8 disc_bit);
  void usb_mass_set_hooks(unsigned hook_flags, void (*hook)(unsigned, void*));

  static __always_inline void usb_mass_remove_hooks(unsigned hook_flags) {
    usb_mass_set_hooks(hook_flags, 0);
  }

#ifdef __cplusplus
}
#endif

#endif
