
#include "usbMassStorage.h"
#include "usb_mass.h"

void USBMassStorageDriver::begin() {
  usb_mass_enable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
}

void USBMassStorageDriver::end() {
  usb_mass_disable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
}

USBMassStorageDriver USBMassStorage;