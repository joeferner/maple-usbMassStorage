
#include "usbMassStorage.h"
#include "usb_mass.h"

static void readHook(unsigned hook, void* readInfo);
static void writeHook(unsigned hook, void* writeInfo);

void USBMassStorageDriver::begin() {
  usb_mass_enable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
  usb_mass_set_hooks(USB_MASS_HOOK_READ, readHook);
  usb_mass_set_hooks(USB_MASS_HOOK_WRITE, writeHook);
}

void USBMassStorageDriver::end() {
  usb_mass_disable(BOARD_USB_DISC_DEV, BOARD_USB_DISC_BIT);
  usb_mass_remove_hooks(USB_MASS_HOOK_READ | USB_MASS_HOOK_WRITE);
}

static void readHook(unsigned hook, void* readInfo) {
}

static void writeHook(unsigned hook, void* writeInfo) {
}

USBMassStorageDriver USBMassStorage;