
#ifndef USBMASSSTORAGE_H
#define	USBMASSSTORAGE_H

#include <wirish/boards.h>

#define USB_MASS_MAL_FAIL    1
#define USB_MASS_MAL_SUCCESS 0

class USBMassStorageDriver {
public:
  void begin();
  void end();
  void loop();
};

extern USBMassStorageDriver USBMassStorage;

#endif	/* USBMASSSTORAGE_H */

