
#ifndef USBMASSSTORAGE_H
#define	USBMASSSTORAGE_H

#include <wirish/boards.h>

class USBMassStorageDriver {
public:
  void begin();
  void end();
};

extern USBMassStorageDriver USBMassStorage;

#endif	/* USBMASSSTORAGE_H */

