# WARNING: I am no longer maintaining this software. The code is here for historical reasons and hopefully it will help someone.

# maple-usbMassStorage

Driver to allow [LeafLabs maple](http://leaflabs.com/devices/maple/) act as a USB mass storage device.

# API

* [USBMassStorage.begin()](#USBMassStorage_begin)
* [USBMassStorage.loop()](#USBMassStorage_loop)
* [MAL_massBlockCount[2] : uint32_t](#massBlockCount)
* [MAL_massBlockSize[2] : uint32_t](#massBlockSize)
* [usb_mass_mal_init(uint8_t lun) : uint16_t](#usb_mass_mal_init)
* [usb_mass_mal_get_status(uint8_t lun) : uint16_t](#usb_mass_mal_get_status)
* [usb_mass_mal_write_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *writebuff, uint16_t transferLength) : uint16_t](#usb_mass_mal_write_memory)
* [usb_mass_mal_read_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *readbuff, uint16_t transferLength) : uint16_t](#usb_mass_mal_read_memory)
* [usb_mass_mal_format() : void](#usb_mass_mal_format)

# API Details

<a name="USBMassStorage_begin" />
**USBMassStorage.begin()**

Starts the USB mass storage driver. After calling this method the computer should now detect the Maple as a USB mass
storage device.

<a name="USBMassStorage_loop" />
**USBMassStorage.loop()**

Allows the USB mass storage driver to process incoming requests. Call this method in your main loop.

Even though the STM32 USB causes interrupts which could be used to process this data. Typically interrupts should be
kept short. The other side effect is that certain libraries do not work inside the interrupt handlers (such as sdfat).
Using the loop function solves these problems by moving most of the logic into non-interrupt code.

__Example__

```
void loop() {
  USBMassStorage.loop();
}
```

<a name="massBlockCount" />
**MAL_massBlockCount[2] : uint32_t**

The block count of the two SCSI LUNs. This should be initialized before [USBMassStorage.begin()](#USBMassStorage_begin).

__Example__

```
uint32_t MAL_massBlockCount[2];
uint32_t MAL_massBlockSize[2];

void setup() {
  MAL_massBlockCount[0] = 100;
  MAL_massBlockCount[1] = 0;
  MAL_massBlockSize[0] = 512;
  MAL_massBlockSize[1] = 0;

  USBMassStorage.begin();
}
```

<a name="massBlockSize" />
**MAL_massBlockSize[2] : uint32_t**

The block size of the two SCSI LUNs. This should be initialized before [USBMassStorage.begin()](#USBMassStorage_begin).
For SDCards you should initialize it to 512.

<a name="usb_mass_mal_init" />
**usb_mass_mal_init(uint8_t lun) : uint16_t**

Called by the USB mass storage driver after the host has requested the device.

__Arguments__

 * lun - the LUN number being initialized. This can be 0 or 1. Typically just 0.

__Return__

0 for success. 1 for failure.

<a name="usb_mass_mal_get_status" />
**usb_mass_mal_get_status(uint8_t lun) : uint16_t**

Gets the status of the given LUN.

__Arguments__

 * lun - the lun number to get status for. This can be 0 or 1. Typically just 0.

__Return__

0 for success. 1 for failure.

<a name="usb_mass_mal_write_memory" />
**usb_mass_mal_write_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *writebuff, uint16_t transferLength) : uint16_t**

Called when data should be written to storage.

__Arguments__

 * lun - the lun number to write to. This can be 0 or 1. Typically just 0.
 * memoryOffset - Address in bytes to write to.
 * writebuff - A buffer containing the data to write. This buffer should not be relied on after this function returns as
               it may be reused.
 * transferLength - The number of bytes to write.

__Return__

USB_MASS_MAL_SUCCESS for success. USB_MASS_MAL_FAIL for failure.

__Example__

```
extern "C" uint16_t usb_mass_mal_write_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *writebuff, uint16_t transferLength) {
  uint32_t block = memoryOffset / 512;
  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (sdcard.writeBlock(block, writebuff)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}
```

<a name="usb_mass_mal_read_memory" />
**usb_mass_mal_read_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *readbuff, uint16_t transferLength) : uint16_t**

Called when data should be read from storage.

__Arguments__

 * lun - the lun number to write to. This can be 0 or 1. Typically just 0.
 * memoryOffset - Address in bytes to read from.
 * readbuff - A buffer in which the data should be read into.
 * transferLength - The number of bytes to read.

__Return__

USB_MASS_MAL_SUCCESS for success. USB_MASS_MAL_FAIL for failure.

__Example__

```
extern "C" uint16_t usb_mass_mal_read_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *readbuff, uint16_t transferLength) {
  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (sdcard.readBlock(memoryOffset / 512, readbuff)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}
```

<a name="usb_mass_mal_format" />
**usb_mass_mal_format() : void**

Called to format the device.
