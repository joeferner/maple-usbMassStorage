
#include <wirish/wirish.h>
#include <usbMassStorage/usbMassStorage.h>
#include <sdfat/Sd2Card.h>
#include <sdfat/SdFat.h>

/**
 * This example requires the maple-sdfat library (http://code.google.com/p/maple-sdfat/).
 * 
 * When the example starts it creates two files /PROC/REBOOT and /PROC/BOOTLOAD
 * If you write anything to these files from the host computer it will reboot or
 * enter the bootloader respectively.
 * 
 *   Connections:
 *      D10             -> SDCard.CS
 *      D11 (SPI1 MOSI) -> SDCard.DI
 *      D12 (SPI1 MISO) -> SDCard.DO
 *      D13 (SPI1 SCK)  -> SDCard.CLK
 */
#define SDCARD_CS  D10
#define SDCARD_SPI 1

uint32_t MAL_massBlockCount[2];
uint32_t MAL_massBlockSize[2];
uint32_t rebootFileBlock;
uint32_t bootLoadFileBlock;

HardwareSPI spi1(SDCARD_SPI);
Sd2Card sdcard(spi1);
SdVolume sdVolume;

uint8_t initSdCardSize();
uint8_t initCard();
void reboot();
void enterBootloader();

void setup() {
  delay(1000); // not needed just easier to debug
  Serial1.begin(9600);
  Serial1.println("begin");
  spi1.begin(SPI_QUARTER_SPEED, MSBFIRST, 0);
  sdcard.init(SPI_QUARTER_SPEED, SDCARD_CS);
  sdVolume.init(&sdcard, 0);

  initSdCardSize();
  initCard();

  Serial1.println("USBMassStorage.begin");
  USBMassStorage.begin();

  Serial1.println("setup complete");
}

void loop() {
  USBMassStorage.loop();
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.

__attribute__((constructor)) void premain() {
  init();
}

int main(void) {
  setup();

  while (true) {
    loop();
  }

  return 0;
}

/**
 * Initializes the USBMassStorage driver with the size of the SDCard. 
 */
uint8_t initSdCardSize() {
  Serial1.println("initSdCardSize");
  uint32_t numberOfBlocks = sdcard.cardSize();
  if (numberOfBlocks == 0) {
    Serial1.println("could not get card size");
    return false;
  }
  MAL_massBlockCount[0] = numberOfBlocks;
  MAL_massBlockCount[1] = 0;
  MAL_massBlockSize[0] = 512;
  MAL_massBlockSize[1] = 0;
  Serial1.print("cardSize = ");
  Serial1.println(numberOfBlocks * 512);
  return true;
}

/**
 * Creates the /PROC/REBOOT and /PROC/BOOTLOAD files to allow
 * remote reboot and bootloader respectively.
 */
uint8_t initCard() {
  Serial1.println("initCard");
  SdFile root, procDir, rebootFile, bootLoadFile;

  // open the root directory
  if (!root.openRoot(&sdVolume)) {
    Serial1.println("failed to openRoot");
    return false;
  }

  // open the /PROC directory
  if (!procDir.open(root, "PROC", O_READ)) {
    if (!procDir.makeDir(root, "PROC")) {
      Serial1.println("failed to create proc");
      return false;
    }
  }

  // create /PROC/REBOOT file and write a '0' to it.
  if (!rebootFile.open(procDir, "REBOOT", O_WRITE | O_CREAT | O_TRUNC)) {
    Serial1.println("Could not create /PROC/REBOOT");
    return false;
  }
  rebootFile.write('0');
  rebootFile.close();
  rebootFileBlock = rebootFile.firstCluster() * sdVolume.blocksPerCluster();
  Serial1.print("/PROC/REBOOT: ");
  Serial1.println(rebootFileBlock);

  // create /PROC/BOOTLOAD file and write a '0' to it.
  if (!bootLoadFile.open(procDir, "BOOTLOAD", O_WRITE | O_CREAT | O_TRUNC)) {
    Serial1.println("Could not create /PROC/BOOTLOAD");
    return false;
  }
  bootLoadFile.write('0');
  bootLoadFile.close();
  bootLoadFileBlock = bootLoadFile.firstCluster() * sdVolume.blocksPerCluster();
  Serial1.print("/PROC/BOOTLOAD: ");
  Serial1.println(bootLoadFileBlock);

  procDir.close();
  root.close();
  return true;
}

/*
 * The following methods are used by the USBMassStorage driver to read and write to the SDCard.  
 */

extern "C" uint16_t usb_mass_mal_init(uint8_t lun) {
  return 0;
}

extern "C" uint16_t usb_mass_mal_get_status(uint8_t lun) {
  return sdcard.errorCode();
}

extern "C" uint16_t usb_mass_mal_write_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *writebuff, uint16_t transferLength) {
  uint32_t block = memoryOffset / 512;
  if (block == rebootFileBlock) {
    reboot();
    return USB_MASS_MAL_SUCCESS;
  }
  if (block == bootLoadFileBlock) {
    enterBootloader();
    return USB_MASS_MAL_SUCCESS;
  }

  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (sdcard.writeBlock(block, writebuff)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}

extern "C" uint16_t usb_mass_mal_read_memory(uint8_t lun, uint32_t memoryOffset, uint8_t *readbuff, uint16_t transferLength) {
  if (lun != 0) {
    return USB_MASS_MAL_FAIL;
  }
  if (sdcard.readBlock(memoryOffset / 512, readbuff)) {
    return USB_MASS_MAL_SUCCESS;
  }
  return USB_MASS_MAL_FAIL;
}

extern "C" void usb_mass_mal_format() {
}

/*
 * Helper methods to reboot and enter the bootloader 
 */

void reboot() {
  Serial1.println("reset");
  nvic_sys_reset();
  while (1);
}

#define RESET_DELAY 100000

static void wait_reset(void) {
  delay_us(RESET_DELAY);
  nvic_sys_reset();
}

#define STACK_TOP 0x20000800
#define EXC_RETURN 0xFFFFFFF9
#define DEFAULT_CPSR 0x61000000

void enterBootloader() {
  Serial1.println("bootloader");

  // Got the magic sequence -> reset, presumably into the bootloader.
  // Return address is wait_reset, but we must set the thumb bit.
  uintptr_t target = (uintptr_t) wait_reset | 0x1;
  asm volatile("mov r0, %[stack_top]      \n\t" // Reset stack
            "mov sp, r0                \n\t"
            "mov r0, #1                \n\t"
            "mov r1, %[target_addr]    \n\t"
            "mov r2, %[cpsr]           \n\t"
            "push {r2}                 \n\t" // Fake xPSR
            "push {r1}                 \n\t" // PC target addr
            "push {r0}                 \n\t" // Fake LR
            "push {r0}                 \n\t" // Fake R12
            "push {r0}                 \n\t" // Fake R3
            "push {r0}                 \n\t" // Fake R2
            "push {r0}                 \n\t" // Fake R1
            "push {r0}                 \n\t" // Fake R0
            "mov lr, %[exc_return]     \n\t"
            "bx lr"
            :
            : [stack_top] "r" (STACK_TOP),
            [target_addr] "r" (target),
            [exc_return] "r" (EXC_RETURN),
            [cpsr] "r" (DEFAULT_CPSR)
            : "r0", "r1", "r2");
}