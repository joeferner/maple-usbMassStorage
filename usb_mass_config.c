
#include "usb_mass.h"

#include <libmaple/usb.h>
#include <libmaple/nvic.h>
#include <libmaple/delay.h>

/* Private headers */
#include "usb_lib_globals.h"
#include "usb_reg_map.h"

/* usb_lib headers */
#include "usb_type.h"
#include "usb_core.h"
#include "usb_def.h"

extern void usb_mass_init();
extern void usb_mass_reset();
extern RESULT usb_mass_data_setup(uint8 request);
extern RESULT usb_mass_no_data_setup(uint8 request);
extern RESULT usb_mass_get_interface_setting(uint8 interface, uint8 altSetting);
extern uint8* usb_mass_get_device_descriptor(uint16 length);
extern uint8* usb_mass_get_config_descriptor(uint16 length);
extern uint8* usb_mass_get_string_descriptor(uint16 length);
extern void usb_mass_set_configuration(void);
extern void usb_mass_set_device_address(void);
extern void usb_mass_status_in(void);
extern void usb_mass_status_out(void);
extern void usb_mass_clear_feature(void);

/* lsusb for a flash drive
Bus 003 Device 067: ID 0781:556c SanDisk Corp. 
Device Descriptor:
  bLength                18
  bDescriptorType         1
  bcdUSB               2.00
  bDeviceClass            0 (Defined at Interface level)
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        64
  idVendor           0x0781 SanDisk Corp.
  idProduct          0x556c 
  bcdDevice            1.03
  iManufacturer           1 SanDisk
  iProduct                2 Ultra 
  iSerial                 3 20054254420320C0F124
  bNumConfigurations      1
  Configuration Descriptor:
    bLength                 9
    bDescriptorType         2
    wTotalLength           32
    bNumInterfaces          1
    bConfigurationValue     1
    iConfiguration          0 
    bmAttributes         0x80
      (Bus Powered)
    MaxPower              200mA
    Interface Descriptor:
      bLength                 9
      bDescriptorType         4
      bInterfaceNumber        0
      bAlternateSetting       0
      bNumEndpoints           2
      bInterfaceClass         8 Mass Storage
      bInterfaceSubClass      6 SCSI
      bInterfaceProtocol     80 Bulk-Only
      iInterface              0 
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x81  EP 1 IN
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0200  1x 512 bytes
        bInterval               0
      Endpoint Descriptor:
        bLength                 7
        bDescriptorType         5
        bEndpointAddress     0x02  EP 2 OUT
        bmAttributes            2
          Transfer Type            Bulk
          Synch Type               None
          Usage Type               Data
        wMaxPacketSize     0x0200  1x 512 bytes
        bInterval               1
Device Qualifier (for other device speed):
  bLength                10
  bDescriptorType         6
  bcdUSB               2.00
  bDeviceClass            0 (Defined at Interface level)
  bDeviceSubClass         0 
  bDeviceProtocol         0 
  bMaxPacketSize0        64
  bNumConfigurations      1
Device Status:     0x0000
  (Bus Powered)
 */

#define NUM_ENDPTS           3
DEVICE Device_Table = {
  .Total_Endpoint = NUM_ENDPTS,
  .Total_Configuration = 1
};

#define MAX_PACKET_SIZE            0x40  /* 64B, maximum for USB FS Devices */
DEVICE_PROP Device_Property = {
  .Init = usb_mass_init,
  .Reset = usb_mass_reset,
  .Process_Status_IN = usb_mass_status_in,
  .Process_Status_OUT = usb_mass_status_out,
  .Class_Data_Setup = usb_mass_data_setup,
  .Class_NoData_Setup = usb_mass_no_data_setup,
  .Class_Get_Interface_Setting = usb_mass_get_interface_setting,
  .GetDeviceDescriptor = usb_mass_get_device_descriptor,
  .GetConfigDescriptor = usb_mass_get_config_descriptor,
  .GetStringDescriptor = usb_mass_get_string_descriptor,
  .RxEP_buffer = NULL,
  .MaxPacketSize = MAX_PACKET_SIZE
};

USER_STANDARD_REQUESTS User_Standard_Requests = {
  .User_GetConfiguration = NOP_Process,
  .User_SetConfiguration = usb_mass_set_configuration,
  .User_GetInterface = NOP_Process,
  .User_SetInterface = NOP_Process,
  .User_GetStatus = NOP_Process,
  .User_ClearFeature = usb_mass_clear_feature,
  .User_SetEndPointFeature = NOP_Process,
  .User_SetDeviceFeature = NOP_Process,
  .User_SetDeviceAddress = usb_mass_set_device_address
};

#define USB_VID              0x0781 /* TODO change this */
#define USB_PID              0x556c
static const usb_descriptor_device usbMassDeviceDescriptor = {
  .bLength = sizeof (usb_descriptor_device),
  .bDescriptorType = USB_DESCRIPTOR_TYPE_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0x00,
  .bMaxPacketSize0 = 0x40,
  .idVendor = USB_VID,
  .idProduct = USB_PID,
  .bcdDevice = 0x0200,
  .iManufacturer = 0x01,
  .iProduct = 0x02,
  .iSerialNumber = 0x00,
  .bNumConfigurations = 0x01,
};

#define CDC_FUNCTIONAL_DESCRIPTOR_SIZE(DataSize) (3 + DataSize)
#define CDC_FUNCTIONAL_DESCRIPTOR(DataSize)     \
  struct {                                      \
      uint8 bLength;                            \
      uint8 bDescriptorType;                    \
      uint8 SubType;                            \
      uint8 Data[DataSize];                     \
  } __packed

#define MAX_POWER (100 >> 1)
const usb_descriptor_config usbMassConfigDescriptor = {
  .Config_Header =
  {
    .bLength = sizeof (usb_descriptor_config_header),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_CONFIGURATION,
    .wTotalLength = sizeof (usb_descriptor_config),
    .bNumInterfaces = 0x01,
    .bConfigurationValue = 0x01,
    .iConfiguration = 0x00,
    .bmAttributes = (USB_CONFIG_ATTR_BUSPOWERED | USB_CONFIG_ATTR_SELF_POWERED),
    .bMaxPower = MAX_POWER,
  },

  .MASS_Interface =
  {
    .bLength = sizeof (usb_descriptor_interface),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_INTERFACE,
    .bInterfaceNumber = 0x00,
    .bAlternateSetting = 0x00,
    .bNumEndpoints = 0x02,
    .bInterfaceClass = 8, // mass storage
    .bInterfaceSubClass = 6, // SCSI
    .bInterfaceProtocol = 80, // Bulk-Only
    .iInterface = 0x00,
  },

  .DataInEndpoint =
  {
    .bLength = sizeof (usb_descriptor_endpoint),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_IN | USB_EP1),
    .bmAttributes = USB_EP_TYPE_BULK,
    .wMaxPacketSize = 0x200, // 1x 512 bytes
    .bInterval = 0x00,
  },

  .DataOutEndpoint =
  {
    .bLength = sizeof (usb_descriptor_endpoint),
    .bDescriptorType = USB_DESCRIPTOR_TYPE_ENDPOINT,
    .bEndpointAddress = (USB_DESCRIPTOR_ENDPOINT_OUT | USB_EP2),
    .bmAttributes = USB_EP_TYPE_BULK,
    .wMaxPacketSize = 0x200, // 1x 512 bytes
    .bInterval = 0x01,
  }
};

/*
  String Descriptors:

  we may choose to specify any or none of the following string
  identifiers:

  iManufacturer:    JoeFerner
  iProduct:         StmWifi
  iSerialNumber:    NONE
  iConfiguration:   NONE
  iInterface(CCI):  NONE
  iInterface(DCI):  NONE

 */

/* Unicode language identifier: 0x0409 is US English */
static const usb_descriptor_string usbMassStringLangID = {
  .bLength = USB_DESCRIPTOR_STRING_LEN(1),
  .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
  .bString =
  {0x09, 0x04},
};

static const usb_descriptor_string usbMassStringManufacturer = {
  .bLength = USB_DESCRIPTOR_STRING_LEN(9),
  .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
  .bString =
  {'J', 0, 'o', 0, 'e', 0, 'F', 0, 'e', 0, 'r', 0, 'n', 0, 'e', 0, 'r', 0},
};

static const usb_descriptor_string usbMassStringProduct = {
  .bLength = USB_DESCRIPTOR_STRING_LEN(7),
  .bDescriptorType = USB_DESCRIPTOR_TYPE_STRING,
  .bString =
  {'S', 0, 't', 0, 'm', 0, 'W', 0, 'i', 0, 'f', 0, 'i', 0},
};

ONE_DESCRIPTOR Device_Descriptor = {
  (uint8*) & usbMassDeviceDescriptor,
  sizeof (usb_descriptor_device)
};

ONE_DESCRIPTOR Config_Descriptor = {
  (uint8*) & usbMassConfigDescriptor,
  sizeof (usb_descriptor_config)
};

#define N_STRING_DESCRIPTORS 3
ONE_DESCRIPTOR String_Descriptor[N_STRING_DESCRIPTORS] = {
  {(uint8*) & usbMassStringLangID, USB_DESCRIPTOR_STRING_LEN(1)},
  {(uint8*) & usbMassStringManufacturer, USB_DESCRIPTOR_STRING_LEN(9)},
  {(uint8*) & usbMassStringProduct, USB_DESCRIPTOR_STRING_LEN(7)}
};

