
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

uint8_t* getMaxLun(uint16_t Length);

#define LUN_DATA_LENGTH            1

extern DEVICE_PROP Device_Property;
extern usb_descriptor_config usbMassConfigDescriptor;

uint32_t maxLun = 0;
uint32_t deviceState = DEVICE_STATE_UNCONNECTED;
uint8_t botState = BOT_STATE_IDLE;
BulkOnlyCBW CBW;

void usb_mass_init() {
  pInformation->Current_Configuration = 0;

  USB_BASE->CNTR = USB_CNTR_FRES;

  USBLIB->irq_mask = 0;
  USB_BASE->CNTR = USBLIB->irq_mask;
  USB_BASE->ISTR = 0;
  USBLIB->irq_mask = USB_CNTR_RESETM | USB_CNTR_SUSPM | USB_CNTR_WKUPM;
  USB_BASE->CNTR = USBLIB->irq_mask;

  USB_BASE->ISTR = 0;
  USBLIB->irq_mask = USB_ISR_MSK;
  USB_BASE->CNTR = USBLIB->irq_mask;

  nvic_irq_enable(NVIC_USB_LP_CAN_RX0);
  USBLIB->state = USB_UNCONNECTED;
}

void usb_mass_reset() {
  pInformation->Current_Configuration = 0;

  /* current feature is current bmAttributes */
  pInformation->Current_Feature = usbMassConfigDescriptor.Config_Header.bmAttributes;

  USB_BASE->BTABLE = BTABLE_ADDRESS;

  /* setup control endpoint 0 */
  usb_set_ep_type(USB_EP0, USB_EP_EP_TYPE_CONTROL);
  usb_set_ep_tx_stat(USB_EP0, USB_EP_STAT_TX_NAK);
  usb_set_ep_rx_addr(USB_EP0, ENDP0_RXADDR);
  usb_set_ep_rx_count(USB_EP0, Device_Property.MaxPacketSize);
  usb_set_ep_tx_addr(USB_EP0, ENDP0_TXADDR);
  usb_clear_status_out(USB_EP0);
  usb_set_ep_rx_stat(USB_EP0, USB_EP_STAT_RX_VALID);

  /* Initialize Endpoint 1 */
  usb_set_ep_type(USB_EP1, USB_EP_EP_TYPE_BULK);
  usb_set_ep_tx_addr(USB_EP1, ENDP1_TXADDR);
  usb_set_ep_tx_stat(USB_EP1, USB_EP_STAT_TX_NAK);
  usb_set_ep_rx_stat(USB_EP1, USB_EP_STAT_RX_DISABLED);

  /* Initialize Endpoint 2 */
  usb_set_ep_type(USB_EP2, USB_EP_EP_TYPE_BULK);
  usb_set_ep_rx_addr(USB_EP2, ENDP2_RXADDR);
  usb_set_ep_rx_count(USB_EP2, Device_Property.MaxPacketSize);
  usb_set_ep_rx_stat(USB_EP2, USB_EP_STAT_RX_VALID);
  usb_set_ep_tx_stat(USB_EP2, USB_EP_STAT_TX_DISABLED);

  usb_set_ep_rx_count(USB_EP0, Device_Property.MaxPacketSize);
  usb_set_ep_rx_stat(USB_EP0, USB_EP_STAT_RX_VALID);

  USBLIB->state = USB_ATTACHED;
  SetDeviceAddress(0);

  deviceState = DEVICE_STATE_ATTACHED;
  CBW.dSignature = BOT_CBW_SIGNATURE;
  botState = BOT_STATE_IDLE;
}

void usb_mass_set_configuration(void) {
  if (pInformation->Current_Configuration != 0) {
    USBLIB->state = USB_CONFIGURED;
    ClearDTOG_TX(USB_EP1);
    ClearDTOG_RX(USB_EP2);
    botState = BOT_STATE_IDLE;
  }
}

/********************** TODO BELOW **********************/

/*
 *  IN
 */
//static void usb_mass_in(void) {
//  switch (Bot_State) {
//    case BOT_CSW_Send:
//    case BOT_ERROR:
//      Bot_State = BOT_IDLE;
//      SetEPRxStatus(ENDP2, EP_RX_VALID); /* enable the Endpoint to receive the next cmd*/
//      break;
//    case BOT_DATA_IN:
//      switch (CBW.CB[0]) {
//        case SCSI_READ10:
//          SCSI_Read10_Cmd(CBW.bLUN, SCSI_LBA, SCSI_BlkLen);
//          break;
//      }
//      break;
//    case BOT_DATA_IN_LAST:
//      Set_CSW(CSW_CMD_PASSED, SEND_CSW_ENABLE);
//      SetEPRxStatus(ENDP2, EP_RX_VALID);
//      break;
//
//    default:
//      break;
//  }
//}

/*
 *  OUT
 */
//static void usb_mass_out(void) {
//  uint8_t CMD;
//  CMD = CBW.CB[0];
//
//  Data_Len = USB_SIL_Read(EP2_OUT, Bulk_Data_Buff);
//
//  switch (Bot_State) {
//    case BOT_IDLE:
//      CBW_Decode();
//      break;
//    case BOT_DATA_OUT:
//      if (CMD == SCSI_WRITE10) {
//        SCSI_Write10_Cmd(CBW.bLUN, SCSI_LBA, SCSI_BlkLen);
//        break;
//      }
//      Bot_Abort(DIR_OUT);
//      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
//      Set_CSW(CSW_PHASE_ERROR, SEND_CSW_DISABLE);
//      break;
//    default:
//      Bot_Abort(BOTH_DIR);
//      Set_Scsi_Sense_Data(CBW.bLUN, ILLEGAL_REQUEST, INVALID_FIELED_IN_COMMAND);
//      Set_CSW(CSW_PHASE_ERROR, SEND_CSW_DISABLE);
//      break;
//  }
//}
//
//static RESULT usbDataSetup(uint8 request) {
//  uint8_t * (*CopyRoutine)(uint16_t);
//
//  CopyRoutine = NULL;
//  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
//          && (request == REQUEST_GET_MAX_LUN) && (pInformation->USBwValue == 0)
//          && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x01)) {
//    CopyRoutine = getMaxLun;
//  } else {
//    return USB_UNSUPPORT;
//  }
//
//  if (CopyRoutine == NULL) {
//    return USB_UNSUPPORT;
//  }
//
//  pInformation->Ctrl_Info.CopyData = CopyRoutine;
//  pInformation->Ctrl_Info.Usb_wOffset = 0;
//  (*CopyRoutine)(0);
//
//  return USB_SUCCESS;
//}
//
//uint8_t* getMaxLun(uint16_t Length) {
//  if (Length == 0) {
//    pInformation->Ctrl_Info.Usb_wLength = LUN_DATA_LENGTH;
//    return 0;
//  } else {
//    return ((uint8_t*) (&maxLun));
//  }
//}
//
//static RESULT usbNoDataSetup(uint8 request) {
//  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
//          && (request == REQUEST_MASS_STORAGE_RESET) && (pInformation->USBwValue == 0)
//          && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x00)) {
//    /* Initialize Endpoint 1 */
//    ClearDTOG_TX(ENDP1);
//
//    /* Initialize Endpoint 2 */
//    ClearDTOG_RX(ENDP2);
//
//    /*initialize the CBW signature to enable the clear feature*/
//    CBW.dSignature = BOT_CBW_SIGNATURE;
//    Bot_State = BOT_IDLE;
//
//    return USB_SUCCESS;
//  }
//  return USB_UNSUPPORT;
//}
//
//static RESULT usbGetInterfaceSetting(uint8 interface, uint8 alt_setting) {
//  if (alt_setting > 0) {
//    return USB_UNSUPPORT;
//  } else if (interface > 0) {
//    return USB_UNSUPPORT;
//  }
//
//  return USB_SUCCESS;
//}
//
//static uint8* usbGetDeviceDescriptor(uint16 length) {
//  return Standard_GetDescriptorData(length, &Device_Descriptor);
//}
//
//static uint8* usbGetConfigDescriptor(uint16 length) {
//  return Standard_GetDescriptorData(length, &Config_Descriptor);
//}
//
//static uint8* usbGetStringDescriptor(uint16 length) {
//  uint8 wValue0 = pInformation->USBwValue0;
//
//  if (wValue0 > N_STRING_DESCRIPTORS) {
//    return NULL;
//  }
//  return Standard_GetDescriptorData(length, &String_Descriptor[wValue0]);
//}
//
//static void usbSetDeviceAddress(void) {
//  USBLIB->state = USB_ADDRESSED;
//}
//
//void usb_mass_enable(gpio_dev *disc_dev, uint8 disc_bit) {
//  /* Present ourselves to the host. Writing 0 to "disc" pin must
//   * pull USB_DP pin up while leaving USB_DM pulled down by the
//   * transceiver. See USB 2.0 spec, section 7.1.7.3. */
//  gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
//  gpio_write_bit(disc_dev, disc_bit, 0);
//
//  /* Initialize the USB peripheral. */
//  usb_init_usblib(USBLIB, ep_mass_int_in, ep_mass_int_out);
//}
//
//void usb_mass_disable(gpio_dev *disc_dev, uint8 disc_bit) {
//  /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
//   * spec, section 7.1.7.3). */
//  nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
//  gpio_write_bit(disc_dev, disc_bit, 1);
//}
//
///*
// * User hooks
// */
//static void (*read_hook)(unsigned, void*) = 0;
//static void (*write_hook)(unsigned, void*) = 0;
//
//void usb_mass_set_hooks(unsigned hook_flags, void (*hook)(unsigned, void*)) {
//  if (hook_flags & USB_MASS_HOOK_READ) {
//    read_hook = hook;
//  }
//  if (hook_flags & USB_MASS_HOOK_WRITE) {
//    write_hook = hook;
//  }
//}