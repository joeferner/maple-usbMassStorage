
#include "usb_mass.h"
#include "usb_scsi.h"

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

static void usb_mass_bot_cbw_decode();

static uint8_t* usb_mass_get_max_lun(uint16_t Length);
static void usb_mass_in(void);
static void usb_mass_out(void);
uint32_t usb_mass_sil_write(uint8_t bEpAddr, uint8_t* pBufferPointer, uint32_t wBufferSize);
uint32_t usb_mass_sil_read(uint8_t bEpAddr, uint8_t* pBufferPointer);

#define LUN_DATA_LENGTH            1

extern DEVICE_PROP Device_Property;
extern ONE_DESCRIPTOR Device_Descriptor;
extern ONE_DESCRIPTOR Config_Descriptor;
extern ONE_DESCRIPTOR String_Descriptor[];
extern usb_descriptor_config usbMassConfigDescriptor;

uint32_t maxLun = 0;
uint32_t deviceState = DEVICE_STATE_UNCONNECTED;
uint8_t botState = BOT_STATE_IDLE;
BulkOnlyCBW CBW;
BulkOnlyCSW CSW;
uint8_t bulkDataBuff[MAX_BULK_PACKET_SIZE];
uint16_t dataLength;

/*
 * Endpoint callbacks
 */
static void (*ep_mass_int_in[7])(void) = {
  usb_mass_in,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process
};

static void (*ep_mass_int_out[7])(void) = {
  NOP_Process,
  usb_mass_out,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process,
  NOP_Process
};

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
  usb_set_ep_rx_count(USB_EP2, usbMassConfigDescriptor.DataOutEndpoint.wMaxPacketSize);
  usb_set_ep_rx_stat(USB_EP2, USB_EP_STAT_RX_VALID);
  usb_set_ep_tx_stat(USB_EP2, USB_EP_STAT_TX_DISABLED);

  usb_set_ep_rx_count(USB_EP0, Device_Property.MaxPacketSize);
  usb_set_ep_rx_stat(USB_EP0, USB_EP_STAT_RX_VALID);

  SetDeviceAddress(0);

  deviceState = DEVICE_STATE_ATTACHED;
  CBW.dSignature = BOT_CBW_SIGNATURE;
  botState = BOT_STATE_IDLE;
}

void usb_mass_set_configuration(void) {
  if (pInformation->Current_Configuration != 0) {
    deviceState = USB_CONFIGURED;
    ClearDTOG_TX(USB_EP1);
    ClearDTOG_RX(USB_EP2);
    botState = BOT_STATE_IDLE;
  }
}

void usb_mass_clear_feature(void) {
  /* when the host send a CBW with invalid signature or invalid length the two
   Endpoints (IN & OUT) shall stall until receiving a Mass Storage Reset     */
  if (CBW.dSignature != BOT_CBW_SIGNATURE) {
    usb_mass_bot_abort(BOT_DIR_BOTH);
  }
}

void usb_mass_set_device_address(void) {
  deviceState = USB_ADDRESSED;
}

void usb_mass_status_in(void) {
  return;
}

void usb_mass_status_out(void) {
  return;
}

RESULT usb_mass_data_setup(uint8 request) {
  uint8_t * (*copy_routine)(uint16_t);

  copy_routine = NULL;
  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
          && (request == REQUEST_GET_MAX_LUN) && (pInformation->USBwValue == 0)
          && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x01)) {
    copy_routine = usb_mass_get_max_lun;
  } else {
    return USB_UNSUPPORT;
  }

  if (copy_routine == NULL) {
    return USB_UNSUPPORT;
  }

  pInformation->Ctrl_Info.CopyData = copy_routine;
  pInformation->Ctrl_Info.Usb_wOffset = 0;
  (*copy_routine)(0);

  return USB_SUCCESS;
}

static uint8_t* usb_mass_get_max_lun(uint16_t length) {
  if (length == 0) {
    pInformation->Ctrl_Info.Usb_wLength = LUN_DATA_LENGTH;
    return 0;
  } else {
    return ((uint8_t*) (&maxLun));
  }
}

RESULT usb_mass_no_data_setup(uint8 request) {
  if ((Type_Recipient == (CLASS_REQUEST | INTERFACE_RECIPIENT))
          && (request == REQUEST_MASS_STORAGE_RESET) && (pInformation->USBwValue == 0)
          && (pInformation->USBwIndex == 0) && (pInformation->USBwLength == 0x00)) {

    /* Initialize Endpoint 1 */
    ClearDTOG_TX(USB_EP1);

    /* Initialize Endpoint 2 */
    ClearDTOG_RX(USB_EP2);

    /*initialize the CBW signature to enable the clear feature*/
    CBW.dSignature = BOT_CBW_SIGNATURE;
    botState = BOT_STATE_IDLE;

    return USB_SUCCESS;
  }
  return USB_UNSUPPORT;
}

RESULT usb_mass_get_interface_setting(uint8 interface, uint8 altSetting) {
  if (altSetting > 0) {
    return USB_UNSUPPORT; /* in this application we don't have AlternateSetting*/
  } else if (interface > 0) {
    return USB_UNSUPPORT; /*in this application we have only 1 interfaces*/
  }
  return USB_SUCCESS;
}

uint8* usb_mass_get_device_descriptor(uint16 length) {
  return Standard_GetDescriptorData(length, &Device_Descriptor);
}

uint8* usb_mass_get_config_descriptor(uint16 length) {
  return Standard_GetDescriptorData(length, &Config_Descriptor);
}

uint8* usb_mass_get_string_descriptor(uint16 length) {
  uint8_t idx = pInformation->USBwValue0;

  if (idx < N_STRING_DESCRIPTORS) {
    return Standard_GetDescriptorData(length, &String_Descriptor[idx]);
  } else {
    return NULL;
  }
}

void usb_mass_enable(gpio_dev *disc_dev, uint8 disc_bit) {
  usb_mass_mal_init(0);

  /* Present ourselves to the host. Writing 0 to "disc" pin must
   * pull USB_DP pin up while leaving USB_DM pulled down by the
   * transceiver. See USB 2.0 spec, section 7.1.7.3. */
  gpio_set_mode(disc_dev, disc_bit, GPIO_OUTPUT_PP);
  gpio_write_bit(disc_dev, disc_bit, 0);

  /* Initialize the USB peripheral. */
  usb_init_usblib(USBLIB, ep_mass_int_in, ep_mass_int_out);
}

void usb_mass_disable(gpio_dev *disc_dev, uint8 disc_bit) {
  /* Turn off the interrupt and signal disconnect (see e.g. USB 2.0
   * spec, section 7.1.7.3). */
  nvic_irq_disable(NVIC_USB_LP_CAN_RX0);
  gpio_write_bit(disc_dev, disc_bit, 1);
}

/*
 *  IN
 */
static void usb_mass_in(void) {
  switch (botState) {
    case BOT_STATE_CSW_Send:
    case BOT_STATE_ERROR:
      botState = BOT_STATE_IDLE;
      SetEPRxStatus(USB_EP2, USB_EP_ST_RX_VAL); /* enable the Endpoint to receive the next cmd*/
      break;
    case BOT_STATE_DATA_IN:
      switch (CBW.CB[0]) {
        case SCSI_READ10:
          scsi_read10_cmd(CBW.bLUN, SCSI_lba, SCSI_blkLen);
          break;
      }
      break;
    case BOT_STATE_DATA_IN_LAST:
      usb_mass_bot_set_csw(BOT_CSW_CMD_PASSED, BOT_SEND_CSW_ENABLE);
      SetEPRxStatus(USB_EP2, USB_EP_ST_RX_VAL);
      break;

    default:
      break;
  }
}

/*
 *  OUT
 */
static void usb_mass_out(void) {
  uint8_t CMD;
  CMD = CBW.CB[0];

  dataLength = usb_mass_sil_read(USB_EP2, bulkDataBuff);

  switch (botState) {
    case BOT_STATE_IDLE:
      usb_mass_bot_cbw_decode();
      break;
    case BOT_STATE_DATA_OUT:
      if (CMD == SCSI_WRITE10) {
        scsi_write10_cmd(CBW.bLUN, SCSI_lba, SCSI_blkLen);
        break;
      }
      usb_mass_bot_abort(BOT_DIR_OUT);
      scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_INVALID_FIELED_IN_COMMAND);
      usb_mass_bot_set_csw(BOT_CSW_PHASE_ERROR, BOT_SEND_CSW_DISABLE);
      break;
    default:
      usb_mass_bot_abort(BOT_DIR_BOTH);
      scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_INVALID_FIELED_IN_COMMAND);
      usb_mass_bot_set_csw(BOT_CSW_PHASE_ERROR, BOT_SEND_CSW_DISABLE);
      break;
  }
}

static void usb_mass_bot_cbw_decode() {
  uint32_t counter;

  for (counter = 0; counter < dataLength; counter++) {
    *((uint8_t *) & CBW + counter) = bulkDataBuff[counter];
  }
  CSW.dTag = CBW.dTag;
  CSW.dDataResidue = CBW.dDataLength;
  if (dataLength != BOT_CBW_PACKET_LENGTH) {
    usb_mass_bot_abort(BOT_DIR_BOTH);
    /* reset the CBW.dSignature to disable the clear feature until receiving a Mass storage reset*/
    CBW.dSignature = 0;
    scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_PARAMETER_LIST_LENGTH_ERROR);
    usb_mass_bot_set_csw(BOT_CSW_CMD_FAILED, BOT_SEND_CSW_DISABLE);
    return;
  }

  if ((CBW.CB[0] == SCSI_READ10) || (CBW.CB[0] == SCSI_WRITE10)) {
    /* Calculate Logical Block Address */
    SCSI_lba = (CBW.CB[2] << 24) | (CBW.CB[3] << 16) | (CBW.CB[4] << 8) | CBW.CB[5];
    /* Calculate the Number of Blocks to transfer */
    SCSI_blkLen = (CBW.CB[7] << 8) | CBW.CB[8];
  }

  if (CBW.dSignature == BOT_CBW_SIGNATURE) {
    /* Valid CBW */
    if ((CBW.bLUN > maxLun) || (CBW.bCBLength < 1) || (CBW.bCBLength > 16)) {
      usb_mass_bot_abort(BOT_DIR_BOTH);
      scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_INVALID_FIELED_IN_COMMAND);
      usb_mass_bot_set_csw(BOT_CSW_CMD_FAILED, BOT_SEND_CSW_DISABLE);
    } else {
      switch (CBW.CB[0]) {
        case SCSI_REQUEST_SENSE:
          scsi_request_sense_cmd(CBW.bLUN);
          break;
        case SCSI_INQUIRY:
          scsi_inquiry_cmd(CBW.bLUN);
          break;
        case SCSI_START_STOP_UNIT:
          scsi_start_stop_unit_cmd(CBW.bLUN);
          break;
        case SCSI_ALLOW_MEDIUM_REMOVAL:
          scsi_start_stop_unit_cmd(CBW.bLUN);
          break;
        case SCSI_MODE_SENSE6:
          scsi_mode_sense6_cmd(CBW.bLUN);
          break;
        case SCSI_MODE_SENSE10:
          scsi_mode_sense10_cmd(CBW.bLUN);
          break;
        case SCSI_READ_FORMAT_CAPACITIES:
          scsi_read_format_capacity_cmd(CBW.bLUN);
          break;
        case SCSI_READ_CAPACITY10:
          scsi_read_capacity10_cmd(CBW.bLUN);
          break;
        case SCSI_TEST_UNIT_READY:
          scsi_test_unit_ready_cmd(CBW.bLUN);
          break;
        case SCSI_READ10:
          scsi_read10_cmd(CBW.bLUN, SCSI_lba, SCSI_blkLen);
          break;
        case SCSI_WRITE10:
          scsi_write10_cmd(CBW.bLUN, SCSI_lba, SCSI_blkLen);
          break;
        case SCSI_VERIFY10:
          scsi_verify10_cmd(CBW.bLUN);
          break;
        case SCSI_FORMAT_UNIT:
          scsi_format_cmd(CBW.bLUN);
          break;

        case SCSI_MODE_SELECT10:
        case SCSI_MODE_SELECT6:
        case SCSI_SEND_DIAGNOSTIC:
        case SCSI_READ6:
        case SCSI_READ12:
        case SCSI_READ16:
        case SCSI_READ_CAPACITY16:
        case SCSI_WRITE6:
        case SCSI_WRITE12:
        case SCSI_VERIFY12:
        case SCSI_VERIFY16:
        case SCSI_WRITE16:
          scsi_invalid_cmd(CBW.bLUN);
          break;

        default:
        {
          usb_mass_bot_abort(BOT_DIR_BOTH);
          scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_INVALID_COMMAND);
          usb_mass_bot_set_csw(BOT_CSW_CMD_FAILED, BOT_SEND_CSW_DISABLE);
        }
      }
    }
  } else {
    /* Invalid CBW */
    usb_mass_bot_abort(BOT_DIR_BOTH);
    scsi_set_sense_data(CBW.bLUN, SCSI_ILLEGAL_REQUEST, SCSI_INVALID_COMMAND);
    usb_mass_bot_set_csw(BOT_CSW_CMD_FAILED, BOT_SEND_CSW_DISABLE);
  }
}

void usb_mass_bot_abort(uint8_t direction) {
  switch (direction) {
    case BOT_DIR_IN:
      SetEPTxStatus(USB_EP1, USB_EP_ST_TX_STL);
      break;
    case BOT_DIR_OUT:
      SetEPRxStatus(USB_EP2, USB_EP_ST_RX_STL);
      break;
    case BOT_DIR_BOTH:
      SetEPTxStatus(USB_EP1, USB_EP_ST_TX_STL);
      SetEPRxStatus(USB_EP2, USB_EP_ST_RX_STL);
      break;
    default:
      break;
  }
}

void usb_mass_transfer_data_request(uint8_t* dataPointer, uint16_t dataLen) {
  usb_mass_sil_write(USB_EP1_IN, dataPointer, dataLen);

  SetEPTxStatus(USB_EP1, USB_EP_ST_TX_VAL);
  botState = BOT_STATE_DATA_IN_LAST;
  CSW.dDataResidue -= dataLen;
  CSW.bStatus = BOT_CSW_CMD_PASSED;
}

void usb_mass_bot_set_csw(uint8_t status, uint8_t sendPermission) {
  CSW.dSignature = BOT_CSW_SIGNATURE;
  CSW.bStatus = status;

  usb_mass_sil_write(USB_EP1_IN, ((uint8_t *) & CSW), BOT_CSW_DATA_LENGTH);

  botState = BOT_STATE_ERROR;
  if (sendPermission) {
    botState = BOT_STATE_CSW_Send;
    SetEPTxStatus(USB_EP1, USB_EP_ST_TX_VAL);
  }
}

uint32_t usb_mass_sil_write(uint8_t bEpAddr, uint8_t* pBufferPointer, uint32_t wBufferSize) {
  /* Use the memory interface function to write to the selected endpoint */
  UserToPMABufferCopy(pBufferPointer, GetEPTxAddr(bEpAddr & 0x7F), wBufferSize);

  /* Update the data length in the control register */
  SetEPTxCount((bEpAddr & 0x7F), wBufferSize);

  return 0;
}

uint32_t usb_mass_sil_read(uint8_t bEpAddr, uint8_t* pBufferPointer) {
  uint32_t dataLength = 0;

  /* Get the number of received data on the selected Endpoint */
  dataLength = GetEPRxCount(bEpAddr & 0x7F);

  /* Use the memory interface function to write to the selected endpoint */
  PMAToUserBufferCopy(pBufferPointer, GetEPRxAddr(bEpAddr & 0x7F), dataLength);

  /* Return the number of received data */
  return dataLength;
}
