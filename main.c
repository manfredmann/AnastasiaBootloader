/*
 * Based on libopencm32 USB HID example
 * Roman G. Serov (c) 2016
 */

#include "main.h"

static const unsigned char bootloader_name[32] = "Anastasia USB Bootloader";
static const uint8_t bootloader_v_major = 0;
static const uint8_t bootloader_v_minor = 1;
static const uint8_t bootloader_v_fixn = 0;

const struct usb_device_descriptor dev = {
  .bLength = USB_DT_DEVICE_SIZE,
  .bDescriptorType = USB_DT_DEVICE,
  .bcdUSB = 0x0200,
  .bDeviceClass = 0,
  .bDeviceSubClass = 0,
  .bDeviceProtocol = 0,
  .bMaxPacketSize0 = 64,
  .idVendor = 0x1209,
  .idProduct = 0xCC86,
  .bcdDevice = 0x0100,
  .iManufacturer = 1,
  .iProduct = 2,
  .iSerialNumber = 3,
  .bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
  0x06, 0x00, 0xff,
  HID_USAGE   (0x01),
  HID_COLLECTION    (HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(1),
    HID_LOGICAL_MINIMUM     (0),
    HID_LOGICAL_MAXIMUM     (255),
    HID_REPORT_SIZE   (8),
    HID_REPORT_COUNT  (9),
    HID_USAGE   (0x01),
    HID_INPUT (0x02),
  HID_END_COLLECTION,
  0x06, 0x00, 0xff,
  HID_USAGE   (0x01),
  HID_COLLECTION    (HID_COLLECTION_APPLICATION),
    HID_REPORT_ID(2),
    HID_LOGICAL_MINIMUM     (0),
    HID_LOGICAL_MAXIMUM     (255),
    HID_REPORT_SIZE   (8),
    HID_REPORT_COUNT  (MAX_PACKET_SIZE),
    HID_USAGE   (0x01),
    HID_INPUT (0x02),
  HID_END_COLLECTION,
};

static const struct {
  struct usb_hid_descriptor hid_descriptor;
  struct {
    uint8_t bReportDescriptorType;
    uint16_t wDescriptorLength;
  } __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
  .hid_descriptor = {
    .bLength = sizeof(hid_function),
    .bDescriptorType = USB_DT_HID,
    .bcdHID = 0x0111,
    .bCountryCode = 33,
    .bNumDescriptors = 1,
  },
  .hid_report = {
    .bReportDescriptorType = USB_DT_REPORT,
    .wDescriptorLength = sizeof(hid_report_descriptor),
  },
};

const struct usb_endpoint_descriptor hid_endpoints[] = {
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x81,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 64,
    .bInterval = 0x08,
  },
  {
    .bLength = USB_DT_ENDPOINT_SIZE,
    .bDescriptorType = USB_DT_ENDPOINT,
    .bEndpointAddress = 0x01,
    .bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
    .wMaxPacketSize = 64,
    .bInterval = 0x08,
  }
};

const struct usb_interface_descriptor hid_iface = {
  .bLength = USB_DT_INTERFACE_SIZE,
  .bDescriptorType = USB_DT_INTERFACE,
  .bInterfaceNumber = 0,
  .bAlternateSetting = 0,
  .bNumEndpoints = 2,
  .bInterfaceClass = USB_CLASS_HID,
  .bInterfaceSubClass = 0,
  .bInterfaceProtocol = 0,
  .iInterface = 0,

  .endpoint = hid_endpoints,

  .extra = &hid_function,
  .extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
  .num_altsetting = 1,
  .altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
  .bLength = USB_DT_CONFIGURATION_SIZE,
  .bDescriptorType = USB_DT_CONFIGURATION,
  .wTotalLength = 0,
  .bNumInterfaces = 1,
  .bConfigurationValue = 1,
  .iConfiguration = 0,
  .bmAttributes = 0xC0,
  .bMaxPower = 0x32,
  .interface = ifaces,
};

static const char *usb_strings[] = {
  "Manfred's Technologies(c)",
  "Anastasia USB Bootloader",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static int hid_control_request(usbd_device *usbd_dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
      void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req)) {
  (void)complete;
  (void)usbd_dev;

  if ((req->bmRequestType != 0x81) ||
     (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
     (req->wValue != 0x2200))
    return 0;

  /* Handle the HID report descriptor. */
  *buf = (uint8_t *)hid_report_descriptor;
  *len = sizeof(hid_report_descriptor);

  return 1;
}

static void hid_set_config(usbd_device *usbd_dev, uint16_t wValue) {
  (void)wValue;
  (void)usbd_dev;

  usbd_ep_setup(usbd_dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 64, endpoint_in_callback);
  usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_INTERRUPT, 64, endpoint_out_callback);
  usbd_register_control_callback(
        usbd_dev,
        USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
        USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
        hid_control_request);
}


/* Set STM32 to 72 MHz. */
static void clock_setup(void) {
  rcc_clock_setup_in_hse_8mhz_out_72mhz();

  /* Enable GPIOC clock. */
  rcc_periph_clock_enable(RCC_GPIOB);
}

static void gpio_setup(void) {
  gpio_set_mode(USB_CONNECT_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_CONNECT_PIN);
  gpio_set(USB_CONNECT_GPIO, USB_CONNECT_PIN);
}

usbd_device *usbd_dev;

static void usbInit(void) {
  nvic_enable_irq(NVIC_USB_LP_CAN_RX0_IRQ);
  nvic_enable_irq(NVIC_USB_WAKEUP_IRQ);

  gpio_set_mode(USB_CONNECT_GPIO, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, USB_CONNECT_PIN);

  gpio_clear(USB_CONNECT_GPIO, USB_CONNECT_PIN);
  int i;
  for (i = 0; i < 100; i++) \
    __asm__("nop");
  usbd_dev = usbd_init(&st_usbfs_v1_usb_driver, &dev, &config, usb_strings, 2, usbd_control_buffer, sizeof(usbd_control_buffer));
  usbd_register_set_config_callback(usbd_dev, hid_set_config);
}

static void hwInit(void) {
  clock_setup();
  gpio_setup();
  gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO14);

  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO12);
  gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO15);

  gpio_clear(GPIOB, GPIO12);
  gpio_set(GPIOB, GPIO15);
}

uint32_t crcTable[256];

void make_crc_table(uint32_t crcTbl[]) {
  uint32_t POLYNOMIAL = 0xEDB88320;
  uint32_t remainder;
  unsigned char b = 0;
  do {
    // Start with the data byte
    remainder = b;
    uint32_t bit;
    for (bit = 8; bit > 0; --bit) {
      if (remainder & 1)
        remainder = (remainder >> 1) ^ POLYNOMIAL;
      else
        remainder = (remainder >> 1);
    }
    crcTbl[(size_t)b] = remainder;
  } while(0 != ++b);
}

uint32_t gen_crc(unsigned char *p, size_t n, uint32_t crcTbl[]) {
  uint32_t crc = 0xfffffffful;
  size_t i;
    for(i = 0; i < n; i++)
        crc = crcTbl[*p++ ^ (crc&0xff)] ^ (crc>>8);
    return(~crc);
}

void softReset(void) {

}

int main(void) {
  hwInit();

  if (gpio_get(GPIOB, GPIO14)) {
    //run forest run
    SCB_VTOR = APP_ADDRESS & 0xFFFF;
    asm volatile("msr msp, %0"::"g" (*(volatile uint32_t *)APP_ADDRESS));
    (*(void (**)())(APP_ADDRESS + 4))();
  } else {
    usbInit();
    make_crc_table(crcTable);
    while (1) {
      __asm__("nop");
    }
  }
  return 0;
}

void endpoint_in_callback(usbd_device *usbd, uint8_t ep) {

}

uint8_t pageData[1024] = {0};
uint8_t *pageDataPtr;
uint32_t pageAddr;

void sendBootOk(usbd_device *usbd, uint8_t ep) {
  #pragma pack(1)
  BOOTCommand command;
  
  command.cmd = BOOT_RETURN_OK;
  command.param1 = 0x00;
  command.param2 = 0x00;
  uint8_t tbuf[sizeof(command) + 1];
  tbuf[0] = BOOT_MODE_CMD;
  memcpy(&tbuf[1], &command, sizeof(command));
  usbd_ep_write_packet(usbd, ep, &tbuf[0], sizeof(tbuf));
}

void endpoint_out_callback(usbd_device *usbd, uint8_t ep) {
  BOOTCommand command;
  BOOTData data;
  
  uint8_t buf[63];
  
  unsigned int r = usbd_ep_read_packet(usbd, ep, &buf, sizeof(buf));
  
  switch(buf[0]) {
    case BOOT_MODE_CMD: {
      if (r < (sizeof(command)-1))
        return;
      
      memcpy(&command, &buf[1], sizeof(command));
        
      switch(command.cmd) {
        case BOOT_GET_INFO: {
          #pragma pack(1)
          BOOTInfoData info;

          memcpy(&info.bootloader_name, &bootloader_name, sizeof(bootloader_name));
          info.v_major = bootloader_v_major;
          info.v_minor = bootloader_v_minor;
          info.v_fixn = bootloader_v_fixn;

          info.flash_size = *((uint16_t *)0x1FFFF7E0);;
          info.page_size = 1024;
          info.app_addr = APP_ADDRESS;

          uint8_t tbuf[63];
          memset(tbuf, 0, 63);
          tbuf[0] = BOOT_MODE_DATA;
          
          data.size = sizeof(info);
          memset(&data.data, 0, MAX_PACKET_SIZE - 1);
          memcpy(&data.data, &info, data.size);
          
          memcpy(&tbuf[1], &data, sizeof(data));
          usbd_ep_write_packet(usbd_dev, ep, &tbuf[0], sizeof(tbuf));
          break;
        }
        case BOOT_FLASH_LOCK: {
          flash_lock();
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_UNLOCK: {
          flash_unlock();
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_SET_PAGE_ADDR: {
          pageAddr = command.param1;
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_ERASE_PAGE: {
          flash_erase_page(pageAddr);
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_WRITE_DATA_START: {
          gpio_set(GPIOB, GPIO15);
          memset(pageData, 0, 1024);
          pageDataPtr = &pageData[0];
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_WRITE_PAGE: {
          int i;

          for (i = 0; i < 1024; i += 2) {
            uint16_t *dat = (uint16_t *)(&pageData[0] + i);
            flash_program_half_word(pageAddr + i, *dat);
          }
          sendBootOk(usbd, ep);
          break;
        }
        case BOOT_FLASH_CHECK_CRC: {
          gpio_set(GPIOB, GPIO15);
          uint32_t *flash = (uint32_t *) command.param2;
          
          memcpy(&pageData[0], flash, 1024);
          uint32_t crc32 = gen_crc(&pageData[0], 1024, crcTable);
          if (crc32 != command.param1)
            command.cmd = BOOT_RETURN_CRC32_ERROR;
          else
            command.cmd = BOOT_RETURN_OK;
            
          command.param1 = crc32;
          
          uint8_t tbuf[sizeof(command) + 1];
          
          tbuf[0] = BOOT_MODE_CMD;
          memcpy(&tbuf[1], &command, sizeof(command));

          usbd_ep_write_packet(usbd, ep, &tbuf[0], sizeof(tbuf));
          break;
        }
      }
      break;
    }
    case BOOT_MODE_DATA: {
      gpio_toggle(GPIOB, GPIO15);
      memcpy(&data, &buf[1], sizeof(data));
      
      memcpy(pageDataPtr, &data.data, data.size);
      pageDataPtr += data.size;
      sendBootOk(usbd, ep);
      break;
    }
    
  }
}

void usb_wakeup_isr(void) {
  usbd_poll(usbd_dev);
}

void usb_hp_can_tx_isr(void) {
  usbd_poll(usbd_dev);
}

void usb_lp_can_rx0_isr(void) {
  usbd_poll(usbd_dev);
}
