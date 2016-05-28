#include <stdlib.h>
#include <string.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>

#define HID_USAGE_PAGE(up)    (uint8_t)(0x05 & 255),(uint8_t)(((up)) & 255)
#define HID_USAGE(u)        (uint8_t)(0x09 & 255),(uint8_t)(((u)) & 255)
#define HID_COLLECTION(c)     (uint8_t)(0xA1 & 255),(uint8_t)(((c)) & 255)
#define HID_END_COLLECTION    0xC0

#define HID_USAGE_MINIMUM(x)      (uint8_t)(0x19 & 255),(uint8_t)(((x)) & 255)
#define HID_USAGE_MAXIMUM(x)      (uint8_t)(0x29 & 255),(uint8_t)(((x)) & 255)
#define HID_LOGICAL_MINIMUM(x)    (uint8_t)(0x15 & 255),(int8_t)(((x)) & 255)
#define HID_LOGICAL_MAXIMUM(x)    (uint8_t)(0x25 & 255),(int8_t)(((x)) & 255)
#define HID_REPORT_COUNT(x)     (uint8_t)(0x95 & 255),(uint8_t)(((x)) & 255)
#define HID_REPORT_SIZE(x)      (uint8_t)(0x75 & 255),(uint8_t)(((x)) & 255)
#define HID_INPUT(x)        (uint8_t)(0x81 & 255),(uint8_t)(((x)) & 255)
#define HID_OUTPUT(x)        (uint8_t)(0x91 & 255),(uint8_t)(((x)) & 255)
#define HID_REPORT_ID(x)        (uint8_t)(0x85 & 255),(uint8_t)(((x)) & 255)

#define HID_COLLECTION_PHYSICAL   0x00
#define HID_COLLECTION_APPLICATION  0x01
#define HID_COLLECTION_LOGICAL    0x02
#define HID_COLLECTION_REPORT   0x03
#define HID_COLLECTION_NAMED_ARRAY  0x04
#define HID_COLLECTION_USAGE_SWITCH 0x05
#define HID_COLLECTION_USAGE_MODIFIER 0x06

#define USB_CONNECT_GPIO GPIOB
#define USB_CONNECT_PIN GPIO9

#define BOOT_GET_INFO 0x01
#define BOOT_FLASH_LOCK 0x04
#define BOOT_FLASH_UNLOCK 0x05
#define BOOT_FLASH_SET_PAGE_ADDR 0x12
#define BOOT_FLASH_ERASE_PAGE 0x06
#define BOOT_FLASH_WRITE_DATA_START 0x07
#define BOOT_FLASH_WRITE_PAGE 0x08
#define BOOT_FLASH_CHECK_CRC 0x09
#define BOOT_FLASH_READ_PAGE 0x10
#define BOOT_FLASH_READ_DATA 0x11

#define BOOT_RETURN_OK 0x01
#define BOOT_RETURN_OVEFLOW 0x02
#define BOOT_RETURN_CRC32_ERROR 0x03

#define TRUE 1
#define FALSE 0

#define APP_ADDRESS 0x08002800
#define MAX_PACKET_SIZE 62

#define BOOT_MODE_CMD 1
#define BOOT_MODE_DATA 2

#pragma pack(1)
typedef struct {
  unsigned char firmware_name[33];
  unsigned char firmware_v[5];
} BOOTFirmwareInfo;

#pragma pack(1)
typedef struct {
  uint8_t cmd;
  uint32_t param1;
  uint32_t param2;
} BOOTCommand;

#pragma pack(1) 
typedef struct {
  uint8_t data[MAX_PACKET_SIZE-1];
  uint8_t size;
} BOOTData;

#pragma pack(1)
typedef struct {
  unsigned char bootloader_name[32];
  uint8_t v_major;
  uint8_t v_minor;
  uint8_t v_fixn;
  uint16_t flash_size;
  uint16_t page_size;
  uint32_t app_addr;
} BOOTInfoData;

void endpoint_in_callback(usbd_device *usbd_dev, uint8_t ep);
void endpoint_out_callback(usbd_device *usbd_dev, uint8_t ep);

void sendBootOk(usbd_device *usbd, uint8_t ep);
void make_crc_table(uint32_t crcTable[]);
uint32_t gen_crc(unsigned char *p, size_t n, uint32_t crcTbl[]);
void softReset(void);

//#pragma pack(1)


