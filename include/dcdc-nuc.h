#ifndef DCDC_NUC_H_
#define DCDC_NUC_H_

#define NUC_PID       0xd006
#define NUC_VID       0x04D8

#define MAX_TRANSFER_SIZE 32

#define STATUS_OK     0x00
#define STATUS_ERASE  0x01
#define STATUS_WRITE  0x02
#define STATUS_READ	  0x03
#define STATUS_ERROR  0xff

#define NUC_OUT_REPORT_IO_DATA  0x81
#define NUC_IN_REPORT_IO_DATA   0x82
#define NUC_OUT_REPORT_IO_DATA2 0x83
#define NUC_IN_REPORT_IO_DATA2  0x84
#define NUC_OUT_REPORT_IO_DATA3 0x85
#define NUC_IN_REPORT_IO_DATA3  0x86
#define NUC_OUT_REPORT_IO_DATA4 0x87
#define NUC_IN_REPORT_IO_DATA4  0x88

#define NUC_CMD_OUT             0xB1
#define NUC_CMD_IN              0xB2
#define NUC_MEM_READ_OUT        0xA1
#define NUC_MEM_READ_IN         0xA2
#define NUC_MEM_WRITE_OUT       0xA3
#define NUC_MEM_WRITE_IN        0xA4
#define NUC_MEM_ERASE           0xA5

#define NUC_ENTER_BOOTLOADER_OUT  0xA9
#define NUC_ENTER_BOOTLOADER_IN   0xAA

#define INTERNAL_MESG               0xFF
#define INTERNAL_MESG_DISCONNECTED  0x01

// commands
#define DCMD_RESTART_NUC          0xAA

/* For reading out memory */
#define TYPE_CODE_MEMORY      0x00
#define TYPE_EPROM_EXTERNAL   0x01
#define TYPE_EPROM_INTERNAL   0x02
#define TYPE_CODE_SPLASH      0x03

#define FLASH_REPORT_ERASE_MEMORY 0xF2 /* AddressLo : AddressHi : AddressUp (anywhere inside the 64 byte-block to be erased) */
#define FLASH_REPORT_READ_MEMORY  0xF3 /* AddressLo : AddressHi : AddressUp : Data Length (1...32) */
#define FLASH_REPORT_WRITE_MEMORY 0xF4 /* AddressLo : AddressHi : AddressUp : Data Length (1...32) : Data.... */
#define KEYBD_REPORT_ERASE_MEMORY 0xB2 /* same as F2 but in keyboard mode */
#define KEYBD_REPORT_READ_MEMORY  0xB3 /* same as F3 but in keyboard mode */
#define KEYBD_REPORT_WRITE_MEMORY 0xB4 /* same as F4 but in keyboard mode */
#define KEYBD_REPORT_MEMORY       0x41 /* response to b3,b4 */

#define IN_REPORT_EXT_EE_DATA     0x31
#define OUT_REPORT_EXT_EE_READ    0xA1
#define OUT_REPORT_EXT_EE_WRITE   0xA2

#define IN_REPORT_INT_EE_DATA     0x32
#define OUT_REPORT_INT_EE_READ    0xA3
#define OUT_REPORT_INT_EE_WRITE   0xA4

/* MEASUREMENT CONSTANTS */
#define CT_RW (double)75
#define CT_RP (double)5000
#define CT_R1_VOUT (double)56200
#define CT_R2_VOUT (double)2740
#define CT_V_FEEDBACK_OUT (double)1.2

#define CHECK_CHAR (unsigned char)0xAA /* used for line/write check */

#define MAX_MESSAGE_CNT 256

#define TERMISTOR_CONSTS 34

#define SETTINGS_ADDR_START 0x003000
#define SETTINGS_ADDR_END   0x003080
#define SETTINGS_PACKS      8

/* USB communication wrappers */
struct usb_dev_handle * dcdc_connect();
int dcdc_send(struct usb_dev_handle *h, unsigned char *data, int size);
int dcdc_recv(struct usb_dev_handle *h, unsigned char *data, int size,
  int timeout);
int dcdc_setup(struct usb_dev_handle *h);

/* DCDC USB protocol */
int dcdc_get_io_data(struct usb_dev_handle *h, unsigned char *buf, int buflen);
int dcdc_get_io_data2(struct usb_dev_handle *h, unsigned char *buf, int buflen);
int dcdc_parse_data(unsigned char *data, int size);

/* DCDC USB data parsing */
void dcdc_parse_io_data(unsigned char *data);
void dcdc_parse_io_data2(unsigned char *data);

/* Convert thermisitor value to temperature */
double therm2temp(unsigned int termistor);

/* from windows source with small sanity edits */



#endif  // DCDC_NUC_H_
