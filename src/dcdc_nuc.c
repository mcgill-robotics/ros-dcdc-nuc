/*
 * Copyright (c) 2011 by Mini-Box.com, iTuner Networks Inc.
 * Written by Nicu Pavel <npavel@mini-box.com>
 * All Rights Reserved
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

#include <usb.h>

#include "dcdc-nuc.h"

#define P(t, v...) fprintf(stderr, t "\n", ##v)



int dcdc_send(struct usb_dev_handle *h, unsigned char *data, int size) {
  if (data == NULL) {
    return -1;
  }
  return usb_interrupt_write(h, USB_ENDPOINT_OUT + 1,
          (char *) data, size, 1000);
}

int dcdc_recv(struct usb_dev_handle *h, unsigned char *data, int size,
  int timeout) {
  if (data == NULL) {
  return -1;
  }

  return usb_interrupt_read(h, USB_ENDPOINT_IN + 1,
          (char *) data, size, timeout);
}

struct usb_dev_handle * dcdc_connect() {
  struct usb_bus *b;
  struct usb_device *d;
  struct usb_dev_handle *h = NULL;

  usb_init();
  usb_set_debug(0);
  usb_find_busses();
  usb_find_devices();

  for (b = usb_get_busses(); b != NULL; b = b->next) {
    for (d = b->devices; d != NULL; d = d->next) {
      if ((d->descriptor.idVendor == NUC_VID) &&
        (d->descriptor.idProduct == NUC_PID)) {
        h = usb_open(d);
        break;
      }
    }
  }
  return h;
}

int dcdc_setup(struct usb_dev_handle *h) {
  char buf[65535];

  if (h == NULL) {
    return -1;
  }

  if (usb_get_driver_np(h, 0, buf, sizeof(buf)) == 0) {
    if (usb_detach_kernel_driver_np(h, 0) < 0) {
      fprintf(stderr, "Cannot detach from kernel driver\n");
      return -2;
    }
  }

  if (usb_set_configuration(h, 1) < 0) {
    fprintf(stderr, "Cannot set configuration 1 for the device\n");
    return -3;
  }

  usleep(1000);

  if (usb_claim_interface(h, 0) < 0) {
    fprintf(stderr, "Cannot claim interface 0\n");
    return -4;
  }

  if (usb_set_altinterface(h, 0) < 0) {
    fprintf(stderr, "Cannot set alternate configuration\n");
    return -5;
  }


  /* Doesn't seem to need this for it to work
  if (usb_control_msg(h, USB_TYPE_CLASS + USB_RECIP_INTERFACE,
    0x000000a, 0x0000000, 0x0000000, buf, 0x0000000, 1000)
    < 0) {
    fprintf(stderr, "Cannot send control message\n");
    return -6;
  }
  */
  return 0;
}


unsigned int g_memTerm[TERMISTOR_CONSTS] = {
  (unsigned int) 0xB,
  (unsigned int) 0xE,
  (unsigned int) 0x13,
  (unsigned int) 0x19,
  (unsigned int) 0x1F,
  (unsigned int) 0x28,
  (unsigned int) 0x32,
  (unsigned int) 0x3E,
  (unsigned int) 0x4C,
  (unsigned int) 0x5D,
  (unsigned int) 0x6F,
  (unsigned int) 0x85,
  (unsigned int) 0x9D,
  (unsigned int) 0xB8,
  (unsigned int) 0xD6,
  (unsigned int) 0xF6,
  (unsigned int) 0x118,
  (unsigned int) 0x13C,
  (unsigned int) 0x162,
  (unsigned int) 0x188,
  (unsigned int) 0x1B0,
  (unsigned int) 0x1D6,
  (unsigned int) 0x1FC,
  (unsigned int) 0x222,
  (unsigned int) 0x246,
  (unsigned int) 0x268,
  (unsigned int) 0x289,
  (unsigned int) 0x2A8,
  (unsigned int) 0x2C5,
  (unsigned int) 0x2E0,
  (unsigned int) 0x2F9,
  (unsigned int) 0x310,
  (unsigned int) 0x325,
  (unsigned int) 0x339
};

double therm2temp(unsigned int termistor) {
  float value;
  if (termistor <= g_memTerm[0]) {
    value = -40;
  } else if (termistor >= g_memTerm[TERMISTOR_CONSTS-1]) {
    value = 125;
  } else {
    int pos = -1;
    for (int i = TERMISTOR_CONSTS-1; i >= 0; i--) {
      if (termistor >= g_memTerm[i]) {
        pos = i;
        break;
      }
    }

    if (termistor == g_memTerm[pos]) {
      value = pos * 5 - 40;
    } else {
      int t1 = pos * 5 - 40;
      int t2 = (pos + 1) * 5 - 40;

      unsigned int d1 = g_memTerm[pos];
      unsigned int d2 = g_memTerm[pos + 1];

      double dtemp = ((double)termistor -
        (double)d1) * ((double)t2 - (double)t1) / ((double)d2 - (double)d1);

      int temp = (int)ceil(0.2) + t1;

      value = temp;
    }
  }
  return value;
}

void dcdc_parse_io_data(unsigned char *data) {
  unsigned int i, input_flag, output_flag;
  float input_voltage, input_current, output_voltage, output_current,
    temperature, ignition_voltage, output_power, thump_voltage;

  input_flag = (((unsigned int)data[1]) << 8) | data[2];
  output_flag = (((unsigned int)data[3]) << 8) | data[4];

  i = (((unsigned int)data[5]) << 8) | data[6];
  input_voltage = (float)i / 1000;

  i = (((unsigned int)data[7]) << 8) | data[8];
  input_current = (float)i / 1000;

  i = (((unsigned int)data[9]) << 8) | data[10];
  output_voltage = (float)i / 1000;

  i = (((unsigned int)data[11]) << 8) | data[12];
  output_current = (float)i / 1000;

  if (output_flag & 0x0100) {  // ENOUT
    i = (data[13] << 8) | data[14];
    temperature = therm2temp(i);
  } else {
    temperature = 1000.0;
  }

  i = (((unsigned int)data[15]) << 8) | data[16];
  ignition_voltage = (float)i / 1000;

  i =  (((unsigned int)data[17]) << 24)
    |(((unsigned int)data[18]) << 16)
    |(((unsigned int)data[19]) << 8)
    | data[20];
  output_power = (float)i / 1000;

  i = (((unsigned int)data[21]) << 8) | data[22];
  thump_voltage = (float)i / 1000;

  P("Protection ok: %s", ((input_flag & 0x0001) ? "True":"False"));
  P("nProtection fault: %s", ((input_flag & 0x0002) ? "True":"False"));
  P("nOpen LED: %s", ((input_flag & 0x04) ? "True":"False"));
  P("nShort LED: %s", ((input_flag & 0x08) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x0010) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x0020) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x0040) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x0080) ? "True":"False"));
  P("CFG1: %s", ((input_flag & 0x0100) ? "True":"False"));
  P("CFG2: %s", ((input_flag & 0x0200) ? "True":"False"));
  P("CFG3: %s", ((input_flag & 0x0400) ? "True":"False"));
  P("Mode: %s", ((input_flag & 0x0800) ? "True":"False"));
  P("USB Sense: %s", ((input_flag & 0x1000) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x2000) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x4000) ? "True":"False"));
  // P("---: %s", ((input_flag & 0x8000) ? "True":"False"));

  P("VIN_GOOD: %s", ((output_flag & 0x0001) ? "True":"False"));
  P("IGN_GOOD : %s", ((output_flag & 0x0002) ? "True":"False"));
  P("MOB_ALIVE_POUT: %s", ((output_flag & 0x04) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x08) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x0010) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x0020) ? "True":"False"));
  P("IGN_RISED: %s", ((output_flag & 0x0040) ? "True":"False"));
  P("IGN_FALLED: %s", ((output_flag & 0x0080) ? "True":"False"));
  P("ENOUT: %s", ((output_flag & 0x0100) ? "True":"False"));
  P("Thump: %s", ((output_flag & 0x0200) ? "True":"False"));
  P("ctrlFREQ: %s", ((output_flag & 0x0400) ? "True":"False"));
  P("nPSW: %s", ((output_flag & 0x0800) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x1000) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x2000) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x4000) ? "True":"False"));
  // P("---: %s", ((output_flag & 0x8000) ? "True":"False"));

  P("input voltage: %.2f", input_voltage);
  P("input current: %.2f", input_current);
  P("output voltage: %.2f", output_voltage);
  P("output current: %.2f", output_current);
  P("temperature: %.2f", temperature);
  P("ignition voltage: %.2f", ignition_voltage);
  P("output power: %2f", output_power);
  P("thump voltage: %2f", thump_voltage);
}

void dcdc_parse_io_data2(unsigned char *data) {
  unsigned int timer_init, timer_ign2out_on, timer_thump, timer_out2mb_pulse,
    timer_mb_pulse_width, timer_ign_cancel, timer_ign_off2mb_pulse_off,
    timer_hard_off, timer_vin_cnt, timer_iin_cnt, state_machine, mode,
    version_major, version_minor;

  timer_init = ((((unsigned int)data[1]) << 8) | data[2]) * 10;
  timer_ign2out_on = ((((unsigned int)data[3]) << 8) | data[4]) * 10;
  timer_thump = (((unsigned int)data[5]) << 8) | data[6];
  timer_out2mb_pulse = (((unsigned int)data[7]) << 8) | data[8];
  timer_mb_pulse_width = ((((unsigned int)data[9]) << 8) | data[10]) * 10;
  timer_ign_cancel = (((unsigned int)data[11]) << 8) | data[12];
  timer_ign_off2mb_pulse_off = (((unsigned int)data[13]) << 8) | data[14];
  timer_hard_off = (((unsigned int)data[15]) << 8) | data[16];
  timer_vin_cnt = ((((unsigned int)data[17]) << 8) | data[18]) * 10;
  timer_iin_cnt = ((((unsigned int)data[19]) << 8) | data[20]) * 10;

  state_machine = (unsigned int)data[21];
  mode = data[22];

  version_major = (data[25] >> 4) & 0x0F;
  version_minor = data[25] & 0x0F;

  P("init timer(ms): %d", timer_init);
  P("ignition on to ouput on(s): %d", timer_ign2out_on);
  P("thump timer(s): %d", timer_thump);
  P("ouput on to motherboard pulse on(ms): %d", timer_out2mb_pulse);
  P("motherboad pulse width(ms): %d", timer_mb_pulse_width);
  P("ignition cancel(s): %d", timer_ign_cancel);
  P("ignition off to motherboad pulse off(s): %d", timer_ign_off2mb_pulse_off);
  P("hard off timer(s): %d", timer_hard_off);
  P("input voltage count(ms): %d", timer_vin_cnt);
  P("input current count(ms): %d", timer_iin_cnt);
  P("state machine state: %d", state_machine);
  P("mode: %s", mode ? "automotive" : "dumb");
  P("firmware version: %d.%d", version_major, version_minor);
}

void dcdc_parse_internal_msg(unsigned char *data) {
    P("Parsing INTERNAL MESSAGE: Not implemented");
}

void dcdc_parse_mem(unsigned char *data) {
    P("Parsing MEM READ IN: Not implemented");
}


int dcdc_get_io_data(struct usb_dev_handle *h, unsigned char *buf, int buflen) {
  unsigned char c[1];
  int ret = 0;

  if (buflen < MAX_TRANSFER_SIZE) {
    return -1;
  }

  c[0] = NUC_OUT_REPORT_IO_DATA;

  if ( dcdc_send(h, c, 1) < 0) {
    fprintf(stderr, "Cannot send command to device\n");
    return -2;
  }

  ret = dcdc_recv(h, buf, MAX_TRANSFER_SIZE, 100);

  if ( ret < 0) {
    fprintf(stderr, "Cannot get device status\n");
    return -3;
  }

  return ret;
}

int dcdc_get_io_data2(struct usb_dev_handle *h,
  unsigned char *buf, int buflen) {
  unsigned char c[1];
  int ret = 0;

  if (buflen < MAX_TRANSFER_SIZE) {
    return -1;
  }

  c[0] = NUC_OUT_REPORT_IO_DATA2;

  if (dcdc_send(h, c, 1) < 0) {
    fprintf(stderr, "Cannot send command to device\n");
    return -2;
  }

  ret = dcdc_recv(h, buf, MAX_TRANSFER_SIZE, 100);

  if (ret < 0) {
    fprintf(stderr, "Cannot get device status\n");
    return -3;
  }

  return ret;
}


int dcdc_parse_data(unsigned char *data, int size) {
  if (data == NULL) {
    return -1;
  }

  if (size < 24) {
    fprintf(stderr, "Data size error! Size: %d\n", size);
    return -2;
  }
  switch (data[0]) {
    case NUC_IN_REPORT_IO_DATA:
      dcdc_parse_io_data(data);
      break;
    case NUC_IN_REPORT_IO_DATA2:
      dcdc_parse_io_data2(data);
      break;
    default:
      fprintf(stderr, "Unknown message\n");
  }

  return 0;
}

void showhelp(char *prgname) {
    printf ("Usage: %s [OPTION]\n", prgname);
    printf ("Options:.\n");
    printf (" -h \t show help message\n");
}

int main(int argc, char **argv) {
  struct usb_dev_handle *h;
  unsigned char data[MAX_TRANSFER_SIZE];
  int ret;
  char *s;
  int arg = 0;

  while ( ++arg < argc ){
    s = argv[arg];
	if (strncmp(s, "-h", 2) == 0) {
      showhelp(argv[0]);
      return 0;
    }
  }

  h = dcdc_connect();

  if (h == NULL){
    fprintf(stderr, "Cannot connect to DCDC-NUC\n");
    return 1;
  }

  if (dcdc_setup(h) < 0){
    fprintf(stderr, "Cannot setup device\n");
    return 2;
  }

  if ((ret = dcdc_get_io_data(h, data, MAX_TRANSFER_SIZE)) <= 0){
    fprintf(stderr, "Failed to get io data from device\n");
    return 3;
  }
  dcdc_parse_data(data, ret);


  if ((ret = dcdc_get_io_data2(h, data, MAX_TRANSFER_SIZE)) <= 0){
    fprintf(stderr, "Failed to get io data 2 from device\n");
    return 3;
  }
  dcdc_parse_data(data, ret);

  return 0;
}
