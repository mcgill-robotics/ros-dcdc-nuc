// Copyright (c) 2017 by McGill Robotics.
// Written by Bei Chen Liu <bei.liu@mail.mcgill.ca>
// All Rights Reserved
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//

#include "dcdc_nuc.h"

int main(int argc, char **argv) {
  Dcdc_Nuc dcdc_nuc = Dcdc_Nuc();

  while (1) {
    Dcdc_Nuc_Data data = dcdc_nuc.get_data();
    printf("ProtectionOK: %s\n", data.protection_ok ? "true" : "false");
    printf("nProtectionFault: %s\n", data.not_protection_fault ? "true" :
                                                                 "false");
    printf("nOpenLED: %s\n", data.not_open_led ? "true" : "false");
    printf("nShortLED: %s\n", data.not_short_led ? "true" : "false");
    printf("CFG1: %s\n", data.cfg1 ? "true" : "false");
    printf("CFG2: %s\n", data.cfg2 ? "true" : "false");
    printf("CFG3: %s\n", data.cfg3 ? "true" : "false");
    printf("Mode: %s\n", data.mode ? "automotive" : "dumb");
    printf("USBSnense: %s\n", data.usb_sense ? "true" : "false");
    printf("VInGood: %s\n", data.input_voltage_good ? "true" : "false");
    printf("IgnGood: %s\n", data.ignition_voltage_good ? "true" : "false");
    printf("MBAlivePOut: %s\n", data.mobo_alive_pout ? "true" : "false");
    printf("IgnRaised: %s\n", data.ignition_raised ? "true" : "false");
    printf("IgnFalled: %s\n", data.ignition_falled ? "true" : "false");
    printf("OutEnabled: %s\n", data.output_enabled ? "true" : "false");
    printf("ThumpEnabled: %s\n", data.thump_ouput_enabled ? "true" : "false");
    printf("ControlFrequency: %s\n", data.control_frequency ? "true" : "false");
    printf("nPSW: %s\n", data.not_power_switch ? "true" : "false");

    printf("InputVoltage: %.2f\n", data.input_voltage);
    printf("InputCurrent: %.2f\n", data.input_current);
    printf("OutputVoltage: %.2f\n", data.output_voltage);
    printf("OutputCurrent: %.2f\n", data.output_current);
    printf("OutputPower: %.2f\n", data.output_power);
    printf("Temperature: %.2f\n", data.temperature);
    printf("Ignition Voltage: %.2f\n", data.ignition_voltage);
    printf("ThumpVoltage: %.2f\n", data.thump_voltage);

    printf("TimerInit: %d\n", data.timer_init);
    printf("TimerIgn2OutOn: %d\n", data.timer_ignition_to_output_on);
    printf("TimerThumpOnOff: %d\n", data.timer_thump_output_on_off);
    printf("TimerOutOn2MBOnPulse: %d\n", data.timer_output_on_to_mobo_on_pulse);
    printf("TimerMBPulseWidth: %d\n", data.timer_mobo_pulse_width);
    printf("TimerIgnCancel: %d\n", data.timer_ignition_cancel);
    printf("TimerIgnOff2MBOffPulse: %d\n",
           data.timer_ignition_off_to_mobo_off_pulse);
    printf("TimerHardOff: %d\n", data.timer_hard_off);
    printf("TimerVInCount: %d\n", data.timer_input_voltage_count);
    printf("TimerIgnCont: %d\n", data.timer_ignition_voltage_count);

    printf("StateMachineState: %d\n", data.state_machine_state);
    printf("Mode2: %d\n", data.mode2);
    printf("FirmwareVerMajor: %d\n", data.firmware_version_major);
    printf("FirmwareVerMinor: %d\n", data.firmware_version_minor);
  }

  return 0;
}
