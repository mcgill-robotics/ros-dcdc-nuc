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
#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <string>


Dcdc_Nuc * dcdc_nuc;
Dcdc_Nuc_Data dcdc_data;

void update_status(diagnostic_updater::DiagnosticStatusWrapper & stat) {
  std::string state;
  switch (dcdc_data.state_machine_state) {
      case 1: state = "Low Power"; break;
      case 2: state = "Off"; break;
      case 3: state = "Wait ignition to output on"; break;
      case 4: state = "Ouput on"; break;
      case 5: state = "Output on to motherboard pulse"; break;
      case 6: state = "Motherboard pulse on"; break;
      case 7: state = "Normal"; break;
      case 8: state = "Ignition off to motherboard pulse"; break;
      case 9: state = "Hard off delay"; break;
      default: state = "Unknown"; break;
  }

  if (dcdc_data.state_machine_state == 7) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, state);
  } else {
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, state);
  }
  stat.add("State", state);

  stat.add("Mode", dcdc_data.mode ? "Automotive" : "Dumb");
  stat.add("Output Enabled", dcdc_data.output_enabled);
  stat.add("Input Voltage Good", dcdc_data.input_voltage_good);
  stat.add("Ignition Voltage Good", dcdc_data.ignition_voltage_good);
  stat.add("Mobo Alive (USB)", dcdc_data.usb_sense);
  stat.add("Mobo Alive (Power)", dcdc_data.mobo_alive_pout);
  stat.add("Ignition Raised", dcdc_data.ignition_raised);
  stat.add("Thump Ouput Enbaled", dcdc_data.thump_output_enabled);

  if (dcdc_data.timer_init) {
      stat.add("Timer Init", dcdc_data.timer_init);
  }

  if (dcdc_data.timer_ignition_to_output_on) {
    stat.add("Timer Output On", dcdc_data.timer_ignition_to_output_on);
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
        "Turning on output soon");
  }

  if (dcdc_data.timer_output_on_to_mobo_on_pulse) {
    stat.add("Timer Mobo On Pulse",
        dcdc_data.timer_output_on_to_mobo_on_pulse);
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
        "Sending motherboard on signal soon");
  }

  if (dcdc_data.timer_ignition_cancel) {
    stat.add("Timer Ignition Cancel", dcdc_data.timer_ignition_cancel);
  }

  if (dcdc_data.timer_ignition_off_to_mobo_off_pulse) {
    stat.add("Timer Mobo On Pulse",
        dcdc_data.timer_ignition_off_to_mobo_off_pulse);
    stat.mergeSummary(diagnostic_msgs::DiagnosticStatus::WARN,
        "Sending motherboard off signal soon");
  }

  if (dcdc_data.timer_hard_off) {
    unsigned int hard_off = dcdc_data.timer_hard_off;
    stat.add("Timer Hard Off", hard_off);

    if (hard_off > 15) {
      stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::OK,
          "Hard off in %d seconds", hard_off);
    } else if (hard_off > 5) {
      stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::WARN,
          "Hard off in %d seconds", hard_off);
    } else {
      stat.mergeSummaryf(diagnostic_msgs::DiagnosticStatus::ERROR,
          "Hard off in %d second%s", hard_off, hard_off == 1 ? "" : "s");
    }
  }

  stat.addf("Firmware Version", "%d.%d", dcdc_data.firmware_version_major,
      dcdc_data.firmware_version_minor);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "dcdc_psu");
  ros::NodeHandle nh("~");

  dcdc_nuc = new Dcdc_Nuc();

  ros::Rate loop_rate(10);

  diagnostic_updater::Updater updater;
  std::string hardware_id;
  nh.param<std::string>("hardware_id", hardware_id, "dcdc_psu");
  updater.setHardwareID(hardware_id);
  updater.add("Status updater", update_status);

  ros::Publisher v_in_pub =
      nh.advertise<std_msgs::Float64>("input_voltage", 1);

  ros::Publisher i_in_pub =
      nh.advertise<std_msgs::Float64>("input_current", 1);

  ros::Publisher v_out_pub =
      nh.advertise<std_msgs::Float64>("output_voltage", 1);

  ros::Publisher i_out_pub =
      nh.advertise<std_msgs::Float64>("output_current", 1);

  ros::Publisher p_out_pub =
      nh.advertise<std_msgs::Float64>("output_power", 1);

  ros::Publisher temp_pub =
      nh.advertise<std_msgs::Float64>("temperature", 1);

  ros::Publisher v_ign_pub =
      nh.advertise<std_msgs::Float64>("ignition_voltage", 1);

  ros::Publisher thump_pub =
      nh.advertise<std_msgs::Float64>("thump_voltage", 1);

  dcdc_data = dcdc_nuc->get_data();

  ROS_INFO("DCDC NUC PSU found in %s mode", dcdc_data.mode ?
      "Automotive" : "Dumb");

  ROS_INFO("Firmware Verion %d.%d", dcdc_data.firmware_version_major,
      dcdc_data.firmware_version_minor);

  ROS_INFO("Starting DCDC NUC PSU node");

  while (ros::ok()) {
    dcdc_data = dcdc_nuc->get_data();

    std_msgs::Float64 msg;

    msg.data = dcdc_data.input_voltage;
    v_in_pub.publish(msg);

    msg.data = dcdc_data.input_current;
    i_in_pub.publish(msg);

    msg.data = dcdc_data.output_voltage;
    v_out_pub.publish(msg);

    msg.data = dcdc_data.output_current;
    i_out_pub.publish(msg);

    msg.data = dcdc_data.output_power;
    p_out_pub.publish(msg);

    msg.data = dcdc_data.ignition_voltage;
    v_ign_pub.publish(msg);

    msg.data = dcdc_data.temperature;
    temp_pub.publish(msg);

    msg.data = dcdc_data.thump_voltage;
    thump_pub.publish(msg);

    if (dcdc_data.timer_ignition_cancel == 0) {
      ROS_INFO_ONCE("Ignition Cancel timer reached");
      ROS_INFO_ONCE("Auto-shutdown enabled");
    }

    if (!dcdc_data.ignition_voltage_good && dcdc_data.output_enabled) {
      ROS_INFO_THROTTLE(60, "Ignition voltage low, shutdown Soon");
    }

    unsigned int mobo_off = dcdc_data.timer_ignition_off_to_mobo_off_pulse;
    if (mobo_off > 0) {
      ROS_WARN_THROTTLE(1, "Sending motherboard off signal in %d second%s",
          mobo_off, mobo_off == 1 ? "" : "s");
    }

    unsigned int hard_off = dcdc_data.timer_hard_off;
    if (hard_off > 15) {
      ROS_INFO_THROTTLE(1, "Hard shutdown in %d seconds", hard_off);
    } else if (hard_off > 5) {
      ROS_WARN_THROTTLE(1, "Hard shutdown in %d seconds", hard_off);
    } else if (hard_off != 0) {
      ROS_ERROR_THROTTLE(1, "Hard shutdown in %d second%s", hard_off,
              hard_off == 1 ? "" : "s");
    }

    updater.update();
    loop_rate.sleep();
  }

  return 0;
}
