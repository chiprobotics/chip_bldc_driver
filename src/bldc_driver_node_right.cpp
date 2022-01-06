/**
Software License Agreement BSD
\file      bldc_driver_node.cpp
\copyright Copyright (c) 2017, Chip Robotics, All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that
the following conditions are met:
* Redistributions of source code must retain the above copyright notice, this list of conditions and the
following disclaimer.
* Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
following disclaimer in the documentation and/or other materials provided with the distribution.
* Neither the name of NAME nor the names of its contributors may be used to endorse or promote
products derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WAR-
RANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, IN-
DIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "chip_bldc_driver/bldc_controller.h"
#include "chip_bldc_driver/bldc_serial.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "bldc_driver_node");
  ros::NodeHandle nh("~");

  std::string port = "/dev/ttyUSB0";
  int32_t baud = 115200;
  nh.param<std::string>("port", port, port);

  bldc_serial::BldcSerial serial(port.c_str(), baud);
  
  bldc_controller::BldcController_right controller(&serial);

  while (ros::ok())
  {
    ROS_INFO("Attempting connection to %s at %i baud.", port.c_str(), baud);

    if (serial.connect())
    {
      ROS_INFO("Connection Succesful");
      ros::AsyncSpinner spinner(1);
      spinner.start();
      while (ros::ok())
      {
        serial.read();
      }
      spinner.stop();
    }
    else
    {
      ROS_WARN("Problem connecting to serial device, again after 1 second");
      sleep(1);
    }
  }

}
