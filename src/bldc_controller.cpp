/*
  Software License Agreement BSD
  \file      bldc_controller.cpp
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
#include <string>

namespace bldc_controller
{

  /**
    * Constructs
    */
  BldcController::BldcController(bldc_serial::BldcSerial *s) : serial(s), nh_("~")  {
    sub_cmd_ = nh_.subscribe("/bldc_driver_node/Command_left", 1, &BldcController::motorCommand, this);
    sub_KP = nh_.subscribe(  "/bldc_driver_node/Left_Motor_KP", 2, &BldcController::KP_Callback);  // 2 is the queue size
    sub_KI = nh_.subscribe(  "/bldc_driver_node/Left_Motor_KI", 2, &BldcController::KI_Callback);  // 2 is the queue size
    sub_KD = nh_.subscribe(  "/bldc_driver_node/Left_Motor_KD", 2, &BldcController::KD_Callback); // 2 is the queue size

    timeout_timer_ = nh_.createTimer(ros::Duration(2), &BldcController::timeoutCallback, this);
    timeout_timer_.start();
  }

  BldcController::~BldcController()
  {  }

  void BldcController::KP_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KP value is : [%d]", msg->data);
    motor_kp = msg->data;
    serial->Send_KP(motor_kp); // send kp to motor
  }
  void BldcController::KI_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KI value is : [%d]", msg->data);
    motor_ki = msg->data;
    serial->Send_KI(motor_ki); // send ki to motor
  }
  void BldcController::KD_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KD value is : [%d]", msg->data);
    motor_kd = msg->data;
    serial->Send_KD(motor_kd); // send kd to motor
  }


  void BldcController::motorCommand(const chip_bldc_driver::Command &command)  {
    timeout_timer_.stop();
    timeout_timer_.start();
    serial->sendMotorCommand(command.motor_command);
  }

  void BldcController::timeoutCallback(const ros::TimerEvent &)
  {
    // In case the motorCommand function is not called the timeout will be called after 2 seconds to stop the motor.
    serial->sendMotorCommand(0);
  }

    /**
    * Constructs for right
    */
  BldcController_right::BldcController_right(bldc_serial::BldcSerial *s) : serial(s), nh_("~")
  {
    sub_cmd_ = nh_.subscribe("/bldc_driver_node/Command_right", 1, &BldcController_right::motorCommand, this);
    sub_KP = nh_.subscribe(  "/bldc_driver_node/Right_Motor_KP", 2, &BldcController_right::KP_Callback);  // 2 is the queue size
    sub_KI = nh_.subscribe(  "/bldc_driver_node/Right_Motor_KI", 2, &BldcController_right::KI_Callback);  // 2 is the queue size
    sub_KD = nh_.subscribe(  "/bldc_driver_node/Right_Motor_KD", 2, &BldcController_right::KD_Callback); // 2 is the queue size

    timeout_timer_ = nh_.createTimer(ros::Duration(2), &BldcController_right::timeoutCallback, this);
    timeout_timer_.start();
  }

  BldcController_right::~BldcController_right()
  {  }

  void BldcController_right::KP_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KP value is : [%d]", msg->data);
    motor_kp = msg->data;
    serial->Send_KP(motor_kp); // send kp to motor
  }
  void BldcController_right::KI_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KI value is : [%d]", msg->data);
    motor_ki = msg->data;
    serial->Send_KI(motor_ki); // send ki to motor
  }
  void BldcController_right::KD_Callback(const std_msgs::UInt16::ConstPtr& msg){
    ROS_INFO("Motor KD value is : [%d]", msg->data);
    motor_kd = msg->data;
    serial->Send_KD(motor_kd); // send kd to motor
  }

  void BldcController_right::motorCommand(const chip_bldc_driver::Command &command)
  {
    timeout_timer_.stop();
    timeout_timer_.start();
    serial->sendMotorCommand(command.motor_command);
  }

  void BldcController_right::timeoutCallback(const ros::TimerEvent &)
  {
    // In case the motorCommand function is not called the timeout will be called after 2 seconds to stop the motor.
    serial->sendMotorCommand(0);
  }

} // namespace bldc_controller