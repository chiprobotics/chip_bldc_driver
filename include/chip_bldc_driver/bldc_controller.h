#ifndef bldc_controller_H
#define bldc_controller_H

#include "ros/ros.h"
#include "chip_bldc_driver/bldc_serial.h"
#include "chip_bldc_driver/Command.h"

#include "std_msgs/String.h"
#include "std_msgs/UInt16.h"

namespace bldc_controller
{

/**
 * Class implementing bldc controller
 */
class BldcController
{
public:
  BldcController(bldc_serial::BldcSerial *s);
  ~BldcController();
  
private:
  uint16_t motor_kp;
  uint16_t motor_ki;
  uint16_t motor_kd;

  bldc_serial::BldcSerial *serial;
  ros::Timer timeout_timer_;
  ros::Subscriber sub_cmd_ , sub_KP , sub_KI , sub_KD;
  ros::NodeHandle nh_;
  void motorCommand(const chip_bldc_driver::Command& command);
  void timeoutCallback(const ros::TimerEvent&);
  
  void  KP_Callback(const std_msgs::UInt16::ConstPtr& msg);  // callback for motor KP
  void  KI_Callback(const std_msgs::UInt16::ConstPtr& msg); // callback for motor KI
  void  KD_Callback(const std_msgs::UInt16::ConstPtr& msg); // callback for motor KD
 
};

class BldcController_right
{
public:
  BldcController_right(bldc_serial::BldcSerial *s);
  ~BldcController_right();
  
private:
  uint16_t motor_kp;
  uint16_t motor_ki;
  uint16_t motor_kd;

  bldc_serial::BldcSerial *serial;
  ros::Timer timeout_timer_;
  ros::Subscriber sub_cmd_ , sub_KP , sub_KI , sub_KD ;
  ros::NodeHandle nh_;
  void motorCommand(const chip_bldc_driver::Command& command);
  void timeoutCallback(const ros::TimerEvent&);
 
};

}  // namespace bldc

#endif  // bldc
