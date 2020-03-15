#ifndef bldc_serial_H
#define bldc_serial_H

#include "ros/ros.h"
#include "serial/serial.h"
#include <string>
#include <vector>
#include "chip_bldc_driver/Status.h"
#include "chip_bldc_driver/Feedback.h"

namespace bldc_serial
{

/**
 * Class implementing bldc controller
 */
class BldcSerial
{
public:
  BldcSerial(const char *port, int baud);
  ~BldcSerial();
  bool connect();
  void read();
  void sendMotorCommand(int16_t motor_speed_cmd);

private:
  std::vector<std::string> splitString(const std::string& s, char delimeter);
  void write(const char *data_ptr, int32_t size);
  void processFeedback(const char *data_ptr, int32_t size);
  void processStatus(const char *data_ptr, int32_t size);
  bool isExpectedStartOfMessage(const std::string& som, const std::string& expected_som);
  bool isExpectedNumberOfValues(int32_t n_values, int32_t expected_n_values);
  bool isExpectedCrc(const char *data, int32_t size, uint16_t expected_crc);

  bool extractData(std::vector<std::string>& out, const char* data, int32_t size, const std::string som, int32_t n_values); 


private:
  const char *port_;
  int baud_;
  serial::Serial *serial_;
  ros::NodeHandle nh_;
  ros::Publisher pub_status_, pub_feedback_;
 
};

}  // namespace bldc serial

#endif  // bldc serial
