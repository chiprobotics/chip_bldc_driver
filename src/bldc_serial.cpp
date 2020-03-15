/**
Software License Agreement BSD
\file      bldc_serial.cpp
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

#include "chip_bldc_driver/bldc_serial.h"
#include "chip_bldc_driver/crc16.h"

#include <sstream>

namespace bldc_serial
{

/**
  * Constructs
  */
BldcSerial::BldcSerial(const char *port, int baud) : port_(port),
                                                     baud_(baud),
                                                     serial_(nullptr),
                                                     nh_("~")
{
  pub_status_ = nh_.advertise<chip_bldc_driver::Status>("status", 1);
  pub_feedback_ = nh_.advertise<chip_bldc_driver::Feedback>("feedback", 1);
}

BldcSerial::~BldcSerial()
{
}

bool BldcSerial::connect()
{
  if (!serial_)
    serial_ = new serial::Serial();

  serial::Timeout to(serial::Timeout::simpleTimeout(500));
  serial_->setTimeout(to);
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);

  for (int n = 0; n < 10; n++)
  {
    try
    {
      serial_->open();
    }
    catch (serial::IOException)
    {
      ROS_WARN("serial IO Exception");
    }

    if (serial_->isOpen())
    {
      ROS_INFO("Serial connection successful on port:  %s", port_);
      return true;
    }
    else
    {
      ROS_WARN("Bad Connection with serial port Error %s, try number %i", port_, n);
    }
  }

  ROS_WARN("Unable to establish serial connection");
  return false;
}

void BldcSerial::read()
{
  const std::string eol("\n");
  const size_t max_line_length(128);
  std::string msg = serial_->readline(max_line_length, eol);
  if (!msg.empty())
  {
    size_t n = msg.size();
    if (n > 2)
    {
      char char_array[n + 1];
      strcpy(char_array, msg.c_str());

      if (msg[1] == 'f')
      {
        processFeedback(char_array, n);
      }
      else if (msg[1] == 's')
      {
        processStatus(char_array, n);
      }
      else
      {
        ROS_WARN("Unkown message type");
      }
    }
  }
}

void BldcSerial::write(const char *data_ptr, int32_t size)
{
  size_t bytes_written = serial_->write(data_ptr);
  ROS_DEBUG("Bytes written: %lu", bytes_written);
}

/**
   * Transmit motor speed command through serial
   * 
   * @param motor_speed_cmd Motor speed command value. Should be within the [-1000, 1000] range, otherwise it's ignored
   */
void BldcSerial::sendMotorCommand(int16_t motor_speed_cmd)
{
  constexpr int32_t MAX_CMD = 1000;
  constexpr int32_t MIN_CMD = -1000;
  constexpr int32_t MSG_SIZE = 16;

  if (motor_speed_cmd < MIN_CMD || motor_speed_cmd > MAX_CMD)
  {
    ROS_WARN("The motor speed command %i exceed the range [%i, %i].", motor_speed_cmd, MIN_CMD, MAX_CMD);
    return;
  }

  char message[MSG_SIZE];
  int32_t msg_len = sprintf(message, "$!MC:%d\r\n", motor_speed_cmd);

  write(message, msg_len);
}

/**
   * Processes the status message and prints it
   * @param data_ptr Pointer to an array of bytes. It should be a complete message data
   * @param size     Number of bytes
   */
void BldcSerial::processStatus(const char *data_ptr, int32_t size)
{
  enum SplitedValueIndexes
  {
    StartOfMsg = 0,
    OpStatus,
    MotorVoltage,
    Temperature,
    MotorCmd,
    PIDError,
    Crc16,
    N_VALUES
  };
  const std::string START_OF_MSG = "&s";

  std::vector<std::string> values(N_VALUES);
  chip_bldc_driver::Status msg;
  msg.header.stamp = ros::Time::now();
  if (extractData(values, data_ptr, size, START_OF_MSG, N_VALUES))
  {
    try
    {
      ROS_DEBUG("[BldcSerial::processStatus] Operation status: %s; Motor voltage: %s; Temperature: %s; Motor command: %s; PID error: %s",
                values[OpStatus].c_str(), values[MotorVoltage].c_str(), values[Temperature].c_str(), values[MotorCmd].c_str(), values[PIDError].c_str());

      msg.motor_operational_status = boost::lexical_cast<int>(values[OpStatus]);
      msg.battery_voltage = boost::lexical_cast<float>(values[MotorVoltage]) / 10.0;
      msg.mcu_temperature = boost::lexical_cast<int>(values[Temperature]);
      msg.motor_command = boost::lexical_cast<int>(values[MotorCmd]);
      msg.motor_pid_error = boost::lexical_cast<int>(values[PIDError]);
    }
    catch (std::bad_cast &e)
    {
      ROS_WARN("Error parsing status message");
      return;
    }

    pub_status_.publish(msg);
  }
}

/**
   * Processes the feedback message and prints it
   * @param data_ptr Pointer to an array of bytes. It should be a complete message data
   * @param size     Number of bytes
   */
void BldcSerial::processFeedback(const char *data_ptr, int32_t size)
{
  enum SplitedValueIndexes
  {
    StartOfMsg = 0,
    FeedbackCounter,
    EncoderCounter,
    MotorRPM,
    Crc16,
    N_VALUES
  };
  const std::string START_OF_MSG = "&f";

  std::vector<std::string> values(N_VALUES);

  chip_bldc_driver::Feedback msg;
  msg.header.stamp = ros::Time::now();

  if (extractData(values, data_ptr, size, START_OF_MSG, N_VALUES))
  {
    try
    {
      ROS_DEBUG("[BldcSerial::processFeedback] Feedback counter: %s; Encoder counter: %s; Motor RPM: %s",
                values[FeedbackCounter].c_str(), values[EncoderCounter].c_str(), values[MotorRPM].c_str());

      msg.motor_encoder_counter = boost::lexical_cast<int>(values[EncoderCounter]);
      msg.motor_rpm_feedback = boost::lexical_cast<int>(values[MotorRPM]);
    }
    catch (std::bad_cast &e)
    {
      ROS_WARN("Error parsing status message");
      return;
    }
    pub_feedback_.publish(msg);
  }
}

/**
   * Do verification and extraction of values from the data
   * @param  data_out Extracted values
   * @param  data     Input data
   * @param  size     Number of bytes of data
   * @param  som      Start of message symbols
   * @param  n_values Expected values in the message
   * @return          true if all the ckecks are passed, otherwise - false
   */
bool BldcSerial::extractData(std::vector<std::string> &data_out, const char *data, int32_t size, const std::string som, int32_t n_values)
{
  std::string str(data, size);

  // Chop end of line characheters \r\n
  str.pop_back();
  str.pop_back();

  // Split the message into tokens (values)
  std::vector<std::string> str_values = splitString(str, ':');

  // Check the start symbol
  if (!isExpectedStartOfMessage(str_values[0], som))
  {
    return false;
  }

  // Check the number of elements
  if (!isExpectedNumberOfValues(str_values.size(), n_values))
  {
    return false;
  }

  // Check the CRC (it covers only the start of a message and values, not CRC and end of line symbols);
  // -1 - is for exluding the ":" symbol right before received crc16
  int32_t size_for_crc = str.size() - str_values[n_values - 1].size() - 1;
  if (!isExpectedCrc(str.data(), size_for_crc, static_cast<uint16_t>(std::stoi(str_values[n_values - 1]))))
  {
    return false;
  }

  data_out.swap(str_values);
  return true;
}

/**
   * Split the string into tokens
   * @param s Input string
   * @param delimeter Delimeter symbol
   * @return Splited tokes
   */
std::vector<std::string> BldcSerial::splitString(const std::string &s, char delimeter)
{
  std::vector<std::string> tokens;
  std::string token;
  std::istringstream token_stream(s);
  while (std::getline(token_stream, token, delimeter))
  {
    tokens.push_back(token);
  }
  return tokens;
}

/**
   * Check if the start of message is equal to expected value
   * @param  som          Start of message
   * @param  expected_som Expected sthart of message
   * @return              true if som is equal expected, otherwise false
   */
bool BldcSerial::isExpectedStartOfMessage(const std::string &som, const std::string &expected_som)
{
  if (som == expected_som)
  {
    return true;
  }
  else
  {
    ROS_WARN("[BldcSerial] Incorrect the start of message. Expected %s, there was %s", expected_som.c_str(), som.c_str());
    return false;
  }
}

/**
   * Check if the number of values in a message is equal to expected value
   * @param  n_values          Number of values
   * @param  expected_n_values Expected number of values
   * @return                   True if it's equal, otherwise false
   */
bool BldcSerial::isExpectedNumberOfValues(int32_t n_values, int32_t expected_n_values)
{
  if (n_values == expected_n_values)
  {
    return true;
  }
  else
  {
    ROS_WARN("[BldcSerial] Incorrect number of values. Expected %i, there was %i", expected_n_values, n_values);
    return false;
  }
}

/**
   * Check CRC16 
   * @param  data         Data upon which the crc16 is calculated
   * @param  size         Number of bytes in data
   * @param  expected_crc Expected crc16 value
   * @return              True if crc values are equal, otherwise false
   */
bool BldcSerial::isExpectedCrc(const char *data, int32_t size, uint16_t expected_crc)
{
  uint16_t crc = crc_calc(0xFFFF, reinterpret_cast<const uint8_t *>(data), size);
  if (crc == expected_crc)
  {
    return true;
  }
  else
  {
    ROS_WARN("[BldcSerial] Incorrect CRC. Expected %i, but there was %i", expected_crc, crc);
    return true;
  }
}

} // namespace bldc_serial