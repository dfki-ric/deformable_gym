/*
 * Copyright (C) 2021 Prensilia s.r.l.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MIA_HAND_SERIAL_PORT_H
#define MIA_HAND_SERIAL_PORT_H

#include <libserial/SerialPort.h>
#include <mutex>

namespace mia_hand
{

/**
 * Struct containing all info that could regard a Mia hand motor.
 */
typedef struct FingerSerialInfo
{

  FingerSerialInfo();

  int16_t mot_pos;
  int8_t mot_spe;
  int16_t mot_cur;
  int16_t fin_sg_raw[2];
} FingerSerialInfo;

/**
 * Class to handle a serial port and its serial communication protocol.
 */
class SerialPort: public LibSerial::SerialPort
{
public:

  /**
   * Class destructor.
   */
  SerialPort(std::mutex* p_finger_data_mtx, std::mutex* p_connection_mtx,
             bool* p_is_connected);

 /**
  * Class destructor.
  */
  ~SerialPort();

  /**
   * Open a serial port.
   * @param port_num integer number of the serial port to open.
   */
  bool open(uint16_t port_num);

  /**
   * Close the serial port.
   */
  bool close();

  /**
   * Send a command to the Mia hand attached to the serial port.
   * This commands add to the input message the proper tail and send the composed message to the Mia hand.
   * @param command command to be sent.
   */
  void sendCommand(const std::string& command);

  /**
   * Parse message received from the Mia hand.
   * @param thumb Received info regarding the thumb flexion motor.
   * @param index Received info regarding the index flexion motor.
   * @param mrl Received info regarding the mrl flexion motor.
   * @param is_checking_on boolean to handle the check of the Mia hand connection.
   */
  void parseStream(FingerSerialInfo& thumb, FingerSerialInfo& index,
                   FingerSerialInfo& mrl, bool& is_checking_on);

private:

  std::string stream_msg_; //!< Message received from the Mia hand.

  std::mutex serial_write_mtx_;
  std::mutex* p_finger_data_mtx_;
  std::mutex* p_connection_mtx_;

  bool* p_is_connected_;
};
}  // namespace

#endif  // MIA_HAND_SERIAL_PORT_H
