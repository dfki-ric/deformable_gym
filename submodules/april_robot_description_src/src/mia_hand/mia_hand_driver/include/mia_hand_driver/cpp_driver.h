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

#ifndef MIA_HAND_CPP_DRIVER_H
#define MIA_HAND_CPP_DRIVER_H

#include "mia_hand_driver/serial_port.h"
#include <thread>

namespace mia_hand
{
class CppDriver
{
public:

  /**
   * Class constructor.
   */
  CppDriver();

  /**
   *Class destructor.
   */
  ~CppDriver();

  /**
   * Open serial port used to plug Mia hand.
   * @param port_num number of the serial port to open.
   * @return True if the port has been opened, False otherwise.
  */
  bool connectToPort(uint16_t port_num);

  /**
   * Disconnect the Mia hand closing the serial port
   * @return True if the serial port has been closed, False otherwise.
  */
  bool disconnect();

  /**
   * Function returning the connection status of the Mia hand.
   * @return True if the Mia hand is attached to the sertial port, False otherwise.
  */
  bool isConnected();

  /**
   * Set the target position of a specific motor of the Mia hand.
   * This function can be used to move specific Mia hand finger in position.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @param mot_pos target finger position [-255, +255] for index and [0, +255] otherwise.
  */
  void setMotorPos(uint8_t fin_id, int16_t mot_pos);

  /**
   * Set the target velocity of a specific motor of the Mia hand.
   * This function can be used to move a specific Mia hand finger in velocity.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @param mot_spe target finger velocity [-90, +90].
  */
  void setMotorSpe(uint8_t fin_id, int16_t mot_spe);

  /**
   * Set the target force of a specific motor of the Mia hand.
   * This function can be used to move specific Mia hand finger with force control.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @param fin_for target finger velocity [0, +1024]
  */
  void setFingerFor(uint8_t fin_id, int16_t fin_for);

  /**
   * Get the current position of a specific motor of the Mia hand.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @return position of the target motor as [-255, +255] for index motor and [0, 255] otherwise.
  */
  int16_t getMotorPos(uint8_t fin_id);

  /**
   * Get the current velocity of a specific motor of the Mia hand.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @return position of the target motor as [-90, +90].
  */
  int16_t getMotorSpe(uint8_t fin_id);

  /**
   * Get the current currently absorbed by a specific motor of the Mia hand.
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @return position of the target motor as [0, +1024].
  */
  int16_t getMotorCur(uint8_t fin_id);

  /**
   * Get the current output of the force sensor of a specific finger of the Mia hand.
   * Get the normal and tangential force signal output of the strain gauge
   * included in the finger of the Mia hand
   * @param fin_id Mia hand finger id. 1 for thumb flexion, 2 for MRL flexion, 3 for index flexion + thumb abduction.
   * @param nor_for sensor output describing the normal force applied on the finger.
   * @param tan_for sensor output describing the tangential force applied on the finger.
  */
  void getFingerSgRaw(uint8_t fin_id, int16_t& nor_for, int16_t& tan_for);

  /**
   * Go to rest pose of a grasp type
   * Move all the Mia hand fingers to the rest pose of a specific grasp type.
   * @param grasp_id Type of grasp: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
  */
  void openGrasp(char grasp_id);

  /**
   * Go to closed pose of a grasp type
   * Move all the Mia hand fingers to the closed pose of a specific grasp type.
   * @param grasp_id Type of grasp: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
  */
  void closeGrasp(char grasp_id);

  /**
   * Go to intermediate pose of a grasp type
   * Move all the Mia hand fingers to a specific intermediate pose of a specific grasp type.
   * @param grasp_id Type of grasp: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
   * @param close_percent Percentage of closure [0, 100].
  */
  void closeGrasp(char grasp_id, int16_t close_percent);

  /**
   * Set grasp parameter for thumb flexion motor.
   * @param grasp_id Type of grasp to set: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
   * @param rest Rest reference position [0, 255].
   * @param pos Closure reference position [0, 255].
   * @param delay Time delay in milliseconds. Used to delay the movement of the target motor of a percentage of the total time needed to perform the grasp.
  */
  void setThuGraspRef(char grasp_id, int16_t rest, int16_t pos, int16_t delay);

  /**
   * Set grasp parameter for Index flexion + thumb abduction motor.
   * @param grasp_id Type of grasp to set: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
   * @param rest Rest reference position [0, 255].
   * @param pos Closure reference position [0, 255].
   * @param delay Time delay in milliseconds. Used to delay the movement of the target motor of a percentage of the total time needed to perform the grasp.
  */
  void setIndGraspRef(char grasp_id, int16_t rest, int16_t pos, int16_t delay);

  /**
   * Set grasp parameter for MRL flexion motor.
   * @param grasp_id Type of grasp to set: 'C' cylindical, 'L' lateral, 'P' pinch, 'S' spherical, 'T' tridigital grasp.
   * @param rest Rest reference position [0, 255].
   * @param pos Closure reference position [0, 255].
   * @param delay Time delay in milliseconds. Used to delay the movement of the target motor of a percentage of the total time needed to perform the grasp.
  */
  void setMrlGraspRef(char grasp_id, int16_t rest, int16_t pos, int16_t delay);

  /**
   * Manage the streaming of the motors position data.
   * @param b_on_off True to enable the streaming of the motors position data, False to disable it.
  */
  void switchPosStream(bool b_on_off);

  /**
   * Manage the streaming of the motors velocity data.
   * @param b_on_off True to enable the streaming, False to disable it.
  */
  void switchSpeStream(bool b_on_off);

  /**
   * Manage the streaming of analog input data (as the straing gauge force sensors).
   * @param b_on_off True to enable the streaming, False to disable it.
  */
  void switchAnaStream(bool b_on_off);

  /**
   * Manage the streaming of the current absorbed by Mi hand motors.
   * @param b_on_off True to enable the streaming, False to disable it.
  */
  void switchCurStream(bool b_on_off);

private:

  SerialPort serial_port_; //!< Hanlder of the serial port_num.

  /**
   * Convert integer number into string.
   * @param num Integer number to be converted.
   * @param n_digits Number of digits of the integer to be converted.
   * @return string of the input integer.
  */
  std::string numToStr(int16_t num, int8_t n_digits);

  /**
   * Function of the serial_poll_trd_ thread.
   * While the serial_poll_trd_ thread is active, parse the data received from
   * the Mia hand.
   * @see #serial_poll_trd_
  */
  void pollSerialPort();

  /**
   * Function of the check_connection_trd_ thread.
   * While the check_connection_trd_ thread is active, check if the Mia hand is connected.
   * @see #check_connection_trd_
  */
  void checkConnection();

  /**
   * Thread to execute the parse of the data received by the Mia hand.
   * @see #pollSerialPort()
  */
  std::thread serial_poll_trd_;

  bool serial_trd_on_;//!< True if #serial_poll_trd_ thread is running.
  std::mutex finger_data_mtx_;//!< Mutex to handle the reading of the motors finger data.


  /**
   * Thread to execute the check of the Mia hand connection.
   * @see #checkConnection()
  */
  std::thread check_connection_trd_;

  bool connection_trd_on_; //!< True if #check_connection_trd_ thread is running.
  bool is_checking_on_;

  std::mutex reply_mtx_;
  std::mutex connection_mtx_;

  FingerSerialInfo thumb_info_; //!< Info about the thumb flexion motor.
  FingerSerialInfo index_info_; //!< Info about the index flexion motor.
  FingerSerialInfo mrl_info_;   //!< Info about the mrl flexion motor.

  bool is_connected_; //!< True if the Mia hand is conncted, False otherwise.
};
}  // namespace

#endif  // MIA_HAND_CPP_DRIVER_H
