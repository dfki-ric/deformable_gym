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

#ifndef MIA_HAND_ROS_DRIVER_H
#define MIA_HAND_ROS_DRIVER_H

#include "mia_hand_driver/cpp_driver.h"
#include "ros/ros.h"

#include "mia_hand_msgs/FingersData.h"
#include "mia_hand_msgs/FingersStrainGauges.h"
#include "mia_hand_msgs/GraspRef.h"
#include "mia_hand_msgs/ConnectSerial.h"

#include "std_msgs/Int16.h"

#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"

namespace mia_hand
{
class ROSDriver
{
public:

  /**
   * Class constructor.
   * @param nh reference to public node handle
   * @param nh_priv reference to private node handle
   */
  ROSDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:

  /**
   * Cpp driver of the Mia hand.
   */
  CppDriver mia_;

  bool is_connected_;  //!< True if the Mia hand is connected, False otherwise.
  bool was_connected_; //!< True if the Mia hand was connected, False otherwise.

  ros::WallTimer publish_data_tmr_; //!< Timer to handle the Mia hand data publishing.
  ros::WallTimer check_connection_tmr_; //!< Timer to handle the check of the Mia hand connection status.

  ros::NodeHandle& nh_;       //!< Reference to public node handle (for topics
                              //!< and services)
  ros::NodeHandle& nh_priv_;  //!< Reference to private node handle (for
                              //!< parameters)

  /* Subscribers
   */
  ros::Subscriber thu_mot_trgt_pos_; //!< Subscriber to receive the target pose of the tumb flexion motor.
  ros::Subscriber ind_mot_trgt_pos_; //!< Subscriber to receive the target pose of the index flexion motor.
  ros::Subscriber mrl_mot_trgt_pos_; //!< Subscriber to receive the target pose of the mrl flexion motor.

  ros::Subscriber thu_mot_trgt_spe_; //!< Subscriber to receive the target velocity of the tumb flexion motor.
  ros::Subscriber ind_mot_trgt_spe_; //!< Subscriber to receive the target velocity of the index flexion motor.
  ros::Subscriber mrl_mot_trgt_spe_; //!< Subscriber to receive the target velocity of the mrl flexion motor.

  ros::Subscriber thu_fin_trgt_for_; //!< Subscriber to receive the target force of the tumb flexion motor.
  ros::Subscriber ind_fin_trgt_for_; //!< Subscriber to receive the target force of the index flexion motor.
  ros::Subscriber mrl_fin_trgt_for_; //!< Subscriber to receive the target force of the mrl flexion motor.

  ros::Subscriber thu_cyl_grasp_ref_; //!< Subscriber to receive the parametrs for cylindrical grasp for the thumb flexion motor.
  ros::Subscriber ind_cyl_grasp_ref_; //!< Subscriber to receive the parametrs for cylindrical grasp for the index flexion motor.
  ros::Subscriber mrl_cyl_grasp_ref_; //!< Subscriber to receive the parametrs for cylindrical grasp for the mrl flexion motor.

  ros::Subscriber thu_pin_grasp_ref_; //!< Subscriber to receive the parametrs for pinch grasp for the thumb flexion motor.
  ros::Subscriber ind_pin_grasp_ref_; //!< Subscriber to receive the parametrs for pinch grasp for the index flexion motor.
  ros::Subscriber mrl_pin_grasp_ref_; //!< Subscriber to receive the parametrs for pinch grasp for the mrl flexion motor.

  ros::Subscriber thu_lat_grasp_ref_; //!< Subscriber to receive the parametrs for lateral grasp for the thumb flexion motor.
  ros::Subscriber ind_lat_grasp_ref_; //!< Subscriber to receive the parametrs for lateral grasp for the index flexion motor.
  ros::Subscriber mrl_lat_grasp_ref_; //!< Subscriber to receive the parametrs for lateral grasp for the mrl flexion motor.

  ros::Subscriber thu_sph_grasp_ref_; //!< Subscriber to receive the parametrs for spherical grasp for the thumb flexion motor.
  ros::Subscriber ind_sph_grasp_ref_; //!< Subscriber to receive the parametrs for spherical grasp for the index flexion motor.
  ros::Subscriber mrl_sph_grasp_ref_; //!< Subscriber to receive the parametrs for spherical grasp for the mrl flexion motor.

  ros::Subscriber thu_tri_grasp_ref_; //!< Subscriber to receive the parametrs for tridigital grasp for the thumb flexion motor.
  ros::Subscriber ind_tri_grasp_ref_; //!< Subscriber to receive the parametrs for tridigital grasp for the index flexion motor.
  ros::Subscriber mrl_tri_grasp_ref_; //!< Subscriber to receive the parametrs for tridigital grasp for the mrl flexion motor.

  ros::Subscriber cyl_grasp_percent_; //!< Subscriber to receive the closure percentage for cylindical grasp.
  ros::Subscriber pin_grasp_percent_; //!< Subscriber to receive the closure percentage for pinch grasp.
  ros::Subscriber lat_grasp_percent_; //!< Subscriber to receive the closure percentage for lateral grasp.
  ros::Subscriber sph_grasp_percent_; //!< Subscriber to receive the closure percentage for sherical grasp.
  ros::Subscriber tri_grasp_percent_; //!< Subscriber to receive the closure percentage for tridigital grasp.

  /* Publishers
   */
  ros::Publisher mot_pos_info_;  //!< Publisher of the positions data of the Mia hand motors.
  ros::Publisher mot_spe_info_; //!< Publisher of the velocity data of the Mia hand motors.
  ros::Publisher mot_cur_info_; //!< Publisher of the current absorbed by the Mia hand motors.
  ros::Publisher fin_for_info_; //!< Publisher of the force data read by the sensors embedded in the Mia hand fingers.

  /* Services
   */
  ros::ServiceServer connect_to_port_; //!< Service to open a serial port and connect the Mia hand.
  ros::ServiceServer disconnect_;     //!< Service to disconnect the Mia hand and close the serial port.

  ros::ServiceServer switch_pos_stream_; //!< Service to manage a the streaming of motor position data sent by the Mia hand.
  ros::ServiceServer switch_spe_stream_; //!< Service to manage a the streaming of motor velocity data sent by the Mia hand.
  ros::ServiceServer switch_ana_stream_; //!< Service to manage a the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
  ros::ServiceServer switch_cur_stream_; //!< Service to manage a the streaming of the current data (i.e. force sensor outputs) sent by the Mia hand.

  ros::ServiceServer open_cyl_grasp_; //!< Service to move the Mia hand motors to the rest position of the cylindrical grasp.
  ros::ServiceServer open_pin_grasp_; //!< Service to move the Mia hand motors to the rest position of the pinch grasp.
  ros::ServiceServer open_lat_grasp_; //!< Service to move the Mia hand motors to the rest position of the lateral grasp.
  ros::ServiceServer open_sph_grasp_; //!< Service to move the Mia hand motors to the rest position of the spherical grasp.
  ros::ServiceServer open_tri_grasp_; //!< Service to move the Mia hand motors to the rest position of the tridigital grasp.

  ros::ServiceServer close_cyl_grasp_; //!< Service to move the Mia hand motors to the closed position of the cylindrical grasp.
  ros::ServiceServer close_pin_grasp_; //!< Service to move the Mia hand motors to the closed position of the pinch grasp.
  ros::ServiceServer close_lat_grasp_; //!< Service to move the Mia hand motors to the closed position of the lateral grasp.
  ros::ServiceServer close_sph_grasp_; //!< Service to move the Mia hand motors to the closed position of the spherical grasp.
  ros::ServiceServer close_tri_grasp_; //!< Service to move the Mia hand motors to the closed position of the tridigital grasp.

  void publishDataTmrCallback(const ros::WallTimerEvent& event); //!< Callback of the #publish_data_tmr_ timer.
  void checkConnectionTmrCallback(const ros::WallTimerEvent& event); //!< Callback of the #check_connection_tmr_ timer.

  void initPublishers();     //!< Initialize publishers.
  void initSubscribersThu(); //!< Initialize subscribers of the thumb flexion motor info.
  void initSubscribersInd(); //!< Initialize subscribers of the index flexion motor info.
  void initSubscribersMrl(); //!< Initialize subscribers of the index mrl motor info.
  void initSubscribersGrasp(); //!< Initialize subscribers of the automatic grasp info.
  void initServices(); //!< Initialize services.

  /* Subscribers Callback Functions
   */

  /**
   * Callback of the subscriber #thu_mot_trgt_pos_.
   * @param msg standard message containing the target position [0, 255] of the motor.
   */
  void thuMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_mot_trgt_pos_.
   * @param msg standard message containing the target position [-255, 255] of the motor
   */
  void indMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_mot_trgt_pos_.
   * @param msg standard message containing the target position [0, 255] of the motor.
   */
  void mrlMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg);



  /**
   * Callback of the subscriber #thu_mot_trgt_spe_.
   * @param msg standard message containing the target velocity [-90, 90] of the motor.
   */
  void thuMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_mot_trgt_spe_.
   * @param msg standard message containing the target velocity [-90, 90] of the motor.
   */
  void indMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_mot_trgt_spe_.
   * @param msg standard message containing the target velocity [-90, 90] of the motor.
   */
  void mrlMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg);


  /**
   * Callback of the subscriber #thu_fin_trgt_for_.
   * @param msg standard message containing the target force [0, +1024] to be applied by
   * the finger (and thus to be read by the finger sensor).
   */
  void thuFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_fin_trgt_for_.
   * @param msg standard message containing the target force [0, +1024] to be applied by
   * the finger (and thus to be read by the finger sensor).
   */
  void indFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_fin_trgt_for_.
   * @param msg standard message containing the target force [0, +1024] to be applied by
   * the finger (and thus to be read by the finger sensor).
   */
  void mrlFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #thu_cyl_grasp_ref_.
   * Set the parameter of the cylindrical grasp for the thumb flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void thuCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #thu_pin_grasp_ref_.
   * Set the parameter of the pinch grasp for the thumb flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void thuPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #thu_lat_grasp_ref_.
   * Set the parameter of the lateral grasp for the thumb flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void thuLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #thu_sph_grasp_ref_.
   * Set the parameter of the spherical grasp for the thumb flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void thuSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #thu_sph_grasp_ref_.
   * Set the parameter of the tridigital grasp for the thumb flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void thuTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);


  /**
   * Callback of the subscriber #ind_cyl_grasp_ref_.
   * Set the parameter of the cylindrical grasp for the index flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void indCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_pin_grasp_ref_.
   * Set the parameter of the pinch grasp for the index flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void indPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_lat_grasp_ref_.
   * Set the parameter of the lateral grasp for the index flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void indLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_sph_grasp_ref_.
   * Set the parameter of the spherical grasp for the index flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void indSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #ind_tri_grasp_ref_.
   * Set the parameter of the tridigital grasp for the index flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void indTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);


  /**
   * Callback of the subscriber #mrl_cyl_grasp_ref.
   * Set the parameter of the cylindrical grasp for the mrl flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void mrlCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_pin_grasp_ref.
   * Set the parameter of the pinch grasp for the mrl flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void mrlPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_lat_grasp_ref.
   * Set the parameter of the lateral grasp for the mrl flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void mrlLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_sph_grasp_ref.
   * Set the parameter of the spherical grasp for the mrl flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters ( rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void mrlSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);

  /**
   * Callback of the subscriber #mrl_tri_grasp_ref.
   * Set the parameter of the tridigital grasp for the mrl flexion motor.
   * @param msg mia_hand_msgs containing the grasp parameters (rest position,
   * closure position and dely) to be set for the specific motor.
   */
  void mrlTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr& msg);


  /**
   * Callback of the subscriber #cyl_grasp_percent_.
   * Set the target closure percentage of the cylindrical grasp.
   * @param msg standard message containing the target closure percentage [0, 100].
   */
  void cylGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #pin_grasp_percent_.
   * Set the target closure percentage of the pinch grasp.
   * @param msg standard message containing the target closure percentage [0, 100].
   */
  void pinGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #lat_grasp_percent_.
   * Set the target closure percentage of the lateral grasp.
   * @param msg standard message containing the target closure percentage [0, 100].
   */
  void latGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #sph_grasp_percent_.
   * Set the target closure percentage of the spherical grasp.
   * @param msg standard message containing the target closure percentage [0, 100].
   */
  void sphGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg);

  /**
   * Callback of the subscriber #tri_grasp_percent_.
   * Set the target closure percentage of the tridigital grasp.
   * @param msg standard message containing the target closure percentage [0, 100].
   */
  void triGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg);

  /* Services Callback Functions
   */

   /**
    * Callback of the service #connect_to_port_.
    * Connect the Mia hand opening a serial port.
    * @param req mia_hand_msgs containing the number of the port to open.
    * @param resp mia_hand_msgs containing the success of the operation (true or false)
    * and a string message.
    */
  bool connectToPortCallback(mia_hand_msgs::ConnectSerial::Request& req,
                             mia_hand_msgs::ConnectSerial::Response& resp);

   /**
    * Callback of the service #disconnect_.
    * Disconnect the Mia hand and close the serial port.
    * @param req unused.
    * @param resp containing the success of the operation (true or false)
    * and a string message.
    */
  bool disconnectCallback(std_srvs::Trigger::Request& req,
                          std_srvs::Trigger::Response& resp);

  /**
   * Callback of the service #switch_pos_stream_.
   * Manage the streaming of motor position data sent by the Mia hand.
   * @param req True to enable the data streaming, False to disable it.
   * @param resp Unused
   */
  bool switchPosStreamCallback(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& resp);


   /**
    * Callback of the service #switch_spe_stream_.
    * Manage the streaming of motor velocity data sent by the Mia hand.
    * @param req True to enable the data streaming, False to disable it.
    * @param resp Unused
    */
  bool switchSpeStreamCallback(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& resp);

   /**
    * Callback of the service #switch_ana_stream_.
    * Manage the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
    * @param req True to enable the data streaming, False to disable it.
    * @param resp Unused
    */
  bool switchAnaStreamCallback(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& resp);

   /**
    * Callback of the service #switch_cur_stream_.
    * Manage the streaming of the analog input data (i.e. force sensor outputs) sent by the Mia hand.
    * @param req True to enable the data streaming, False to disable it.
    * @param resp Unused
    */
  bool switchCurStreamCallback(std_srvs::SetBool::Request& req,
                               std_srvs::SetBool::Response& resp);

   /**
    * Callback of the service #open_cyl_grasp_.
    * Move the Mia hand motors to the rest position of the cylindrical grasp.
    * @param req Unused.
    * @param resp Unused
    */
  bool openCylGraspCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& resp);

  /**
   * Callback of the service #open_pin_grasp_.
   * Move the Mia hand motors to the rest position of the pinch grasp.
   * @param req Unused.
   * @param resp Unused
   */
  bool openPinGraspCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& resp);

  /**
   * Callback of the service #open_lat_grasp_.
   * Move the Mia hand motors to the rest position of the lateral grasp.
   * @param req Unused.
   * @param resp Unused
   */
  bool openLatGraspCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& resp);

  /**
   * Callback of the service #open_sph_grasp_.
   * Move the Mia hand motors to the rest position of the spherical grasp.
   * @param req Unused.
   * @param resp Unused
   */
  bool openSphGraspCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& resp);

  /**
   * Callback of the service #open_tri_grasp_.
   * Move the Mia hand motors to the rest position of the tridigital grasp.
   * @param req Unused.
   * @param resp Unused
   */
  bool openTriGraspCallback(std_srvs::Empty::Request& req,
                            std_srvs::Empty::Response& resp);

  /**
   * Callback of the service #close_cyl_grasp_.
   * Move the Mia hand motors to the closed position of the cylindrical grasp.
   * @param req Unused.
   * @param resp Unused
   */
  bool closeCylGraspCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);

   /**
    * Callback of the service #close_pin_grasp_.
    * Move the Mia hand motors to the closed position of the pinch grasp.
    * @param req Unused.
    * @param resp Unused
    */
  bool closePinGraspCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);

   /**
    * Callback of the service #close_lat_grasp_.
    * Move the Mia hand motors to the closed position of the lateral grasp.
    * @param req Unused.
    * @param resp Unused
    */
  bool closeLatGraspCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);

   /**
    * Callback of the service #close_sph_grasp_.
    * Move the Mia hand motors to the closed position of the spherical grasp.
    * @param req Unused.
    * @param resp Unused
    */
  bool closeSphGraspCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);

   /**
    * Callback of the service #close_tri_grasp_.
    * Move the Mia hand motors to the closed position of the tridigital grasp.
    * @param req Unused.
    * @param resp Unused
    */
  bool closeTriGraspCallback(std_srvs::Empty::Request& req,
                             std_srvs::Empty::Response& resp);
};
}  // namespace

#endif  // MIA_HAND_ROS_DRIVER_H
