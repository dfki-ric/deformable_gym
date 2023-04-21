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

#include "mia_hand_driver/ros_driver.h"
#include <iostream>

namespace mia_hand
{
ROSDriver::ROSDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_priv):
  nh_(nh),
  nh_priv_(nh_priv),
  is_connected_(false),
  was_connected_(false)
{
  uint16_t port_num;

  ROS_INFO("Please specify Mia Hand port number: ");
  std::cin >> port_num;

  bool port_opened = mia_.connectToPort(port_num);

  if (port_opened)
  {
    std::string info_msg = "/dev/ttyUSB successfully opened.";
    info_msg.insert(11, std::to_string(port_num));

    ROS_INFO("%s\n", info_msg.c_str());

    initPublishers();
    initSubscribersThu();
    initSubscribersInd();
    initSubscribersMrl();
    initSubscribersGrasp();
    initServices();

    publish_data_tmr_ = nh_.createWallTimer(ros::WallDuration(0.1),
                                      &ROSDriver::publishDataTmrCallback,
                                      this);

    check_connection_tmr_ = nh_.createWallTimer(ros::WallDuration(1),
                                      &ROSDriver::checkConnectionTmrCallback,
                                      this);
  }
  else
  {
    ROS_ERROR("Could not open specified serial port.");
  }
}

void ROSDriver::publishDataTmrCallback(const ros::WallTimerEvent& event)
{
  mia_hand_msgs::FingersData msg;

  msg.thu = mia_.getMotorPos(0);
  msg.ind = mia_.getMotorPos(2);
  msg.mrl = mia_.getMotorPos(1);

  mot_pos_info_.publish(msg);

  msg.thu = mia_.getMotorSpe(0);
  msg.ind = mia_.getMotorSpe(2);
  msg.mrl = mia_.getMotorSpe(1);

  mot_spe_info_.publish(msg);

  msg.thu = mia_.getMotorCur(0);
  msg.ind = mia_.getMotorCur(2);
  msg.mrl = mia_.getMotorCur(1);

  mot_cur_info_.publish(msg);

  mia_hand_msgs::FingersStrainGauges msg_sg;

  mia_.getFingerSgRaw(0, msg_sg.thu[0], msg_sg.thu[1]);
  mia_.getFingerSgRaw(2, msg_sg.ind[0], msg_sg.ind[1]);
  mia_.getFingerSgRaw(1, msg_sg.mrl[0], msg_sg.mrl[1]);

  fin_for_info_.publish(msg_sg);

  return;
}

void ROSDriver::checkConnectionTmrCallback(const ros::WallTimerEvent& event)
{
  is_connected_ = mia_.isConnected();

  if (is_connected_ && !was_connected_)
  {
    was_connected_ = true;
    ROS_INFO("Mia Hand connected.");
  }
  else if (!is_connected_ && was_connected_)
  {
    was_connected_ = false;
    ROS_INFO("Mia Hand disconnected.");
  }
  else
  {
    // Default case: keep program running.
  }

  return;
}

void ROSDriver::initSubscribersThu()
{
	thu_mot_trgt_pos_ = nh_.subscribe("thumb_mot_trgt_pos", 1000,
                                    &ROSDriver::thuMotTrgtPosCallback, this);

	thu_mot_trgt_spe_ = nh_.subscribe("thumb_mot_trgt_spe", 1000,
                                    &ROSDriver::thuMotTrgtSpeCallback, this);

	thu_fin_trgt_for_ = nh_.subscribe("thumb_fin_trgt_for", 1000,
                                    &ROSDriver::thuFinTrgtForCallback, this);      

	thu_cyl_grasp_ref_ = nh_.subscribe("thumb_cyl_grasp_ref", 1000,
                                     &ROSDriver::thuCylGraspRefCallback, this);

	thu_pin_grasp_ref_ = nh_.subscribe("thumb_pin_grasp_ref", 1000,
                                     &ROSDriver::thuPinGraspRefCallback, this);

	thu_lat_grasp_ref_ = nh_.subscribe("thumb_lat_grasp_ref", 1000,
                                     &ROSDriver::thuLatGraspRefCallback, this);

	thu_sph_grasp_ref_ = nh_.subscribe("thumb_sph_grasp_ref", 1000,
                                     &ROSDriver::thuSphGraspRefCallback, this);

	thu_tri_grasp_ref_ = nh_.subscribe("thumb_tri_grasp_ref", 1000,
                                     &ROSDriver::thuTriGraspRefCallback, this);

  return;
}

void ROSDriver::initSubscribersInd()
{
	ind_mot_trgt_pos_ = nh_.subscribe("index_mot_trgt_pos", 1000,
                                    &ROSDriver::indMotTrgtPosCallback, this);

	ind_mot_trgt_spe_ = nh_.subscribe("index_mot_trgt_spe", 1000,
                                    &ROSDriver::indMotTrgtSpeCallback, this);

	ind_fin_trgt_for_ = nh_.subscribe("index_fin_trgt_for", 1000,
                                    &ROSDriver::indFinTrgtForCallback, this);

	ind_cyl_grasp_ref_ = nh_.subscribe("index_cyl_grasp_ref", 1000,
                                     &ROSDriver::indCylGraspRefCallback, this);

	ind_pin_grasp_ref_ = nh_.subscribe("index_pin_grasp_ref", 1000,
                                     &ROSDriver::indPinGraspRefCallback, this);

	ind_lat_grasp_ref_ = nh_.subscribe("index_lat_grasp_ref", 1000,
                                     &ROSDriver::indLatGraspRefCallback, this);

	ind_sph_grasp_ref_ = nh_.subscribe("index_sph_grasp_ref", 1000,
                                     &ROSDriver::indSphGraspRefCallback, this);

	ind_tri_grasp_ref_ = nh_.subscribe("index_tri_grasp_ref", 1000,
                                     &ROSDriver::indTriGraspRefCallback, this);

  return;
}

void ROSDriver::initSubscribersMrl()
{
	mrl_mot_trgt_pos_ = nh_.subscribe("mrl_mot_trgt_pos", 1000,
                                    &ROSDriver::mrlMotTrgtPosCallback, this);

	mrl_mot_trgt_spe_ = nh_.subscribe("mrl_mot_trgt_spe", 1000,
                                    &ROSDriver::mrlMotTrgtSpeCallback, this);

	mrl_fin_trgt_for_ = nh_.subscribe("mrl_fin_trgt_for", 1000,
                                    &ROSDriver::mrlFinTrgtForCallback, this);

	mrl_cyl_grasp_ref_ = nh_.subscribe("mrl_cyl_grasp_ref", 1000,
                                     &ROSDriver::mrlCylGraspRefCallback, this);

	mrl_pin_grasp_ref_ = nh_.subscribe("mrl_pin_grasp_ref", 1000,
                                     &ROSDriver::mrlPinGraspRefCallback, this);

	mrl_lat_grasp_ref_ = nh_.subscribe("mrl_lat_grasp_ref", 1000,
                                     &ROSDriver::mrlLatGraspRefCallback, this);

	mrl_sph_grasp_ref_ = nh_.subscribe("mrl_sph_grasp_ref", 1000,
                                     &ROSDriver::mrlSphGraspRefCallback, this);

	mrl_tri_grasp_ref_ = nh_.subscribe("mrl_tri_grasp_ref", 1000,
                                     &ROSDriver::mrlTriGraspRefCallback, this);

  return;
}

void ROSDriver::initSubscribersGrasp()
{
  cyl_grasp_percent_ = nh_.subscribe("cyl_grasp_percent", 1000,
                                     &ROSDriver::cylGraspPercentCallback, this);

  pin_grasp_percent_ = nh_.subscribe("pin_grasp_percent", 1000,
                                     &ROSDriver::pinGraspPercentCallback, this);

  lat_grasp_percent_ = nh_.subscribe("lat_grasp_percent", 1000,
                                     &ROSDriver::latGraspPercentCallback, this);

  sph_grasp_percent_ = nh_.subscribe("sph_grasp_percent", 1000,
                                     &ROSDriver::sphGraspPercentCallback, this);

  tri_grasp_percent_ = nh_.subscribe("tri_grasp_percent", 1000,
                                     &ROSDriver::triGraspPercentCallback, this);

  return;
}

void ROSDriver::initServices()
{      
	connect_to_port_ = nh_.advertiseService("connect_to_port",
                                          &ROSDriver::connectToPortCallback,
                                          this);

	disconnect_ = nh_.advertiseService("disconnect",
                                     &ROSDriver::disconnectCallback, this);
        
	switch_pos_stream_ = nh_.advertiseService("switch_pos_stream",
                                            &ROSDriver::switchPosStreamCallback,
                                            this);   

	switch_spe_stream_ = nh_.advertiseService("switch_spe_stream",
                                            &ROSDriver::switchSpeStreamCallback,
                                            this);

	switch_ana_stream_ = nh_.advertiseService("switch_ana_stream",
                                            &ROSDriver::switchAnaStreamCallback,
                                            this);

	switch_cur_stream_ = nh_.advertiseService("switch_cur_stream",
                                            &ROSDriver::switchCurStreamCallback,
                                            this);

	open_cyl_grasp_ = nh_.advertiseService("open_cyl_grasp",
                                         &ROSDriver::openCylGraspCallback,
                                         this);

	open_pin_grasp_ = nh_.advertiseService("open_pin_grasp",
                                         &ROSDriver::openPinGraspCallback,
                                         this);

	open_lat_grasp_ = nh_.advertiseService("open_lat_grasp",
                                         &ROSDriver::openLatGraspCallback,
                                         this);

	open_sph_grasp_ = nh_.advertiseService("open_sph_grasp",
                                         &ROSDriver::openSphGraspCallback,
                                         this);

	open_tri_grasp_ = nh_.advertiseService("open_tri_grasp",
                                         &ROSDriver::openTriGraspCallback,
                                         this);

	close_cyl_grasp_ = nh_.advertiseService("close_cyl_grasp",
                                          &ROSDriver::closeCylGraspCallback,
                                          this);

	close_pin_grasp_ = nh_.advertiseService("close_pin_grasp",
                                          &ROSDriver::closePinGraspCallback,
                                          this);

	close_lat_grasp_ = nh_.advertiseService("close_lat_grasp",
                                          &ROSDriver::closeLatGraspCallback,
                                          this);

	close_sph_grasp_ = nh_.advertiseService("close_sph_grasp",
                                          &ROSDriver::closeSphGraspCallback,
                                          this);

	close_tri_grasp_ = nh_.advertiseService("close_tri_grasp",
                                          &ROSDriver::closeTriGraspCallback,
                                          this);

  return;
}

void ROSDriver::initPublishers()
{
	mot_pos_info_ = nh_.advertise<mia_hand_msgs::FingersData>("mot_pos", 1000);
	mot_spe_info_ = nh_.advertise<mia_hand_msgs::FingersData>("mot_spe", 1000);
	mot_cur_info_ = nh_.advertise<mia_hand_msgs::FingersData>("mot_cur", 1000);
	fin_for_info_ = nh_.advertise<mia_hand_msgs::FingersStrainGauges>("fin_sg",
                                                                    1000);

  return;
}

void ROSDriver::thuMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorPos(0, msg->data);

  return;
}

void ROSDriver::thuMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorSpe(0, msg->data);

  return;
}

void ROSDriver::thuFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setFingerFor(0, msg->data);

  return;
}

void ROSDriver::thuCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setThuGraspRef('C', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::thuPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setThuGraspRef('P', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::thuLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setThuGraspRef('L', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::thuSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setThuGraspRef('S', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::thuTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setThuGraspRef('T', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setIndGraspRef('C', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setIndGraspRef('P', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setIndGraspRef('L', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setIndGraspRef('S', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setIndGraspRef('T', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::mrlCylGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setMrlGraspRef('C', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::mrlPinGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setMrlGraspRef('P', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::mrlLatGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setMrlGraspRef('L', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::mrlSphGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setMrlGraspRef('S', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::mrlTriGraspRefCallback(const mia_hand_msgs::GraspRef::ConstPtr&
                                       msg)
{
  mia_.setMrlGraspRef('T', msg->rest, msg->pos, msg->delay);

  return;
}

void ROSDriver::indMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorPos(2, msg->data);

  return;
}

void ROSDriver::indMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorSpe(2, msg->data);

  return;
}

void ROSDriver::indFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setFingerFor(2, msg->data);

  return;
}

void ROSDriver::mrlMotTrgtPosCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorPos(1, msg->data);

  return;
}

void ROSDriver::mrlMotTrgtSpeCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setMotorSpe(1, msg->data);

  return;
}

void ROSDriver::mrlFinTrgtForCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.setFingerFor(1, msg->data);

  return;
}

void ROSDriver::cylGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.closeGrasp('C', msg->data);

  return;
}

void ROSDriver::pinGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.closeGrasp('P', msg->data);

  return;
}


void ROSDriver::latGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.closeGrasp('L', msg->data);

  return;
}

void ROSDriver::sphGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.closeGrasp('S', msg->data);

  return;
}

void ROSDriver::triGraspPercentCallback(const std_msgs::Int16::ConstPtr& msg)
{
  mia_.closeGrasp('T', msg->data);

  return;
}

bool ROSDriver::connectToPortCallback(
       mia_hand_msgs::ConnectSerial::Request&  req,
       mia_hand_msgs::ConnectSerial::Response& resp)
{
  bool is_port_open = mia_.connectToPort(req.port);
    
  if (is_port_open)
  {
    resp.success = true;
    resp.message = "/dev/ttyUSB" + std::to_string(req.port)
                 + " succesfully opened.";
  }
  else
  {
    resp.success = false;
    resp.message = "Could not open /dev/ttyUSB" + std::to_string(req.port)
                 + ".";
  }
	
  return true;
}

bool ROSDriver::disconnectCallback(std_srvs::Trigger::Request& req,
                                   std_srvs::Trigger::Response& resp)
{
  bool is_port_closed = mia_.disconnect();

  if (is_port_closed)
  {
    resp.success = true;
    resp.message = "Mia Hand serial port closed.";
  }
  else
  {
    resp.success = false;
    resp.message = "Could not close Mia Hand serial port.";
  }

	return true;
}

bool ROSDriver::switchPosStreamCallback(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& resp)
{	
  mia_.switchPosStream(req.data);

	return true;
}

bool ROSDriver::switchSpeStreamCallback(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& resp)
{
  mia_.switchSpeStream(req.data);
	
	return true;
}

bool ROSDriver::switchAnaStreamCallback(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& resp)
{
  mia_.switchAnaStream(req.data);
	
	return true;
}

bool ROSDriver::switchCurStreamCallback(std_srvs::SetBool::Request& req,
                                         std_srvs::SetBool::Response& resp)
{
  mia_.switchCurStream(req.data);
	
	return true;
}

bool ROSDriver::openCylGraspCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& resp)
{
  mia_.openGrasp('C');

  return true;
}

bool ROSDriver::openPinGraspCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& resp)
{
  mia_.openGrasp('P');

  return true;
}

bool ROSDriver::openLatGraspCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& resp)
{
  mia_.openGrasp('L');

  return true;
}

bool ROSDriver::openSphGraspCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& resp)
{
  mia_.openGrasp('S');

  return true;
}

bool ROSDriver::openTriGraspCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& resp)
{
  mia_.openGrasp('T');

  return true;
}

bool ROSDriver::closeCylGraspCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
{
  mia_.closeGrasp('C');

  return true;
}

bool ROSDriver::closePinGraspCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
{
  mia_.closeGrasp('P');

  return true;
}

bool ROSDriver::closeLatGraspCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
{
  mia_.closeGrasp('L');

  return true;
}

bool ROSDriver::closeSphGraspCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
{
  mia_.closeGrasp('S');

  return true;
}

bool ROSDriver::closeTriGraspCallback(std_srvs::Empty::Request& req,
                                      std_srvs::Empty::Response& resp)
{
  mia_.closeGrasp('T');

  return true;
}
}  // namespace
