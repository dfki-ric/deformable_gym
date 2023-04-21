/*
* Copyright (C) 2021 Prensilia s.r.l.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation and/or
* other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#include "mia_hand_ros_control/mia_thfle_transmission.h"
#include <math.h>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

//////////////////////////////////////////////////////////////////////////////////////////////
//MiaThfleTransmission : constructor
//////////////////////////////////////////////////////////////////////////////////////////////

 MiaThfleTransmission::MiaThfleTransmission()
  : Transmission()
{

  // Load config file
  std::string config_file;
  if (ros::param::has("mia_transmissions_config_"))
  {
    ros::param::get("mia_transmissions_config_", config_file);
    ROS_INFO_STREAM("MiaThfleTransmission: "<< "Config file found: "<< config_file);

  }
  else
  {
    ROS_ERROR_NAMED("MiaTransmission","Transmission config file not found. Unable to load Mia Thfle transmission");
    throw TransmissionInterfaceException("Mia transmission config file name not received - Aborting.. " );
  }

  // Parse the yaml file and get transmission configurations
  YAML::Node lconf= YAML::LoadFile(config_file);

  reduction_ = 64.0 * 40.0;

  h_tf_offset_      = lconf["transmissions"]["j_thumb_fle"]["h_tf_offset"].as<double>();
  h_tf_scale_       = lconf["transmissions"]["j_thumb_fle"]["h_tf_scale"].as<double>();
  h_tf_inv_offset_  = lconf["transmissions"]["j_thumb_fle"]["h_tf_inv_offset"].as<double>();
  h_tf_inv_scale_   = lconf["transmissions"]["j_thumb_fle"]["h_tf_inv_scale"].as<double>();
  h2_tf_offset_     = lconf["transmissions"]["j_thumb_fle"]["h2_tf_offset"].as<double>();
  h2_tf_scale_      = lconf["transmissions"]["j_thumb_fle"]["h2_tf_scale"].as<double>();
  h2_tf_inv_offset_ = lconf["transmissions"]["j_thumb_fle"]["h2_tf_inv_offset"].as<double>();
  h2_tf_inv_scale_  = lconf["transmissions"]["j_thumb_fle"]["h2_tf_inv_scale"].as<double>();


  if (0.0 == reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratio cannot be zero.");
  }

  // TO DELETE -- TEST

  double test_act_state = 210; //0.906

  double test_jnt_state = testActuatorToJointPosition( test_act_state );
  ROS_INFO_STREAM("MiaThfleTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
  test_act_state = testJointToActuatorPosition( test_jnt_state);
  ROS_INFO_STREAM("MiaThfleTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);

  //
  test_act_state = 75;//0.319
  test_jnt_state = testActuatorToJointPosition( test_act_state );
  ROS_INFO_STREAM("MiaThfleTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
  test_act_state = testJointToActuatorPosition( test_jnt_state);
  ROS_INFO_STREAM("MiaThfleTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);

 //
 test_act_state = 23;// 0.093
 test_jnt_state = testActuatorToJointPosition( test_act_state );
 ROS_INFO_STREAM("MiaThfleTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
 test_act_state = testJointToActuatorPosition( test_jnt_state);
 ROS_INFO_STREAM("MiaThfleTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);
}


 //////////////////////////////////////////////////////////////////////////////////////////////
// Simple functions needed to evaluate trasmission ratio
/////////////////////////////////////////////////////////////////////////////////////////////

// mu = h_thfle(pos)
inline double  MiaThfleTransmission::h_thfle(const double pos)
{
  double scale  = h_tf_scale_;
  double offset = h_tf_offset_;
  double mu = scale * pos + offset;
  return mu;
}

// pos = h_thfle_inv(mu)
double  MiaThfleTransmission::h_thfle_inv(const double mu)
{
  double scale  = h_tf_inv_scale_;
  double offset = h_tf_inv_offset_;
  double pos = (double) round( scale * mu + offset); // cmd
  return pos;
}

// omega_m = dh(spe)
double  MiaThfleTransmission::dh (const double spe)
{
  double scale  = 65.4167;
  double offset = 0;
  double omega_m = scale * spe + offset;
  return omega_m;
}

// spe = dh_i_inv(omega_m)
double  MiaThfleTransmission::dh_inv(const double omega_m)
{
  double scale  = 0.015287;
  double offset = 0;
  double spe = (double) round( scale * omega_m + offset); // cmd
  return spe;
}


// sigma = h2_thfle(mu)
inline double  MiaThfleTransmission::h2_thfle(const double mu)
{
  double scale  = h2_tf_scale_;
  double offset = h2_tf_offset_;
  double sigma = scale * mu + offset;
  return sigma;
}

// mu = h_thfle_inv(sigma)
double  MiaThfleTransmission::h2_thfle_inv(const double sigma)
{
  double scale  = h2_tf_inv_scale_;
  double offset = h2_tf_inv_offset_;
  double mu =  scale * sigma + offset;
  return mu;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Macro functions of for the ros trasmission
/////////////////////////////////////////////////////////////////////////////////////////////

void MiaThfleTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{

  double spe = *act_data.velocity[0];
  double omega_m = dh(spe);
  double omega_f = omega_m / reduction_;

  *jnt_data.velocity[0] = omega_f;
}

void MiaThfleTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{
  double pos = *act_data.position[0];
  double mu = h_thfle(pos);
  //double sigma = mu / reduction_;
  double sigma = h2_thfle(mu);

  *jnt_data.position[0] = sigma;
}


void MiaThfleTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double omega_f = *jnt_data.velocity[0];
  double omega_m = omega_f * reduction_;
  double spe = dh_inv(omega_m);

  *act_data.velocity[0] = spe;
}

void MiaThfleTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double sigma = *jnt_data.position[0];
  double mu = h2_thfle_inv(sigma);
  //double mu = sigma * reduction_;
  double pos = h_thfle_inv(mu);

  *act_data.position[0] = pos;
}

// TO DELETE

double MiaThfleTransmission::testActuatorToJointPosition(double act_data)
{
  double pos = act_data;
  double mu = h_thfle(pos);
  //double sigma = mu / reduction_;
  double sigma = h2_thfle(mu);


  return sigma;
}

double MiaThfleTransmission::testJointToActuatorPosition(double jnt_data)
{

  double sigma = jnt_data;
  double mu = h2_thfle_inv(sigma);
  //double mu = sigma * reduction_;
  double pos = h_thfle_inv(mu);

  return pos;
}

//////////////////////////////////////////////////////////////////////////////////
// NOT TO BE USED

void MiaThfleTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                             JointData&    jnt_data)
{

   *jnt_data.effort[0] = *act_data.effort[0] * 1;
}

void MiaThfleTransmission::jointToActuatorEffort(const JointData&    jnt_data,
                                                             ActuatorData& act_data)
{

   *act_data.effort[0] = *jnt_data.effort[0] / 1;
}

///////////////////////////////////////////////////////////////////////////////////////////

}// namespace mia_transmission_interface
