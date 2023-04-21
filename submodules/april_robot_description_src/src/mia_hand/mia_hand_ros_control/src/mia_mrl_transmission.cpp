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

#include "mia_hand_ros_control/mia_mrl_transmission.h"
#include <math.h>
#include <ros/console.h>
#include <stdexcept>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

//////////////////////////////////////////////////////////////////////////////////////////////
//MiaMrlTransmission : constructor
//////////////////////////////////////////////////////////////////////////////////////////////

 MiaMrlTransmission::MiaMrlTransmission()
  : Transmission()
{

  // Load config file
  std::string config_file;
  if (ros::param::has("mia_transmissions_config_"))
  {
    ros::param::get("mia_transmissions_config_", config_file);
    ROS_INFO_STREAM("MiaMrlTransmission: "<< "Config file found: "<< config_file);

  }
  else
  {
    ROS_ERROR_NAMED("MiaTransmission","Transmission config file not found. Unable to load Mia Mrl transmission");
    throw TransmissionInterfaceException("Mia transmission config file name not received - Aborting.. " );
  }

  // Parse the yaml file and get transmission configurations
  YAML::Node lconf= YAML::LoadFile(config_file);

   reduction_ = 64.0 * 20.0;
   h_mrl_offset_      = lconf["transmissions"]["j_mrl_fle"]["h_mrl_offset"].as<double>();
   h_mrl_scale_       = lconf["transmissions"]["j_mrl_fle"]["h_mrl_scale"].as<double>();
   h_mrl_inv_offset_  = lconf["transmissions"]["j_mrl_fle"]["h_mrl_inv_offset"].as<double>();
   h_mrl_inv_scale_   = lconf["transmissions"]["j_mrl_fle"]["h_mrl_inv_scale"].as<double>();
   h2_mrl_offset_     = lconf["transmissions"]["j_mrl_fle"]["h2_mrl_offset"].as<double>();
   h2_mrl_scale_      = lconf["transmissions"]["j_mrl_fle"]["h2_mrl_scale"].as<double>();
   h2_mrl_inv_offset_ = lconf["transmissions"]["j_mrl_fle"]["h2_mrl_inv_offset"].as<double>();
   h2_mrl_inv_scale_  = lconf["transmissions"]["j_mrl_fle"]["h2_mrl_inv_scale"].as<double>();

  if (0.0 == reduction_)
  {
    throw TransmissionInterfaceException("Transmission reduction ratio cannot be zero.");
  }

  // TO DELETE -- TEST

  double test_act_state = 210; //1.016

  double test_jnt_state = testActuatorToJointPosition( test_act_state );
  ROS_INFO_STREAM("MiaMrlTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
  test_act_state = testJointToActuatorPosition( test_jnt_state);
  ROS_INFO_STREAM("MiaMrlTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);

  //
  test_act_state = 75;//0.259
  test_jnt_state = testActuatorToJointPosition( test_act_state );
  ROS_INFO_STREAM("MiaMrlTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
  test_act_state = testJointToActuatorPosition( test_jnt_state);
  ROS_INFO_STREAM("MiaMrlTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);

 //
 test_act_state = 23;// -0.05
 test_jnt_state = testActuatorToJointPosition( test_act_state );
 ROS_INFO_STREAM("MiaMrlTransmission: "<< "Actuator: "<<   test_act_state <<" --> Joint: "<< test_jnt_state);
 test_act_state = testJointToActuatorPosition( test_jnt_state);
 ROS_INFO_STREAM("MiaMrlTransmission: "<< "joiont: "<<   test_jnt_state <<" --> Actuator: "<< test_act_state);

}


 //////////////////////////////////////////////////////////////////////////////////////////////
// Simple functions needed to evaluate trasmission ratio
/////////////////////////////////////////////////////////////////////////////////////////////

// mu = h_mrl(pos)
inline double  MiaMrlTransmission::h_mrl(const double pos)
{
  double scale  = h_mrl_scale_;
  double offset = h_mrl_offset_;
  double mu = scale * pos + offset;

  //ROS_INFO_STREAM("MiaMrlTransmission: "<< "pos: "<<  pos <<" --> mu: "<< mu<<" -- Scale: "<< scale <<" offset: "<< offset );
  return mu;
}

// pos = h_mrl_inv(mu)
double  MiaMrlTransmission::h_mrl_inv(const double mu)
{
  double scale  = h_mrl_inv_scale_;
  double offset = h_mrl_inv_offset_;
  double pos = (double) round( scale * mu + offset); // cmd
  return pos;
}

// omega_m = dh(spe)
double  MiaMrlTransmission::dh (const double spe)
{
  double scale  = 65.4167;
  double offset = 0;
  double omega_m = scale * spe + offset;
  return omega_m;
}

// spe = dh_i_inv(omega_m)
double  MiaMrlTransmission::dh_inv(const double omega_m)
{
  double scale  = 0.015287;
  double offset = 0;
  double spe = (double) round( scale * omega_m + offset); // cmd
  return spe;
}

//fi = h2_mrl(mu).
double MiaMrlTransmission::h2_mrl(const double mu)
{
  double scale  = h2_mrl_scale_;
  double offset = h2_mrl_offset_;
  double fi = scale * mu + offset;
  //ROS_INFO_STREAM("MiaMrlTransmission: "<< "mu: "<<  mu <<" --> Joint: "<< fi<<" -- Scale: "<< scale <<" offset: "<< offset );
  return fi;
}

//mu = h2_mrl_inv(fi).
double MiaMrlTransmission::h2_mrl_inv(const double fi)
{
  double scale  = h2_mrl_inv_scale_;
  double offset = h2_mrl_inv_offset_;
  double mu = scale * fi + offset;
  return mu;
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Macro functions of for the ros trasmission
/////////////////////////////////////////////////////////////////////////////////////////////

void MiaMrlTransmission::actuatorToJointVelocity(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{

  double spe = *act_data.velocity[0];
  double omega_m = dh(spe);
  double omega_f = omega_m / reduction_;

  *jnt_data.velocity[0] = omega_f;
}

void MiaMrlTransmission::actuatorToJointPosition(const ActuatorData& act_data,
                                                               JointData&    jnt_data)
{
  double pos = *act_data.position[0];
  double mu = h_mrl(pos);
  double fi = h2_mrl(mu);
  //double fi = mu / reduction_;

  *jnt_data.position[0] = fi;
}


void MiaMrlTransmission::jointToActuatorVelocity(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double omega_f = *jnt_data.velocity[0];
  double omega_m = omega_f * reduction_;
  double spe = dh_inv(omega_m);

  *act_data.velocity[0] = spe;
}

void MiaMrlTransmission::jointToActuatorPosition(const JointData&    jnt_data,
                                                               ActuatorData& act_data)
{

  double fi = *jnt_data.position[0];
  double mu = h2_mrl_inv(fi);
  //double mu = fi * reduction_;
  double pos = h_mrl_inv(mu);

  *act_data.position[0] = pos;
}

// TO DELETE

double MiaMrlTransmission::testActuatorToJointPosition(double act_data)
{
  double pos = act_data;
  double mu = h_mrl(pos);
  double fi = h2_mrl(mu);
  //double fi = mu / reduction_;

  return fi;
}

double MiaMrlTransmission::testJointToActuatorPosition(double jnt_data)
{

  double fi = jnt_data;
  double mu = h2_mrl_inv(fi);
  //double mu = fi * reduction_;
  double pos = h_mrl_inv(mu);

  return pos;
}

//////////////////////////////////////////////////////////////////////////////////
// NOT TO BE USED

void MiaMrlTransmission::actuatorToJointEffort(const ActuatorData& act_data,
                                                             JointData&    jnt_data)
{

   *jnt_data.effort[0] = *act_data.effort[0] * 1;
}

void MiaMrlTransmission::jointToActuatorEffort(const JointData&    jnt_data,
                                                             ActuatorData& act_data)
{

   *act_data.effort[0] = *jnt_data.effort[0] / 1;
}

///////////////////////////////////////////////////////////////////////////////////////////

}// namespace mia_transmission_interface
