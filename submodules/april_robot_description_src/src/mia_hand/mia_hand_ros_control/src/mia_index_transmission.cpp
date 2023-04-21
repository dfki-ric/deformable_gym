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

#include "mia_hand_ros_control/mia_index_transmission.h"
#include <math.h>

using transmission_interface::ActuatorData;
using transmission_interface::JointData;

namespace transmission_interface
{

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Initialize broken lines fucntons imlementing mia index transmisiion
  /////////////////////////////////////////////////////////////////////////////////////////////
  MiaIndexTransmission::MiaIndexTransmission ()
   : Transmission()
  {

    // Load config file
    std::string config_file;
    if (ros::param::has("mia_transmissions_config_"))
    {
      ros::param::get("mia_transmissions_config_", config_file);
      ROS_INFO_STREAM("MiaIndexTransmission: "<< "Config file found: "<< config_file);

    }
    else
    {
      ROS_ERROR_NAMED("MiaTransmission","Transmission config file not found. Unable to load Mia Index transmission");
      throw TransmissionInterfaceException("Mia transmission config file name not received - Aborting.. " );
    }

    // Parse the yaml file and get transmission configurations
    YAML::Node lconf= YAML::LoadFile(config_file);

    // linear trasmission section
    linear_reduction_ = 64.0 * 20.0; // TR of the linear segment of the trasmission

    // non linear trasmission section
    x_intervals_f_     = lconf["transmissions"]["j_index_fle"]["x_intervals_f"].as<std::vector<double>>();
    x_intervals_f_inv_ = lconf["transmissions"]["j_index_fle"]["x_intervals_f_inv"].as<std::vector<double>>();
    x_intervals_df_    = lconf["transmissions"]["j_index_fle"]["x_intervals_df"].as<std::vector<double>>();
    f_scale_           = lconf["transmissions"]["j_index_fle"]["f_scale"].as<std::vector<double>>();
    f_offset_          = lconf["transmissions"]["j_index_fle"]["f_offset"].as<std::vector<double>>();
    f_inv_scale_       = lconf["transmissions"]["j_index_fle"]["f_inv_scale"].as<std::vector<double>>();
    f_inv_offset_      = lconf["transmissions"]["j_index_fle"]["f_inv_offset"].as<std::vector<double>>();
    df_scale_          = lconf["transmissions"]["j_index_fle"]["df_scale"].as<std::vector<double>>();
    df_offset_         = lconf["transmissions"]["j_index_fle"]["df_offset"].as<std::vector<double>>();
    f2_offset_          = lconf["transmissions"]["j_index_fle"]["f2_offset"].as<double>();

    h_i_offset_      = lconf["transmissions"]["j_index_fle"]["h_i_offset"].as<double>();
    h_i_scale_       = lconf["transmissions"]["j_index_fle"]["h_i_scale"].as<double>();
    h_i_inv_offset_  = lconf["transmissions"]["j_index_fle"]["h_i_inv_offset"].as<double>();
    h_i_inv_scale_   = lconf["transmissions"]["j_index_fle"]["h_i_inv_scale"].as<double>();
    h2_i_offset_     = lconf["transmissions"]["j_index_fle"]["h2_i_offset"].as<double>();
    h2_i_scale_      = lconf["transmissions"]["j_index_fle"]["h2_i_scale"].as<double>();
    h2_i_inv_offset_ = lconf["transmissions"]["j_index_fle"]["h2_i_inv_offset"].as<double>();
    h2_i_inv_scale_  = lconf["transmissions"]["j_index_fle"]["h2_i_inv_scale"].as<double>();
  }


  //////////////////////////////////////////////////////////////////////////////////////////////
  // Simple functions needed to evaluate trasmission ratio
  /////////////////////////////////////////////////////////////////////////////////////////////

  // mu = h_i(pos)
  double  MiaIndexTransmission::h_i(const double pos)
  {
    double scale  = h_i_scale_;//-8.673;
    double offset = h_i_offset_; //580.8284;
    double mu = scale * pos + offset;
    return mu;
  }

  // pos = h_i_inv(mu)
  double  MiaIndexTransmission::h_i_inv(const double mu)
  {
    double scale  = h_i_inv_scale_;//-0.1153;
    double offset = h_i_inv_offset_;//66.9697;
    double pos = (double) round( scale * mu + offset); // cmd
    return pos;
  }

  // omega_m = dh_i(spe)
  double  MiaIndexTransmission::dh_i(const double spe)
  {
    double scale  = 65.4167;
    double offset = 0;
    double omega_m = scale * spe + offset;
    return omega_m;
  }

  // spe = dh_i_inv(omega_m)
  double  MiaIndexTransmission::dh_i_inv(const double omega_m)
  {
    double scale  = 0.015287;
    double offset = 0;
    double spe = (double) round( scale * omega_m + offset); // cmd
    return spe;
  }

  // alpha = h2_i(mu)
  double  MiaIndexTransmission::h2_i(const double mu)
  {
    double scale  = h2_i_scale_;
    double offset = h2_i_offset_;
    double alpha = scale * mu + offset;

    //ROS_INFO_STREAM("MiaIndexTransmission: "<< "h2i mu: "<<   mu <<" --> alpha: "<< alpha << "-- Scale: " << scale << " Offset: "<< offset);
    return alpha;
  }

  // mu = h2_i_inv(alpha)
  double  MiaIndexTransmission::h2_i_inv(const double alpha)
  {
    double scale  = h2_i_inv_scale_ ;
    double offset = h2_i_inv_offset_ ;
    double mu = scale * alpha + offset;
    return mu;
  }

  // delta = f(alpha)
  double  MiaIndexTransmission::f(const double alpha )
  {

    double delta;

    // input larger than the last x interval limit
    if (alpha > x_intervals_f_[x_intervals_f_.size() - 1 ])
    {
      delta = f_scale_[x_intervals_f_.size() - 1] * alpha + f_offset_[x_intervals_f_.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_f_.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (alpha <= x_intervals_f_[j])
          {
            delta = f_scale_[j] * alpha + f_offset_[j];
            break;
          }
        }
        else // Other intervals
        {
          if (alpha > x_intervals_f_[j-1] && alpha <= x_intervals_f_[j])
          {
            delta = f_scale_[j] * alpha + f_offset_[j];
            break;
          }
        }
      }

    }
    return delta;


  }


  // alpha = f_inv(delta)
  double  MiaIndexTransmission::f_inv( const double delta)
  {

    double alpha;

    // input larger than the last x interval limit
    if (delta > x_intervals_f_inv_[x_intervals_f_inv_.size() - 1 ])
    {
      alpha = f_inv_scale_[x_intervals_f_inv_.size() - 1] * delta + f_inv_offset_[x_intervals_f_inv_.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_f_inv_.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (delta <= x_intervals_f_inv_[j])
          {
            alpha = f_inv_scale_[j] * delta + f_inv_offset_[j];
            break;
          }
        }
        else // Other intervals
        {
          if (delta > x_intervals_f_inv_[j-1] && delta <= x_intervals_f_inv_[j])
          {
            alpha = f_inv_scale_[j] * delta + f_inv_offset_[j];
            break;
          }
        }
      }

    }
    return alpha;

  }


  // Non linear TR-1
  double  MiaIndexTransmission::df( const double alpha )
  {

    double f_alpha;

    // input larger than the last x interval limit
    if (alpha > x_intervals_df_[x_intervals_df_.size() - 1 ])
    {
      f_alpha = df_scale_[x_intervals_df_.size() - 1] * alpha + df_offset_[x_intervals_df_.size() - 1];
    }
    else
    {

      for(unsigned int j=0; j < x_intervals_df_.size() ; j++)
      {

        // First interval
        if (j == 0)
        {
          if (alpha <= x_intervals_df_[j])
          {
            f_alpha = df_scale_[j] * alpha + df_offset_[j];
            break;
          }
        }
        else // Other intervals
        {
          if (alpha > x_intervals_df_[j-1] && alpha <= x_intervals_df_[j])
          {
            f_alpha = df_scale_[j] * alpha + df_offset_[j];
            break;
          }
        }
      }

    }

    if (0.0 == f_alpha)
    {
  	  f_alpha = 0.0051;
    }

    return abs(f_alpha);


  }

  // delta_calib = f2(delta)
    double  MiaIndexTransmission::f2(const double delta)
    {
      double offset = f2_offset_;
      double delta_calib = delta;

      if(offset > 0)
      {
        if(delta >= 0){
          delta_calib = delta+offset;}
        else {
          delta_calib = delta - offset;}
      }
      else if (offset < 0)
      {
        if(delta >= abs(offset)){
          delta_calib = delta + offset; // pull down
        }
        else if (delta <= -abs(offset)){
          delta_calib = delta - offset; // pull up
        }
        else if (abs(delta) < abs(offset)) {
          delta_calib = 0; // zeros
        }
      }

      //ROS_INFO_STREAM("MiaIndexTransmission: "<< "f2 delta: "<<   delta <<" --> delta_calib: "<< delta_calib << " -- offset calib: "<<offset);
      return delta_calib;
    }

    // delta = f2_inv(delta_calib)
    double  MiaIndexTransmission::f2_inv(const double delta_calib)
    {
      double offset = f2_offset_;
      double delta = delta_calib;

      if(offset > 0)
      {
        if(delta_calib  >= abs(offset)){
          delta = delta_calib - offset;
        }
        else if (delta_calib <= -abs (offset)){
          delta = delta_calib + offset;
        }
        else if (abs(delta_calib) < abs(offset)) {
          delta = offset; // zeros
        }
      }
      else if(offset < 0)
      {
        if(delta_calib >= 0){
          delta = delta_calib - offset;
        }
        else if(delta_calib < 0){
          delta = delta_calib + offset;
        }
      }

      return delta;
    }

  //////////////////////////////////////////////////////////////////////////////////////////////
  // Macro functions of for the ros trasmission
  /////////////////////////////////////////////////////////////////////////////////////////////


  // Mapping from data pos returned by MIA to joint angle.
  void MiaIndexTransmission::actuatorToJointPosition(
    const ActuatorData &ind_act_state,
          JointData    &ind_jnt_data)
  {

    double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
    double mu = h_i(pos);
    double alpha = h2_i(mu);
    //double alpha = mu / linear_reduction_;
    double delta = f(alpha);
    double delta_calib = f2(delta);

    *ind_jnt_data.position[0] = delta_calib;

    return;
  }

  // Mapping from data spe returned by MIA to joint velocity.
  void MiaIndexTransmission::actuatorToJointVelocity(
    const ActuatorData &ind_act_state,
          JointData    &ind_jnt_data)
  {

    double spe = *ind_act_state.velocity[0];  // discreto (-90 ; +90)
    double omega_m = dh_i(spe);
    double omega_a = omega_m / linear_reduction_;

     double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
     double mu = h_i(pos);
     double alpha = h2_i(mu);
     //double alpha = mu / linear_reduction_;

     double df_alpha = df(alpha); // non linear reduction ratio inv
     double omega_d = omega_a * df_alpha;

    // ROS_INFO_STREAM("alpha: "<< alpha<< " | df_alpha " << df_alpha <<  " | omega_d " << omega_d);

    *ind_jnt_data.velocity[0] = omega_d;

    return;
  }

  // Mapping from joint position to Mia pos command.
  void MiaIndexTransmission::jointToActuatorPosition(
    const JointData    &ind_jnt_data,
          ActuatorData &ind_act_cmd)
  {

    double delta_calib = *ind_jnt_data.position[0];
    double delta = f2_inv(delta_calib);
    double alpha = f_inv(delta);
    double mu = h2_i_inv(alpha);
    //double mu = alpha * linear_reduction_;
    double pos = h_i_inv(mu);                // discreto (-255 ; 255)


    *ind_act_cmd.position[0] = pos;

    return;
  }

  // Mapping from joint velocity to Mia spe command.
  void MiaIndexTransmission::IndexjointToActuatorVelocity(
    const JointData    &ind_jnt_data,
    const ActuatorData &ind_act_state,
          ActuatorData &ind_act_cmd)
  {
    double omega_d = *ind_jnt_data.velocity[0] ;

    double pos = *ind_act_state.position[0]; // -255 ; 255 (discreto)
    double mu = h_i(pos);
    double alpha = h2_i(mu);
    //double alpha = mu / linear_reduction_;

    double df_alpha = df(alpha); // non linear reduction ratio inv
    double omega_a = omega_d/df_alpha;
    double omega_m = omega_a * linear_reduction_;
    double spe = dh_i_inv(omega_m);

    // ROS_INFO_STREAM("alpha: "<< alpha<< " | df_alpha " << df_alpha <<  " | omega_d " << omega_d << " | omega_m " << omega_m);

    *ind_act_cmd.velocity[0] = spe;

    return;
  }

  // NOT USED
  void MiaIndexTransmission::jointToActuatorVelocity(
        const transmission_interface::JointData&    ind_jnt_data,
              transmission_interface::ActuatorData& act_data)
  {
     *act_data.velocity[0] = *ind_jnt_data.velocity[0] * 1;
  }



  //////////////////////////////////////////////////////////////////////////////////////////////
  // NOT IMPLEMENTED FUNCTIONS :since these cannot be used with MIA
  /////////////////////////////////////////////////////////////////////////////////////////////

  void MiaIndexTransmission::actuatorToJointEffort(const ActuatorData &ind_act_state,
                                                    JointData    &ind_jnt_data)
  {
    /* TODO: replace this calculation with the mapping from motor to joint
     * effort.
     */
    *ind_jnt_data.effort[0] = *ind_act_state.effort[0] * 1;

    return;
  }

  // NOT IMPLEMENTED:
  void MiaIndexTransmission::jointToActuatorEffort(
    const JointData    &ind_jnt_data,
          ActuatorData &ind_act_cmd)
  {
    /* TODO: replace this calculation with the mapping from joint to motor
     * effort.
     */
    *ind_act_cmd.effort[0] = *ind_jnt_data.effort[0] / 1;

    return;
  }

  /*-------------------------------------------------------------------------------------*/

  double MiaIndexTransmission::testActuatorToJointPosition(
    double ind_act_state )
    {
      double pos = ind_act_state; // -255 ; 255 (discreto)
      double mu = h_i(pos);
      double alpha = h2_i(mu);
      //double alpha = mu / linear_reduction_;
      double delta = f(alpha);
      double delta_calib = f2(delta);


      return delta_calib;
    }


  double MiaIndexTransmission::testJointToActuatorPosition(
    double ind_joint_state )
    {
      double delta_calib = ind_joint_state;
      double delta = f2_inv(delta_calib);
      double alpha = f_inv(delta);
      double mu = h2_i_inv(alpha);
      //double mu = alpha * linear_reduction_;
      double pos = h_i_inv(mu);                // discreto (-255 ; 255)



      return pos;
    }
/*-------------------------------------------------------------------------------------*/

}  // namespace
