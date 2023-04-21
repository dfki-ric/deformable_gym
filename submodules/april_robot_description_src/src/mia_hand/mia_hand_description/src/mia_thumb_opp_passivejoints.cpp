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

#include "mia_hand_description/mia_thumb_opp_passivejoints.h"
#include <stdexcept>


namespace mia_hand
{

   /** Initialize*/
    thumb_opp_passive_joint::thumb_opp_passive_joint()
    {

    }

    void thumb_opp_passive_joint::init(const bool LoadURDFInfo_ )
    {
      j_index_name ="j_index_fle";  // default
      j_thumb_name ="j_thumb_opp";  // default
      robot_description_ = "robot_description"; // default

      if(LoadURDFInfo_) { bool temp = LoadURDFInfo();}
      else
      {
        updateThOppJointLimits(0.0 , 0.0);

      }

      // load thumb opp config from yaml file and evaluate parameters
      std::string config_file;
      if (ros::param::has("mia_transmissions_config_"))
      {
        ros::param::get("mia_transmissions_config_", config_file);
        ROS_INFO_STREAM("thumb_opp_passive_joint: "<< "Config file found: "<< config_file);

      }
      else
      {
        ROS_ERROR_NAMED("thumb_opp_passive_joint","Transmission config file not found. Unable to load thumb_opp_passive_joint class");
        throw std::runtime_error("Mia transmission config file name not received - Aborting.. " );
      }

      // Parse the yaml file and get transmission configurations
      YAML::Node lconf= YAML::LoadFile(config_file);
      // non linear trasmission section
      th_opp_start_close_index_angle_     = lconf["transmissions"]["j_thumb_opp"]["th_opp_start_close_index_angle"].as<double>();
      th_opp_stop_close_index_angle_ = lconf["transmissions"]["j_thumb_opp"]["th_opp_stop_close_index_angle"].as<double>();

    }


    thumb_opp_passive_joint::~thumb_opp_passive_joint() {}

    /** Taking the position of the index_fle joint and returns the position of the thumb_opp joint.*/
     double thumb_opp_passive_joint::GetThumbOppPosition(double j_index_flex_pos)
     {
       // evaluate scale and offset
       //Th_opp_scale_ = (y(1) - y(2))/(x(1)-x(2));
       // Th_opp_offset = y(1) - (Th_abd_scale*x(1));

       //double empirical_scale_j_thumb_opp_pos = 0.86418; // 0.95238; old with cmd 25
       //double empirical_scale_j_thumb_opp_pos =  -0.063146;// -0.0052884;old with cmd 25
       double j_index_flex_pos_filt = (floor(j_index_flex_pos*1000))/1000; // filt the j_index_fle pose to avoid shakin
       th_opp_scale_ = (th_min_pos_ - th_max_pos_) / (th_opp_start_close_index_angle_ - th_opp_stop_close_index_angle_);
       th_opp_offset_ = th_min_pos_ - (th_opp_scale_*th_opp_start_close_index_angle_ );
         //ROS_INFO_STREAM("thumb_opp_passive_joint: "<< "Scale: "<< th_opp_scale_<< "  Offset: "<<th_opp_offset_);

       // Implementation of g3(delta) according with Mia trasmission and empirical test
       double j_th_opp_target_position_temp = th_opp_scale_ * (j_index_flex_pos_filt) + th_opp_offset_;
       double j_th_opp_target_position;


       if (j_th_opp_target_position_temp <= th_min_pos_)
       	 j_th_opp_target_position = th_min_pos_ + 0.02;

       else if (j_th_opp_target_position_temp >= th_max_pos_)
       	j_th_opp_target_position = th_max_pos_ - 0.02;

       else
       	j_th_opp_target_position = j_th_opp_target_position_temp;

        //ROS_INFO_STREAM("Scale: "<< th_opp_scale_<< "  Offset: "<<th_opp_offset_ << " Ix_: "<< j_index_flex_pos<< " Ix_f: "<< j_index_flex_pos_filt<< " thOpp :"<< j_th_opp_target_position_temp << "->" << j_th_opp_target_position);

       return j_th_opp_target_position;
     }


     /** Load joint limits from URD */
     bool thumb_opp_passive_joint::LoadURDFInfo()
     {
       bool result = false;

       const std::string urdf_string = getURDF(robot_description_);

        // get URDF model
        urdf::Model urdf_model;
        const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;

       const urdf::JointConstSharedPtr urdf_joint = urdf_model_ptr->getJoint(j_thumb_name);

       joint_limits_interface::JointLimits limits;

        if (urdf_joint != NULL)
        {

             // Get limits from the URDF file.
             if (joint_limits_interface::getJointLimits(urdf_joint, limits))
             {
               updateThOppJointLimits(limits.min_position, limits.max_position);

               result = true;
             }
       }
       else
       {
         updateThOppJointLimits(0.0, 0.0);
         result = false;
       }

      return result;
     }

     /** Update values of j_thumb_opp limits */
     void thumb_opp_passive_joint::updateThOppJointLimits(double min_position,
        double max_position)
     {
       th_min_pos_ = min_position;
       th_max_pos_ = max_position;
     }

     /** Find the URDF parameter name */
     std::string thumb_opp_passive_joint::getURDF(std::string param_name) const
    {
        std::string urdf_string;

        // search and wait for robot_description on param server
        while (urdf_string.empty())
        {
           std::string search_param_name;

           if (n.searchParam(param_name, search_param_name))
           {
                ROS_INFO_ONCE_NAMED("remap_mia_joint_states", "node is waiting for model"
               " URDF in parameter  on the ROS param server.");

                n.getParam(search_param_name, urdf_string);
           }
           else
           {
                ROS_INFO_ONCE_NAMED("remap_mia_joint_states", "Node  is waiting for model"
           	" URDF in parameter on the ROS param server.");

                n.getParam(param_name, urdf_string);
           }

        usleep(100000);
        }
        ROS_DEBUG_STREAM_NAMED("remap_mia_joint_states", "Recieved urdf from param server, parsing...");
	return urdf_string;
    }





}  // namespace
