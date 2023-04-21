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

#include <mia_hand_gazebo/Mia_hw_sim.h>


namespace
{
  /**
    * Clamp function:return the median value among three doubles
    */
    double clamp(const double val, const double min_val, const double max_val)
    {
        return std::min(std::max(val, min_val), max_val);
    }



}

namespace mia
{
    bool MiaHWSim::initSim(const std::string& robot_namespace,
                            ros::NodeHandle model_nh,
                            gazebo::physics::ModelPtr parent_model,
                            const urdf::Model *const urdf_model,
                            std::vector<transmission_interface::TransmissionInfo> transmissions)
    {

      ROS_INFO_NAMED("mia_hw_sim","Initialization of Mia Hw_Sim" );

      // Initialize passive joint thumb_opp + Index_fle
      const bool load = false;
      MyTh_opp_passiveJoint.init(load); // initialize without loading now URDF

      // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
      // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
      const ros::NodeHandle joint_limit_nh(model_nh);

      // Resize vectors to our mia hand DOF
      write_counter = 0;
      n_dof_ = transmissions.size();
      joint_names_.resize(n_dof_);
      joint_types_.resize(n_dof_);
      joint_lower_limits_.resize(n_dof_);
      joint_upper_limits_.resize(n_dof_);
      joint_effort_limits_.resize(n_dof_);
      joint_control_methods_.resize(n_dof_);
      pid_controllers_.resize(n_dof_);
      joint_position_.resize(n_dof_);
      joint_velocity_.resize(n_dof_);
      joint_effort_.resize(n_dof_);
      joint_effort_command_.resize(n_dof_);
      joint_position_command_.resize(n_dof_);
      joint_velocity_command_.resize(n_dof_);

      last_joint_position_command_.resize(n_dof_);
      last_joint_velocity_command_.resize(n_dof_);
      last_joint_effort_command_.resize(n_dof_);
      last_joint_control_methods_.resize(n_dof_);

      List_joint_control_methods_.resize(n_dof_);

      //*
      // Check if the loaded URDF has middle little ring independt or not.
      mrl_united = true;
      for(unsigned int j=0; j < n_dof_; j++)
      {
        // Check if in the Gazebo URDF there are j_middle_flex or j_ring_flex or j_little_flex
        // If any of them are found --> URDF mode mrl_united true
        const std::string jname = transmissions[j].joints_[0].name_;
        if (jname == "j_middle_fle" )
        {
          joints_ii.j_mrl_flex = j;
          mrl_united = false;
          break;
        }
        else if (jname == "j_ring_fle" )
        {
          joints_ii.j_mrl_2 = j;
          mrl_united = false;
          break;
        }
        else if (jname == "j_little_fle" )
        {
          joints_ii.j_mrl_3 = j;
          mrl_united = false;
          break;
        }
      }

      // Print the result
      if(!mrl_united)
      {
        ROS_INFO_STREAM_NAMED("mia_hw_sim"," middle ring and little as indipendent joints ");
      }
      else
      {
        ROS_INFO_STREAM_NAMED("mia_hw_sim"," middle ring and little as physically united joints ")	;
      }

      ///*
      // Initialize values
      for(unsigned int j=0; j < n_dof_; j++)
      {

        // Check that this transmission has one joint
        if(transmissions[j].joints_.size() == 0)
        {
          ROS_WARN_STREAM_NAMED("mia_hw_sim","Transmission " << transmissions[j].name_
          << " has no associated joints.");
          continue;
        }
        else if(transmissions[j].joints_.size() > 1)
        {
          ROS_WARN_STREAM_NAMED("mia_hw_sim","Transmission " << transmissions[j].name_
          << " has more than one joint. Currently the default robot hardware simulation "
          << " interface only supports one.");
          continue;
        }

        /* Get info about the used hardware interface from the tag hardware interface.
        Or if the hardware interface is empty try to finf this info within the actuator tag
        (deprectaed option) */
        std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;

        if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty())) // TODO: Deprecate HW interface specification in actuators in ROS J
        {
          joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
          ROS_WARN_STREAM_NAMED("mia_hw_sim", "The <hardware_interface> element of tranmission " <<
          transmissions[j].name_ << " should be nested inside the <joint> element, not <actuator>. " <<
          "The transmission will be properly loaded, but please update " <<
          "your robot model to remain compatible with future versions of the plugin.");
        }
        if (joint_interfaces.empty())
        {
          ROS_WARN_STREAM_NAMED("mia_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " does not specify any hardware interface. " <<
          "Not adding it to the robot hardware simulation.");
          continue;
        }
        else if (joint_interfaces.size() > 1)
        {
          ROS_WARN_STREAM_NAMED("mia_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
          " of transmission " << transmissions[j].name_ << " specifies multiple hardware interfaces. " <<
          "Currently the default robot hardware simulation interface only supports one. Using the first entry!");
          //continue;
        }


        /* Add data from transmission and save the index number of each joint of
          the simulated Mia hand */

        const std::string temp_joint_name = transmissions[j].joints_[0].name_;

        if (temp_joint_name == "j_thumb_opp" )
        {
          joints_ii.j_thumb_opp = j;
        }
        else if (temp_joint_name == "j_thumb_fle" )
        {
          joints_ii.j_thumb_flex = j;
        }
        else if (temp_joint_name == "j_index_fle" )
        {
          joints_ii.j_index_flex = j;
        }
        else if (temp_joint_name == "j_mrl_fle" )
        {
          joints_ii.j_mrl_flex = j;
        }
        else if (temp_joint_name == "j_ring_fle" )
        {
          joints_ii.j_mrl_2 = j;
        }
        else if (temp_joint_name == "j_little_fle" )
        {
          joints_ii.j_mrl_3 = j;
        }
        else
        {
          //ROS_INFO_STREAM_NAMED("mia_hw_sim","Additional joint that are not part of the MiaHand has been loaded: "
          //     << temp_joint_name);
        }

        // Add data from transmission
        joint_names_[j] = transmissions[j].joints_[0].name_;
        joint_position_[j] = 0;
        joint_velocity_[j] = 0.0;
        joint_effort_[j] = 1.0;  // N/m for continuous joints
        joint_effort_command_[j] = 0.0;
        joint_position_command_[j] = 0.0;
        joint_velocity_command_[j] = 0.0;

        const std::string& hardware_interface = joint_interfaces.front(); // returns the reference to the first element in the vector

        // Debug
        //ROS_INFO_STREAM_NAMED("mia_hw_sim", "loading Joint " << joint_names_[j]<< " with the index " << j );


        //*
        // Register Joint states interface for all the joints of the simulated hardware
        js_interface_.registerHandle(hardware_interface::JointStateHandle(
                      joint_names_[j], &joint_position_[j],
                      &joint_velocity_[j], &joint_effort_[j]));

          // Select and register the correct command interface for all the joints of the simulated hardware.
          std::vector< hardware_interface::JointHandle> joint_handle;

          if(hardware_interface == "EffortJointInterface" || hardware_interface =="hardware_interface/EffortJointInterface" )
          {
            ROS_INFO_STREAM_NAMED("mia_hw_sim", " Joint " << joint_names_[j]<< " EFFORT" );

            joint_handle.resize(1);
            List_joint_control_methods_[j].resize(1);

            // Create effort joint interface
            joint_control_methods_[j] = EFFORT;
            List_joint_control_methods_[j][0] = EFFORT;

            joint_handle[0] = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
            &joint_effort_command_[j]);
            ej_interface_.registerHandle(joint_handle[0]);
          }
          else if(hardware_interface == "PositionJointInterface" || hardware_interface == "VelocityJointInterface"
          || hardware_interface =="hardware_interface/PositionJointInterface" || hardware_interface == "hardware_interface/VelocityJointInterface" )
          {

            ROS_INFO_STREAM_NAMED("mia_hw_sim", " Joint " << joint_names_[j]<< " POS-VEL" );
            joint_handle.resize(2);

            List_joint_control_methods_[j].resize(2);

            // Create position joint interface
            joint_control_methods_[j] = POSITION;
            List_joint_control_methods_[j][0] = POSITION;

            joint_handle[0] = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
            &joint_position_command_[j]);
            pj_interface_.registerHandle(joint_handle[0]);

            // Create velocity joint interface
            hardware_interface::JointHandle joint_handle_vel;
            List_joint_control_methods_[j][1] = VELOCITY;

            joint_handle[1] = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
            &joint_velocity_command_[j]);
            vj_interface_.registerHandle(joint_handle[1]);
          }
          else
          {
            ROS_FATAL_STREAM_NAMED("default_robot_hw_sim","No matching hardware interface found for '"
            << hardware_interface );
            return false;
          }

          //*
          // Get the gazebo joint that corresponds to the robot joint.
          gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);

          if (!joint)
          {
            ROS_ERROR_STREAM("This robot has a joint named \"" << joint_names_[j]
            << "\" which is not in the gazebo model.");
            return false;
          }
          sim_joints_.push_back(joint); // Adds a new element at the end of the vector, after its current last element.


          //*
          // Get joint Limits
          registerJointLimits(joint_names_[j], joint_handle, List_joint_control_methods_[j],
            joint_limit_nh, urdf_model,
            &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
            &joint_effort_limits_[j]);


            if (List_joint_control_methods_[j][0] != EFFORT)
            {
              // Initialize the PID controller of the J_thumb_opp or other robot joints.
              const ros::NodeHandle nh(model_nh, "/gazebo_ros_control/pid_gains/" +
              joint_names_[j]);
              if (pid_controllers_[j].init(nh, true))
              {
                switch (List_joint_control_methods_[j][0])
                {
                  case POSITION:
                  List_joint_control_methods_[j][0] = POSITION_PID;
                  if (j != joints_ii.j_thumb_opp)
                  {
                    ROS_WARN_STREAM_NAMED("mia_hw_sim","POSITION_PID control not effective for mia joints. Please use ROS controllers"
                    <<" to load pid gains");
                  }

                  break;
                  case VELOCITY:
                  List_joint_control_methods_[j][0] = VELOCITY_PID;
                  ROS_WARN_STREAM_NAMED("mia_hw_sim","VELOCITY_PID control not effective for mia joints. Please use ROS controllers"
                  <<" to load pid gains");
                  break;
                }

                ROS_INFO_STREAM_NAMED("mia_hw_sim", " Get PID params !"  );
              }
              else
              {
                // joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
                // going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
                // going to be called.
                #if GAZEBO_MAJOR_VERSION > 2
                joint->SetParam("fmax", 0, joint_effort_limits_[j]); // fmax = maximum force (enum of the joint gazebo class)

                #else
                joint->SetMaxForce(0, joint_effort_limits_[j]);
                #endif
                ROS_INFO_STREAM_NAMED("mia_hw_sim", " Set fmax !"  );
              }




            }
          }// endfor joints

          // Register interfaces
          registerInterface(&js_interface_);
          registerInterface(&ej_interface_);
          registerInterface(&pj_interface_);
          registerInterface(&vj_interface_);

          // Initialize simulated Mia hand variables values
          j_index_flex_pos  = &joint_position_[joints_ii.j_index_flex];
          j_index_flex_pos_Th = 0.1848; // Threshold to change the sign of the index joint in position control
          j_index_flex_sign = 1; // initialize index position sign

          MyTh_opp_passiveJoint.updateThOppJointLimits (joint_lower_limits_[joints_ii.j_thumb_opp],
                                                        joint_upper_limits_[joints_ii.j_thumb_opp] );

          // MyTh_opp_passiveJoint.ThMinPos = joint_lower_limits_[joints_ii.j_thumb_opp] ;
          // MyTh_opp_passiveJoint.ThMaxPos = joint_upper_limits_[joints_ii.j_thumb_opp] ;


          // Initialize the emergency stop code.
          // TO BE DELETED IN THE NEXT VERSIONS
          e_stop_active_ = false;
          last_e_stop_active_ = false;

          return true;
        }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // RegisterJointLimits:Register the limits of the joint specified by joint_name and joint_handle.
    // The limits are retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
    // Return the joint's type, lower position limit, upper position limit, and effort limit.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void MiaHWSim::registerJointLimits(const std::string& joint_name,
                              const std::vector<hardware_interface::JointHandle>& joint_handle,
                              const std::vector<ControlMethod> ctrl_method,
                              const ros::NodeHandle& joint_limit_nh,
                              const urdf::Model *const urdf_model,
                              int *const joint_type, double *const lower_limit,
                              double *const upper_limit, double *const effort_limit)
    {
      // Initialize variables
      *joint_type = urdf::Joint::UNKNOWN;
      *lower_limit = -std::numeric_limits<double>::max();
      *upper_limit = std::numeric_limits<double>::max();
      *effort_limit = std::numeric_limits<double>::max();

      joint_limits_interface::JointLimits limits;
      bool has_limits = false;
      joint_limits_interface::SoftJointLimits soft_limits;
      bool has_soft_limits = false;

      // find limits
      if (urdf_model != NULL)
      {
        //const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
        const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
        if (urdf_joint != NULL)
        {
          *joint_type = urdf_joint->type;
          // Get limits from the URDF file.
          if (joint_limits_interface::getJointLimits(urdf_joint, limits))
          has_limits = true;
          if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
          has_soft_limits = true;
        }
      }
      // Get limits from the parameter server.
      if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
      has_limits = true;

      if (!has_limits)
      return;

      if (*joint_type == urdf::Joint::UNKNOWN)
      {
        // Infer the joint type.

        if (limits.has_position_limits)
        {
          *joint_type = urdf::Joint::REVOLUTE;
        }
        else
        {
          if (limits.angle_wraparound)
          *joint_type = urdf::Joint::CONTINUOUS;
          else
          *joint_type = urdf::Joint::PRISMATIC;
        }
      }

      if (limits.has_position_limits)
      {
        *lower_limit = limits.min_position;
        *upper_limit = limits.max_position;
      }
      if (limits.has_effort_limits)
      *effort_limit = limits.max_effort;

      if (has_soft_limits)
      {
        for (unsigned int cm = 0; cm< ctrl_method.size(); cm++)
        {
          switch (ctrl_method[cm])
          {
            case EFFORT:
            {
              const joint_limits_interface::EffortJointSoftLimitsHandle
              limits_handle(joint_handle[cm], limits, soft_limits);
              ej_limits_interface_.registerHandle(limits_handle);
            }
            break;
            case POSITION:
            {
              const joint_limits_interface::PositionJointSoftLimitsHandle
              limits_handle(joint_handle[cm], limits, soft_limits);
              pj_limits_interface_.registerHandle(limits_handle);
            }
            break;
            case VELOCITY:
            {
              const joint_limits_interface::VelocityJointSoftLimitsHandle
              limits_handle(joint_handle[cm], limits, soft_limits);
              vj_limits_interface_.registerHandle(limits_handle);
            }
            break;
          }
        }
      }
      else
      {
        for (unsigned int cm = 0; cm< ctrl_method.size(); cm++)
        {
          switch (ctrl_method[cm])
          {

            case EFFORT:
            {
              const joint_limits_interface::EffortJointSaturationHandle
              sat_handle(joint_handle[cm], limits);
              ej_sat_interface_.registerHandle(sat_handle);
            }
            break;
            case POSITION:
            {
              const joint_limits_interface::PositionJointSaturationHandle
              sat_handle(joint_handle[cm], limits);
              pj_sat_interface_.registerHandle(sat_handle);
            }
            break;
            case VELOCITY:
            {
              const joint_limits_interface::VelocityJointSaturationHandle
              sat_handle(joint_handle[cm], limits);
              vj_sat_interface_.registerHandle(sat_handle);
            }
            break;

          }//endswicth
        }

      }//endifelse

    }//end registerlimits function


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //readSim: Read state data from simulated robot hardware such as joint position, velocities,effort </summury>
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void MiaHWSim::readSim(ros::Time time, ros::Duration period)
    {
      for(unsigned int j=0; j < n_dof_; j++)
      {
        double position;
        double velocity;
        double effort;

        //*
        // 1) Read position
        if (joint_types_[j] == urdf::Joint::PRISMATIC)
        {
          #if GAZEBO_MAJOR_VERSION >= 9
          position = sim_joints_[j]->Position (0); // used with Gazebo 9
          #else

          position = sim_joints_[j]->GetAngle(0).Radian(); //Deprecated used for Gazebo 7.1.0,
          #endif

        }
        else
        {
          #if GAZEBO_MAJOR_VERSION >= 9
            position  = joint_position_[j] + angles::shortest_angular_distance(joint_position_[j],
            sim_joints_[j]->Position (0));

          #else
            position = joint_position_[j] + angles::shortest_angular_distance(joint_position_[j],
            sim_joints_[j]->GetAngle(0).Radian());
          #endif
        }

        //*
        // 2) Read velocity
        if (j == joints_ii.j_thumb_opp || j == joints_ii.j_thumb_flex || j == joints_ii.j_index_flex ||
            j == joints_ii.j_mrl_2 || j == joints_ii.j_mrl_3 || j== joints_ii.j_mrl_flex)
        {
          velocity = (position - joint_position_[j]) / period.toSec(); // rad/sec estimated for mia_hand
        }
        else
        {
          velocity = sim_joints_[j]->GetVelocity(0); // return wrong values
        }
        //*
        // 3) Read effort
        effort = sim_joints_[j]->GetForce((unsigned int)(0));

        //*
        // 4) Publish joint states
        if (j == joints_ii.j_index_flex)
        {
          // Read velocity with the right sign for the index
          velocity = ((position * j_index_flex_sign) - joint_position_[j]) / period.toSec(); // rad/sec estimated

          joint_position_[j] = (position * j_index_flex_sign);
          joint_velocity_[j] = velocity;
          joint_effort_[j] = effort * j_index_flex_sign;
        }
        else
        {

          joint_position_[j] = position;
          joint_velocity_[j] = velocity;
          joint_effort_[j] = effort;
        }

      } // end for joint

    }// end readSim function

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //writeSim: Write commands to the simulated robot hardware. such as joint position, velocities,
    // effort commands
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void MiaHWSim::writeSim(ros::Time time, ros::Duration period)
    {

      /* Handle the initialization of the last commands when this function
      has been called for the first time */
      if(write_counter == 0)
      {
        // Initialize variables used to select the control methods for the joints
        for(unsigned int j=0; j < n_dof_; j++)
        {
          // for the first iteration initialize the last command and the last joint position status
          last_joint_position_command_[j] = joint_position_command_ [j];
          last_joint_velocity_command_[j] = joint_velocity_command_[j];
          last_joint_effort_command_[j]   = joint_effort_command_[j];

          last_joint_control_methods_[j] =   POSITION;
        }
        write_counter++;
      }

      // If the E-stop is active, joints controlled by position commands will maintain their positions.
      if (e_stop_active_)
      {
        if (!last_e_stop_active_)
        {
          last_joint_position_command_ = joint_position_;
          last_e_stop_active_ = true;
        }
        joint_position_command_ = last_joint_position_command_;
      }
      else
      {
        last_e_stop_active_ = false;
      }

      // Enforce limits
      ej_sat_interface_.enforceLimits(period);
      ej_limits_interface_.enforceLimits(period);
      pj_sat_interface_.enforceLimits(period);
      pj_limits_interface_.enforceLimits(period);
      vj_sat_interface_.enforceLimits(period);
      vj_limits_interface_.enforceLimits(period);

      // Set the max difference position that is allowed between middle, ring and middle finger
      const double MaxDiff_mrl = 0.1; // rad

      for(unsigned int j=0; j < n_dof_; j++)
      {

        /* Skip write action for j_mrl_2 and j_mrl_3 joints since they
        have to follow j_mrl_flex joint */
        if(j == joints_ii.j_mrl_2 || j == joints_ii.j_mrl_3 || j == joints_ii.j_thumb_opp)
        {continue;}

        joint_control_methods_[j] = SelectCtrMethod(  List_joint_control_methods_[j][0],
        last_joint_control_methods_[j],
        last_joint_velocity_command_[j],
        joint_velocity_command_[j],
        last_joint_position_command_[j],
        joint_position_command_[j]);

        // Save the last recevived command for the next function call
        last_joint_position_command_[j] = joint_position_command_[j];
        last_joint_velocity_command_[j] = joint_velocity_command_[j];
        last_joint_effort_command_[j] 	= joint_effort_command_[j];
        last_joint_control_methods_[j] = joint_control_methods_[j];

        //ROS_INFO_STREAM("Joint " <<j << " "<< joint_control_methods_[j]);

        // Control the joint based on the selected control method
        switch (joint_control_methods_[j])
        {
          case EFFORT:
          {
            const double effort = e_stop_active_ ? 0 : joint_effort_command_[j];

            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3
            {
              // Check if ring little fingers get stack and evaluate the the middle position accordingly
              bool isStack = GetMiddlePosition(joint_position_[j], joint_position_[joints_ii.j_mrl_2], joint_position_[joints_ii.j_mrl_3], effort);
              if (isStack)
              {
                double target_pos = jMiddle_StackTarget_position; // stack --> Get fixed position
                sim_joints_[j]->SetPosition(0, target_pos); // command the middle joint in to stop in position
              }
              else
              {
                sim_joints_[j]->SetForce(0, effort); // unstack --> normal behaviour
              }


              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF then...
              {
                // ring and little has to mirror the target position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetPosition(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetPosition(0, joint_position_[j]);
              }
            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {
              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetPosition(0, jThOpp_Target_position);

              if(*j_index_flex_pos ==0  )
              {
                j_index_flex_sign = 1;
                sim_joints_[j]->SetForce(0, effort);
              }

              // this condition prevents that the index get stuck when it is started at pos = 0
              else if (*j_index_flex_pos < 0 &&  *j_index_flex_pos >= -0.0005 &&  j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, -(-0.0005));
              }

              else if (*j_index_flex_pos < 0 &&  *j_index_flex_pos < -0.0005 && j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, - *j_index_flex_pos);
              }

              else if (*j_index_flex_pos < 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = -1;
                sim_joints_[j]->SetForce(0, -joint_effort_command_[j]);

              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = 1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0,  *j_index_flex_pos);
              }
              else if (*j_index_flex_pos > 0 && j_index_flex_sign == 1 )
              {

                j_index_flex_sign = 1;
                sim_joints_[j]->SetForce(0, effort);
              }

            }

            /* For all the other joints of the simulated hardware (i.e the Mia
              thumb flexion joint or joints of a robot arm ) behave in the
              standard way */
            else
            {
              sim_joints_[j]->SetForce(0, effort);
            }

          }
          break;

          case POSITION:
          {

            #if GAZEBO_MAJOR_VERSION >= 4

            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3
            {
              // Check if ring little fingers get stack and evaluate the the middle position accordingly
              double flex_direction = joint_position_command_[j] - joint_position_[j];
              bool isStack = GetMiddlePosition(joint_position_[j], joint_position_[joints_ii.j_mrl_2], joint_position_[joints_ii.j_mrl_3], flex_direction);
              if (isStack)
              {
                double target_pos = jMiddle_StackTarget_position; // stack --> fixed position
                // command the middle joint in to stop in position
                sim_joints_[j]->SetPosition(0, target_pos);
              }
              else
              {
                sim_joints_[j]->SetPosition(0, joint_position_command_[j]); // unstack --> normal behaviour
              }

              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF
              {

                // ring and little has to mirror the position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetPosition(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetPosition(0, joint_position_[j]);

              }

            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {
              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetPosition(0, jThOpp_Target_position);

              //ROS_INFO_STREAM_NAMED("mia_hw_sim", "PI: " << joint_position_command_[j] );
              if( joint_position_command_[j] < j_index_flex_pos_Th)
              {

                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, -joint_position_command_[j]);
                j_index_flex_sign = -1;

              }
              else
              {
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, joint_position_command_[j]);
                j_index_flex_sign = 1;
              }

            }
            else if (j == joints_ii.j_thumb_flex )
            {
              if (joint_position_command_[j] < joint_lower_limits_[j] +0.01)
              {
                sim_joints_[j]->SetPosition(0, joint_position_command_[j]+0.01);
              }
              else
              {
                sim_joints_[j]->SetPosition(0, joint_position_command_[j]);
              }

            }

            /* For all the other joints of the simulated hardware (i.e joints of
               a robot arm ) behave in the standard way */
            else
            {sim_joints_[j]->SetPosition(0, joint_position_command_[j]);}

            #else
            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3

            {

              sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF
              {
                // ring and little has to mirror the position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetAngle(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetAngle(0, joint_position_[j]);
              }

            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {
              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetAngle(0, jThOpp_Target_position);


              if( joint_position_command_[j] < j_index_flex_pos_Th)
              {

                sim_joints_[joints_ii.j_index_flex]->SetAngle(0, -joint_position_command_[j]);
                j_index_flex_sign = -1;

              }
              else
              {
                sim_joints_[joints_ii.j_index_flex]->SetAngle(0, joint_position_command_[j]);
                j_index_flex_sign = 1;
              }

            }
            else if (j == joints_ii.j_thumb_flex )
            {
              if (joint_position_command_[j] < joint_lower_limits_[j] +0.01)
              {
                sim_joints_[j]->SetAngle(0, joint_position_command_[j]+0.01);
              }
              else
              {
                sim_joints_[j]->SetAngle(0, joint_position_command_[j]);
              }

            }

            /* For all the other joints of the simulated hardware (i.e joints of
               a robot arm ) behave in the standard way */
            else
            {sim_joints_[j]->SetAngle(0, joint_position_command_[j]);}

            #endif
          }
          break;

          case POSITION_PID:
          {

            if (j == joints_ii.j_index_flex || j == joints_ii.j_thumb_flex || j == joints_ii.j_mrl_flex)
            {continue;} // control type not implemented for mia hand joints

            double error;
            switch (joint_types_[j])
            {
              case urdf::Joint::REVOLUTE:
              angles::shortest_angular_distance_with_limits(joint_position_[j],
                joint_position_command_[j],
                joint_lower_limits_[j],
                joint_upper_limits_[j],
                error);
                break;
                case urdf::Joint::CONTINUOUS:
                error = angles::shortest_angular_distance(joint_position_[j],
                  joint_position_command_[j]);
                  break;
                  default:
                  error = joint_position_command_[j] - joint_position_[j];
                }

                const double effort_limit = joint_effort_limits_[j];
                const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
                -effort_limit, effort_limit);
                sim_joints_[j]->SetForce(0, effort);

              }
          break;

          case VELOCITY:
          {

            const double velocity_control = e_stop_active_ ? 0 : joint_velocity_command_[j];

            // since SetParam("vel", 0, velocity_control); seems not to work properly we bypassed it simulating and instantaneus velocity
            //using position control and the simulation time step
            double delta = velocity_control * period.toSec();
            double target_pos;

            #if GAZEBO_MAJOR_VERSION >= 4

            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3
            {

              // Check if ring little fingers get stack and evaluate the the middle position accordingly
              bool isStack = GetMiddlePosition(joint_position_[j], joint_position_[joints_ii.j_mrl_2], joint_position_[joints_ii.j_mrl_3], velocity_control);
              if (isStack)
              {
                target_pos = jMiddle_StackTarget_position; // stack --> fixed position
              }
              else
              {
                target_pos = joint_position_[j] + delta; // unstack --> normal behaviour
              }

              // command the middle joint
              sim_joints_[j]->SetPosition(0, target_pos);

              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF then...
              {
                // ring and little has to mirror the position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetPosition(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetPosition(0, joint_position_[j]);

              }
            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {
              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetPosition(0, jThOpp_Target_position);

              // index control
              if(*j_index_flex_pos ==0  )
              {
                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetPosition(0, target_pos);
              }

              // This condition avoids that the index get stuck when it is started at pos = 0
              else if (*j_index_flex_pos < 0 &&  *j_index_flex_pos >= -0.0005 &&  j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, - (-0.0005));
              }

              else if (*j_index_flex_pos < 0 &&  *j_index_flex_pos < -0.0005  && j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0, - *j_index_flex_pos);
              }

              else if (*j_index_flex_pos < 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = -1;
                target_pos = -joint_position_[j] - delta;
                sim_joints_[j]->SetPosition(0, target_pos);
              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = 1;
                sim_joints_[joints_ii.j_index_flex]->SetPosition(0,  *j_index_flex_pos);
              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == 1 )
              {
                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetPosition(0, target_pos);
              }

            }
            else if (j == joints_ii.j_thumb_flex )
            {
              target_pos = joint_position_[j] + delta;
              sim_joints_[j]->SetPosition(0, target_pos);
            }

            /* For all the other joints of the simulated hardware (i.e joints of
               a robot arm ) behave in the standard way */
            else
            {
              sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
            }

            #elif GAZEBO_MAJOR_VERSION <= 2

            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3
            {
              target_pos = joint_position_[j] + delta;
              sim_joints_[j]->SetAngle(0, target_pos);

              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF
              {
                // ring and little has to mirror the position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetAngle(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetAngle(0, joint_position_[j]);
              }
            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {


              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetAngle(0, jThOpp_Target_position);


              // index control
              if(*j_index_flex_pos ==0  )
              {
                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetAngle(0, target_pos);


              }
              else if (*j_index_flex_pos < 0 && j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetAngle(0, - *j_index_flex_pos);

              }


              else if (*j_index_flex_pos < 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = -1;
                target_pos = -joint_position_[j] - delta;
                sim_joints_[j]->SetAngle(0, target_pos);


              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = 1;
                sim_joints_[joints_ii.j_index_flex]->SetAngle(0,  *j_index_flex_pos);
              }
              else if (*j_index_flex_pos > 0 && j_index_flex_sign == 1 )
              {

                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetAngle(0, target_pos);


              }
            }
            else if (j == joints_ii.j_thumb_flex )
            {
              target_pos = joint_position_[j] + delta;
              sim_joints_[j]->SetAngle(0, target_pos);
            }

            /* For all the other joints of the simulated hardware (i.e joints of
               a robot arm ) behave in the standard way */
            else
            {
              sim_joints_[j]->SetVelocity(0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
            }
            #else

            if (j == joints_ii.j_mrl_flex) // then send the command also to jmrl_2 and j_mrl_3
            {
              target_pos = joint_position_[j] + delta;
              sim_joints_[j]->SetAngle(0, target_pos);

              if(!mrl_united) // if the middle ring little joints are not physically united in the URDF
              {
                // ring and little has to mirror the position of the middle
                sim_joints_[joints_ii.j_mrl_2]->SetAngle(0, joint_position_[j]);
                sim_joints_[joints_ii.j_mrl_3]->SetAngle(0, joint_position_[j]);
              }
            }
            else if (j == joints_ii.j_index_flex) // then menage also the thumb position control
            {
              // Get the position to send to the j_thumb_opp
              const double j_thumb_opp_effort_control = GetThumbOppPosition(period);
              sim_joints_[joints_ii.j_thumb_opp]->SetAngle(0, jThOpp_Target_position);

              // index control
              if(*j_index_flex_pos ==0  )
              {
                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetAngle(0, target_pos);
              }

              else if (*j_index_flex_pos < 0 && j_index_flex_sign == 1)
              {
                j_index_flex_sign = -1;
                sim_joints_[joints_ii.j_index_flex]->SetAngle(0, - *j_index_flex_pos);
              }

              else if (*j_index_flex_pos < 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = -1;
                target_pos = -joint_position_[j] - delta;
                sim_joints_[j]->SetAngle(0, target_pos);
              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == -1 )
              {
                j_index_flex_sign = 1;
                sim_joints_[joints_ii.j_index_flex]->SetAngle(0,  *j_index_flex_pos);
              }

              else if (*j_index_flex_pos > 0 && j_index_flex_sign == 1 )
              {
                j_index_flex_sign = 1;
                target_pos = joint_position_[j] + delta;
                sim_joints_[j]->SetAngle(0, target_pos);
              }
            }
            else if (j == joints_ii.j_thumb_flex )
            {
              target_pos = joint_position_[j] + delta;
              sim_joints_[j]->SetAngle(0, target_pos);
            }

            /* For all the other joints of the simulated hardware (i.e joints of
               a robot arm ) behave in the standard way */
            else
            {
              sim_joints_[j]->SetParam("vel", 0, e_stop_active_ ? 0 : joint_velocity_command_[j]);
            }

            #endif
          }
          break;

          case VELOCITY_PID:
          {
            if (j == joints_ii.j_index_flex || j == joints_ii.j_thumb_flex || j == joints_ii.j_mrl_flex)
            {continue;} // control type not implemented for mia hand joints


            double error;
            if (e_stop_active_)
            error = -joint_velocity_[j];
            else
            error = joint_velocity_command_[j] - joint_velocity_[j];

            const double effort_limit = joint_effort_limits_[j];
            const double effort = clamp(pid_controllers_[j].computeCommand(error, period),
            -effort_limit, effort_limit);
            sim_joints_[j]->SetForce(0, effort);
          }
          break;

        } // switch
      } // for joint
    } // end function


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //SelectCtrMethod: Select the method to use to control a joint. </summury>
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    enum MiaHWSim::ControlMethod MiaHWSim::SelectCtrMethod(enum MiaHWSim::ControlMethod Default_joint_control_methods_,
    							    enum MiaHWSim::ControlMethod last_joint_control_methods_,
    							    const double last_joint_velocity_command_,
    							    const double joint_velocity_command_,
    							    const double last_joint_position_command_,
    							    const double joint_position_command_)
    {

      if(Default_joint_control_methods_ == EFFORT)
        return EFFORT;
      else
      {
        if( joint_position_command_ != last_joint_position_command_) // is the only command that if it changes it is sure that some one modified the requested position
          return POSITION;
        else if( joint_velocity_command_ != last_joint_velocity_command_)
          return VELOCITY;
        else // default
          return last_joint_control_methods_;
      }
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //GetThumbOppPosition: Get the target position of the Mia thumb_opp joint based on the position of
    //the simulated Mia index actual position.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    double MiaHWSim::GetThumbOppPosition(ros::Duration period)
    {
       jThOpp_Target_position=MyTh_opp_passiveJoint.GetThumbOppPosition(*j_index_flex_pos);

       // Use a pid to calculate the effort for the thumb
        double error;

        angles::shortest_angular_distance_with_limits(joint_position_[joints_ii.j_thumb_opp], jThOpp_Target_position,
                                                             joint_lower_limits_[joints_ii.j_thumb_opp],
                                                             joint_upper_limits_[joints_ii.j_thumb_opp],
                                                             error);

           const double effort_limit = joint_effort_limits_[joints_ii.j_thumb_opp];
           const double effort = clamp(pid_controllers_[joints_ii.j_thumb_opp].computeCommand(error, period),
                                       -effort_limit, effort_limit);
           return effort;
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //GetMiddlePosition: Get the position of the Mia mrl1 joint based in order to avoid that a max
    // distance between mrl1 mrl2 and mrl3 can occur. This is usefull to simulate situation in which
    // the little or ring finger of the simulated Mia hand get stack.
    // flex direction can be teh velocity control received, the effort control etc </summury>
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool MiaHWSim::GetMiddlePosition(double joint_position_m, double joint_position_r, double joint_position_l, double flex_direction)
    {
        // Initialize variables
        const double MaxDist = 0.05; // rad max distance that can be accepted between the middle, ring and little fingers
        double isStack = false;


        // Simulate the stack of the ring and/or little finger
        double diff_mr = joint_position_m - joint_position_r;
        double diff_ml = joint_position_m - joint_position_l;

        if (diff_mr >= MaxDist & flex_direction > 0 ) // If m is more flexed than ring --> m can only extend
        {
            jMiddle_StackTarget_position = joint_position_r + (MaxDist+0.01);
            isStack = true;
        }

        else if (diff_ml >= MaxDist & flex_direction > 0 ) // If m is more flexed than little --> m can only extend
        {
            jMiddle_StackTarget_position = joint_position_l + (MaxDist+0.01);
            isStack = true;
        }

        else if (diff_mr <= -MaxDist  & flex_direction < 0 ) // If m is more extended than ring --> m can only flex
        {
            jMiddle_StackTarget_position = joint_position_r - (MaxDist+0.01);
            isStack = true;
        }

        else if (diff_ml <= -MaxDist  & flex_direction < 0 ) // If m is more extended than ring --> m can only flex
        {
            jMiddle_StackTarget_position = joint_position_l - (MaxDist+0.01);
            isStack = true;
        }

        else
        {
            isStack = false;
        }

        return isStack;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //eStopActive: Set the simulated robot's emergency stop state. The default implementation of
    // this function does nothing.
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void MiaHWSim::eStopActive(const bool active)
    {
       e_stop_active_ = active;
    }

}//end namespace mia

//  the pluginlib macros that allow us to register classes as plugins.
 PLUGINLIB_EXPORT_CLASS(mia::MiaHWSim, gazebo_ros_control::RobotHWSim)
// PLUGINLIB_EXPORT_CLASS(namespace::PluginClassName, namespace::BaseClassName)1
