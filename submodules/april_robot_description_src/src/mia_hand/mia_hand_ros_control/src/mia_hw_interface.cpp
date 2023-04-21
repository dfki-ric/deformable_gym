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

#include <iostream>
#include "mia_hand_ros_control/mia_hw_interface.h"

using hardware_interface::JointStateHandle;
using hardware_interface::JointHandle;
using transmission_interface::JointToActuatorPositionHandle;
using transmission_interface::JointToActuatorVelocityHandle;
using transmission_interface::ActuatorToJointStateHandle;

namespace mia_hand
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Class constructor: Initialize com and Urdf name getting ros parameters
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MiaHWInterface::MiaHWInterface()
  {
  	robot_description_ = "robot_description"; // default
  	n_dof_sim_ = 6; 			   // as declared in the URDF
  	write_counter = 0;
  	read_counter = 0; 			   // to delete
  	n_actuators_ = 3;

  	//Get Server IP address
  	if (ros::param::has("~Mia_COM_"))
  	{
  		ros::param::get("~Mia_COM_", COM_number_);
  	}
  	else
  	{
  		COM_number_ = 1; // default value
  		ros::param::set("~Mia_COM_", 1);

  	}

    // initialize connection vars
    is_connected_ = false;
    was_connected_ = false;
    is_initialized = false;

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Class distructor: Stop Mia data stream and disconnect mia hardware
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  MiaHWInterface:: ~MiaHWInterface()
  {
    // disconnect mia
  	disconnect_mia();

    is_initialized = false;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Disconnect Mia
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool MiaHWInterface:: disconnect_mia()
   {
     connectionmia_mtx_.lock();

     // Disable Mia streaming
     sleep(0.3);
     mia_.switchPosStream(false);
     sleep(0.3);
     mia_.switchSpeStream(false);
     sleep(0.3);
     mia_.switchAnaStream(false);
     sleep(0.3);
     mia_.switchCurStream(false);

     bool is_port_closed = mia_.disconnect();

      connectionmia_mtx_.unlock();
     return is_port_closed;
   }
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // Connect Mia
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool MiaHWInterface:: connect_mia(uint16_t port_num)
   {
     connectionmia_mtx_.lock();

     // connect Mia hand
     bool port_opened = mia_.connectToPort(port_num);

     if (port_opened)
     {
       std::string info_msg = "/dev/ttyUSB successfully opened.";
       info_msg.insert(11, std::to_string(port_num));
       ROS_INFO_NAMED("MiaHWInterface", "%s\n", info_msg.c_str());

       // Enable Mia streaming
       sleep(0.5);
       mia_.switchPosStream(true);
       sleep(1);
       mia_.switchSpeStream(true);
       sleep(0.5);

       // save com
       COM_number_ = port_num; // default value
       ros::param::set("~Mia_COM_", COM_number_);


     }
     else
     {
       std::string info_msg = "/dev/ttyUSB can not be open.";
       info_msg.insert(11, std::to_string(port_num));
       ROS_ERROR_NAMED("MiaHWInterface", "%s\n", info_msg.c_str());
     }

     connectionmia_mtx_.unlock();

     return port_opened;
   }

   //////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //checkConnection status of the mia hand
   /////////////////////////////////////////////////////////////////////////////////////////////////////////////
   bool MiaHWInterface::checkConnection()
   {
     connectionstatus_mtx_.lock();
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
     connectionstatus_mtx_.unlock();

     return is_connected_;
   }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //init: Initialize class and hardware
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::init(ros::NodeHandle &root_nh, ros::NodeHandle &mia_hw_nh)
  {
    // connect Mia hand
    bool isconnected_ = connect_mia(COM_number_);
    if(!isconnected_)
    	return false;

    if(is_initialized) // class has already been initialized
      return true;

  	nh_ = mia_hw_nh;
  	ROS_INFO_NAMED("MiaHWInterface","Initialization of MiaHWInterface" );

  	// Initialize passive joint thumb_opp + Index_fle
    const bool load = false;
    MyTh_opp_passiveJoint.init(load); // initialize without loading now URDF

  	if(!ros::isInitialized())
		{
 			ROS_FATAL_STREAM_NAMED("MiaHWInterface","Ros has not been initialized");
  		return false;
		 }

  	// Get Transmission number declared in the URDF to know which joint are actuated
  	const std::string urdf_string = getURDF(robot_description_);



  	if (!parseTransmissionsFromURDF(urdf_string))
  	{
  		ROS_ERROR_NAMED("MiaHWInterface", "Error parsing URDF in MiaHWInterface, MiaHWInterface not active.\n");
  		return false;
  	}

  	// get URDF model
  	urdf::Model urdf_model;
  	const urdf::Model *const urdf_model_ptr = urdf_model.initString(urdf_string) ? &urdf_model : NULL;


  	// getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  	// parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  	const ros::NodeHandle joint_limit_nh(nh_);



  	// Resize variables
  	joint_names_.resize(n_dof_sim_);
        	joint_types_.resize(n_dof_sim_);
  	joint_lower_limits_.resize(n_dof_sim_);
  	joint_upper_limits_.resize(n_dof_sim_);
  	joint_effort_limits_.resize(n_dof_sim_);

  	joint_position_state_.resize(n_dof_sim_);
  	joint_velocity_state_.resize(n_dof_sim_);
  	joint_effort_state_.resize(n_dof_sim_);

  	joint_effort_command_.resize(n_dof_sim_);
  	joint_position_command_.resize(n_dof_sim_);
  	joint_velocity_command_.resize(n_dof_sim_);

  	act_position_state_.resize(n_dof_sim_);
  	act_velocity_state_.resize(n_dof_sim_);
  	act_effort_state_.resize(n_dof_sim_);

  	act_position_command_.resize(n_dof_sim_);
  	act_velocity_command_.resize(n_dof_sim_);
  	act_effort_command_.resize(n_dof_sim_);

  	MiaTrasmissions.resize(n_dof_sim_);
  	trasmission_names_.resize(n_dof_sim_);

  	joint_control_methods_.resize(n_dof_sim_);
  	last_joint_position_command_.resize(n_dof_sim_);
  	last_joint_velocity_command_.resize(n_dof_sim_);
  	last_joint_control_methods_.resize(n_dof_sim_);
  	List_joint_control_methods_.resize(n_dof_sim_);

  	// TO DELETE
  	old_act_position_state_.resize(n_dof_sim_);

  	// scan the joints in the URDF and find the Mia joints and register them

  	int URDF_n_dof_sim_ = URDFtransmissions_.size();



  	for(unsigned int j=0; j < URDF_n_dof_sim_; j++)
  	{
  	// Check that this transmission has one joint
  	if(URDFtransmissions_[j].joints_.size() == 0)
  	{
  		ROS_WARN_STREAM_NAMED("MiaHWInterface","URDFTransmission " << URDFtransmissions_[j].name_
  		  << " has no associated joints.");
  		continue;
  	}

  	std::vector<std::string> joint_interfaces = URDFtransmissions_[j].joints_[0].hardware_interfaces_;

		if (joint_interfaces.empty() &&
		!(URDFtransmissions_[j].actuators_.empty()) &&
		!(URDFtransmissions_[j].actuators_[0].hardware_interfaces_.empty())) // Deprecate HW interface specification in actuators in ROS J
		{
			joint_interfaces = URDFtransmissions_[j].actuators_[0].hardware_interfaces_;
		}

		if (joint_interfaces.empty())
		{
			 ROS_WARN_STREAM_NAMED("MiaHWInterface", "Joint " << URDFtransmissions_[j].joints_[0].name_ <<
			" of URDFtransmission " << URDFtransmissions_[j].name_ << " does not specify any hardware interface. " <<
			"Not adding it to the robot hardware simulation.");
			continue;
		}

		const std::string temp_joint_name = URDFtransmissions_[j].joints_[0].name_;


		// Add data from URDFtransmission and save the position of eac joints
		int k ;

		if (temp_joint_name == "j_thumb_fle" )
		{
			k = 0;
			joints_ii.j_thumb_flex = k;
			MiaTrasmissions[k] = &ThfleTrans;
			trasmission_names_[k] = "MiaThumbTrans";
		}
		else if (temp_joint_name == "j_index_fle" )
		{
			k = 2;
			joints_ii.j_index_flex = k;
			MiaTrasmissions[k] = &IndexTrans;
			trasmission_names_[k] = "MiaIndexTrans";

		}
		else if (temp_joint_name == "j_mrl_fle" )
		{
			k = 1;
			joints_ii.j_mrl_flex = k;
			MiaTrasmissions[k] = &MrlTrans;
			trasmission_names_[k] = "MiaMrlTrans";
		}
		else if (temp_joint_name == "j_ring_fle" )
		{
			k = 3;
			joints_ii.j_mrl_2 = k;
		}
		else if (temp_joint_name == "j_little_fle" )
		{
			k= 4;
			joints_ii.j_mrl_3 = k;
		}
		else if (temp_joint_name == "j_thumb_opp" )
		{
			k = 5;
			joints_ii.j_thumb_opp = k;
		}
		else
		{
			continue; // ignore joints that are not of the MIA Hand
		}


		// Initialize vector where to store state commands and actuartor data
		joint_names_[k] = URDFtransmissions_[j].joints_[0].name_;
		joint_position_state_[k] = 0;
		joint_velocity_state_[k] = 0.0;
		joint_effort_state_[k] = 1.0;  // N/m for continuous joints

		joint_position_command_[k] = 0.0;
		joint_velocity_command_[k] = 0.0;
		joint_effort_command_[k] = 0.0;


		act_position_state_[k] = 0.0;
		act_velocity_state_[k] = 0.0;
		act_effort_state_[k] = 0.0;

		act_position_command_ [k] = 0.0;
		act_velocity_command_ [k] = 0.0;
		act_effort_command_ [k] = 0.0;

		List_joint_control_methods_[k].resize(2);


		// Register Joint State interface for the joint
		js_interface_.registerHandle(hardware_interface::JointStateHandle(
              	joint_names_[k], &joint_position_state_[k], &joint_velocity_state_[k], &joint_effort_state_[k]));

		// Register Joint Position and Joint Velocity interface for the joint
		std::vector< hardware_interface::JointHandle> joint_handle;
		joint_handle.resize(2);

		joint_control_methods_[k] = POSITION;
		List_joint_control_methods_[k][0] = POSITION;
		joint_handle[0] = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[k]),
                                                        		    &joint_position_command_[k]);


		pj_interface_.registerHandle(joint_handle[0]);

		List_joint_control_methods_[k][1] = VELOCITY;
		joint_handle[1] = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[k]),
                                                        		   &joint_velocity_command_[k]);
    vj_interface_.registerHandle(joint_handle[1]);


  	// Get joint Limits
		registerJointLimits(joint_names_[k], joint_handle, List_joint_control_methods_[k],
                 		    joint_limit_nh, urdf_model_ptr,
                 		    &joint_types_[k], &joint_lower_limits_[k], &joint_upper_limits_[k],
                 		    &joint_effort_limits_[k]);




		// Initialize trasmission varibales for 3 joints: thumb (k=0) index (k=1)  and mrl flex (k=2)
		if( k >= 0 && k <= 2)
		{
			// Wrap state raw data
			a_state_data[k].position.push_back(&act_position_state_[k]);
			a_state_data[k].velocity.push_back(&act_velocity_state_[k]);
			a_state_data[k].effort.push_back(&act_effort_state_[k]);

			j_state_data[k].position.push_back(&joint_position_state_[k]);
			j_state_data[k].velocity.push_back(&joint_velocity_state_[k]);
			j_state_data[k].effort.push_back(&joint_effort_state_[k]);

			// Wrap command data
			a_cmd_data[k].position.push_back(&act_position_command_[k]);
			a_cmd_data[k].velocity.push_back(&act_velocity_command_[k]);

			j_cmd_data[k].position.push_back(&joint_position_command_[k]);
			j_cmd_data[k].velocity.push_back(&joint_velocity_command_[k]);

			// Register transmissions to each interface
			if( k == joints_ii.j_index_flex ) // IndexTrans
			{
				index_act_to_jnt_pos_state.registerHandle(transmission_interface::MiaActuatorToJointPositionHandle(trasmission_names_[k], &IndexTrans, a_state_data[k], j_state_data[k]));
				index_act_to_jnt_vel_state.registerHandle(transmission_interface::MiaActuatorToJointVelocityHandle(trasmission_names_[k], &IndexTrans, a_state_data[k], j_state_data[k]));

				index_jnt_to_act_pos.registerHandle(transmission_interface::MiaJointToActuatorPositionHandle(trasmission_names_[k], &IndexTrans, a_cmd_data[k], j_cmd_data[k], a_state_data[k]));
				index_jnt_to_act_vel.registerHandle(transmission_interface::MiaJointToActuatorVelocityHandle(trasmission_names_[k], &IndexTrans, a_cmd_data[k], j_cmd_data[k], a_state_data[k]));
			}
			else
			{
				act_to_jnt_pos_state.registerHandle(transmission_interface::ActuatorToJointPositionHandle(trasmission_names_[k], MiaTrasmissions[k], a_state_data[k], j_state_data[k]));
				act_to_jnt_vel_state.registerHandle(transmission_interface::ActuatorToJointVelocityHandle(trasmission_names_[k], MiaTrasmissions[k], a_state_data[k], j_state_data[k]));

				jnt_to_act_pos.registerHandle(transmission_interface::JointToActuatorPositionHandle(trasmission_names_[k], MiaTrasmissions[k], a_cmd_data[k], j_cmd_data[k]));
				jnt_to_act_vel.registerHandle(transmission_interface::JointToActuatorVelocityHandle(trasmission_names_[k], MiaTrasmissions[k], a_cmd_data[k], j_cmd_data[k]));
			}
			// ROS_WARN_STREAM_NAMED("MiaHWInterface", "k " << k <<", Name" << joint_names_[k] <<", trasm" << trasmission_names_[k] );


		}

	}

	// Update the passive joints limit
  MyTh_opp_passiveJoint.updateThOppJointLimits (joint_lower_limits_[joints_ii.j_thumb_opp],
                                                joint_upper_limits_[joints_ii.j_thumb_opp] );
	// MyTh_opp_passiveJoint.ThMinPos = joint_lower_limits_[joints_ii.j_thumb_opp] ;
  // MyTh_opp_passiveJoint.ThMaxPos = joint_upper_limits_[joints_ii.j_thumb_opp] ;

	// Register interfaces
	registerInterface(&js_interface_);
	registerInterface(&ej_interface_);
	registerInterface(&pj_interface_);
	registerInterface(&vj_interface_);

  // Initialize Services and publishers
  initPublishers();
  initServices();

  is_initialized = true;

	return true;
  }




  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //initPublishers
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void MiaHWInterface::initPublishers()
  {
  	mot_cur_info_ = nh_.advertise<mia_hand_msgs::FingersData>("mot_cur", 1000);
  	fin_for_info_ = nh_.advertise<mia_hand_msgs::FingersStrainGauges>("fin_sg",
                                                                      1000);
    connection_status_info_ = nh_.advertise<mia_hand_msgs::ComponentStatus>("status"
                                                                            ,1000);

    is_mot_cur_info_enabled = false;
    is_fin_for_info_enabled = false;
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //initServices
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void MiaHWInterface::initServices()
  {
  	connect_to_port_ = nh_.advertiseService("connect_to_port",
                                            &MiaHWInterface::connectToPortCallback,
                                            this);

  	disconnect_ = nh_.advertiseService("disconnect",
                                       &MiaHWInterface::disconnectCallback, this);


  	switch_ana_stream_ = nh_.advertiseService("switch_ana_stream",
                                              &MiaHWInterface::switchAnaStreamCallback,
                                              this);

  	switch_cur_stream_ = nh_.advertiseService("switch_cur_stream",
                                              &MiaHWInterface::switchCurStreamCallback,
                                              this);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //switchAnaStreamCallback: callback of the service
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::switchAnaStreamCallback(std_srvs::SetBool::Request& req,
                                           std_srvs::SetBool::Response& resp)
  {
    mia_.switchAnaStream(req.data);

    is_fin_for_info_enabled = req.data;
    resp.success = true;

  	return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //switchCurStreamCallback: callback of the service
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::switchCurStreamCallback(std_srvs::SetBool::Request& req,
                                           std_srvs::SetBool::Response& resp)
  {
    mia_.switchCurStream(req.data);
    is_mot_cur_info_enabled = req.data;
    resp.success = true;

  	return true;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //connectToPortCallback: callback of the service
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::connectToPortCallback(
         mia_hand_msgs::ConnectSerial::Request&  req,
         mia_hand_msgs::ConnectSerial::Response& resp)
  {

    bool is_port_open = connect_mia(req.port);

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

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //disconnectCallback: callback of the service
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::disconnectCallback(std_srvs::Trigger::Request& req,
                                     std_srvs::Trigger::Response& resp)
  {
    bool is_port_closed = disconnect_mia();

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

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //read: Read actuator state from hardware and propagate to joint spce
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void MiaHWInterface::read(const ros::Time& time, const ros::Duration& duration)
  {
  	// Get from Mia hand
  	for (uint8_t jnt = 0; jnt < n_actuators_; jnt++)
  	{
  		act_position_state_[jnt] = (double)mia_.getMotorPos(jnt);
  		act_velocity_state_[jnt] = (double)mia_.getMotorSpe(jnt);

  		if (read_counter== 0 )
  		{
  			old_act_position_state_[jnt] = act_position_state_[jnt] ;
  			read_counter ++ ;
  		}
  		else
  		{
  			if (act_position_state_[jnt] < old_act_position_state_[jnt] )
  				 act_velocity_state_[jnt] = - act_velocity_state_[jnt];
  			old_act_position_state_[jnt] = act_position_state_[jnt] ;
  		}
  	}


  	// propagate state trasmission
  	act_to_jnt_pos_state.propagate();
  	act_to_jnt_vel_state.propagate();
  	index_act_to_jnt_pos_state.propagate();
  	index_act_to_jnt_vel_state.propagate();

  	// update the sate of the other passive joints (helpfull for visualization tools)
  	joint_position_state_[joints_ii.j_mrl_2 ] = joint_position_state_[joints_ii.j_mrl_flex ];
  	joint_velocity_state_[joints_ii.j_mrl_2 ] = joint_velocity_state_[joints_ii.j_mrl_flex ];

  	joint_position_state_[joints_ii.j_mrl_3 ] = joint_position_state_[joints_ii.j_mrl_flex ];
  	joint_velocity_state_[joints_ii.j_mrl_3 ] = joint_velocity_state_[joints_ii.j_mrl_flex ];

  	joint_position_state_[joints_ii.j_thumb_opp ] = GetThumbOppPosition();
  	joint_velocity_state_[joints_ii.j_thumb_opp ] = 0;

    // publish current stream if is_fin_for_info_enabled
    if(is_fin_for_info_enabled)
    {
      mia_hand_msgs::FingersStrainGauges msg_sg;

      mia_.getFingerSgRaw(0, msg_sg.thu[0], msg_sg.thu[1]);
      mia_.getFingerSgRaw(2, msg_sg.ind[0], msg_sg.ind[1]);
      mia_.getFingerSgRaw(1, msg_sg.mrl[0], msg_sg.mrl[1]);

      fin_for_info_.publish(msg_sg);
    }

    if(is_mot_cur_info_enabled)
    {
      mia_hand_msgs::FingersData msg;

      msg.thu = mia_.getMotorCur(0);
      msg.ind = mia_.getMotorCur(2);
      msg.mrl = mia_.getMotorCur(1);

      mot_cur_info_.publish(msg);
    }

    // publish the hand connection ComponentStatus
    bool status_ = checkConnection();
    mia_hand_msgs::ComponentStatus msg_status;
    msg_status.status = status_;
    if(status_)
    {
      msg_status.msg = "Mia hand connected";
    }
    else
    {
      msg_status.msg = "Mia hand disconnected";
    }
    connection_status_info_.publish(msg_status);

  return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //write: Get joint target from ros interfaces, propagate to the actuator space and
  // Write the actuator target to the hardware.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void MiaHWInterface::write(const ros::Time& time, const ros::Duration& duration)
  {
  	/* Selectively send position or speed command according to
  	* the external commands sent by the controllers.
  	*/
  	if(write_counter == 0)
   	{
      InitBkLastCommands();
	  }

  	// Enforce limits
  	pj_sat_interface_.enforceLimits(duration);
  	pj_limits_interface_.enforceLimits(duration);
 	  vj_sat_interface_.enforceLimits(duration);
  	vj_limits_interface_.enforceLimits(duration);

  	for(unsigned int j=0; j < n_actuators_; j++)
    {

      // Select the control method for the current joint
  		joint_control_methods_[j] = SelectCtrMethod(last_joint_control_methods_[j],
  								last_joint_velocity_command_[j],
  								joint_velocity_command_[j],
  								last_joint_position_command_[j],
  								joint_position_command_[j]);

      // update backup
  		last_joint_position_command_[j] = joint_position_command_ [j];
  		last_joint_velocity_command_[j] = joint_velocity_command_[j];
  		last_joint_control_methods_[j] = joint_control_methods_[j]; // save for the next iteration

  		switch (joint_control_methods_[j])
  		{
  			case POSITION:
  			{
  				// Propagate cmd to actuator space
  				if(j == joints_ii.j_index_flex)
  					index_jnt_to_act_pos.propagate();

  				else
  					jnt_to_act_pos.propagate();


  				// send command to Mia
  				mia_.setMotorPos( j, (int16_t)act_position_command_[j]);
  			}
  			break;

  			case VELOCITY:
  			{
  				// Propagate cmd to actuator space
  				if(j == joints_ii.j_index_flex)
  					index_jnt_to_act_vel.propagate();
  				else
  					jnt_to_act_vel.propagate();

  				// send command to Mia
  				mia_.setMotorSpe( j, (int16_t)act_velocity_command_[j]);
  			}
  			break;

  		} // end switch
  	}// end for joint

  	return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // InitBkLastCommands: Initialize the arrays saving the last received joint command.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void  MiaHWInterface::InitBkLastCommands()
  {
  	for(unsigned int j=0; j < n_actuators_; j++)
  	{
  		// for the first iteration initialize the last command and the last joint position status
  		last_joint_position_command_[j] = joint_position_command_ [j];
  		last_joint_velocity_command_[j] = joint_velocity_command_[j];
  		//last_joint_effort_command_[j]   = joint_effort_command_[j];

  		last_joint_control_methods_[j] =   List_joint_control_methods_[j][0]; // POSITION
  	}
  	write_counter++;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // RegisterJointLimits:Register the limits of the joint specified by joint_name and joint_handle.
  // The limits are retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
  // Return the joint's type, lower position limit, upper position limit, and effort limit.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void MiaHWInterface::registerJointLimits(const std::string& joint_name,
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
         }// end switch
       } // end for
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
  		} // end for

    }//endifelse

  }//end registerlimits function


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //GetThumbOppPosition: Get the target position of the Mia thumb_opp joint based on the position of
  //the Mia index actual position.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  double MiaHWInterface::GetThumbOppPosition()
  {

    double j_index_flex_pos = joint_position_state_[joints_ii.j_index_flex ];
    double jThOpp_Target_position = MyTh_opp_passiveJoint.GetThumbOppPosition(j_index_flex_pos);

    return jThOpp_Target_position;
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SelectCtrMethod: Select the method to control a joint.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  enum MiaHWInterface::ControlMethod MiaHWInterface::SelectCtrMethod(
  							    enum MiaHWInterface::ControlMethod last_joint_control_methods_,
  							    const double last_joint_velocity_command_,
  							    const double joint_velocity_command_,
  							    const double last_joint_position_command_,
  							    const double joint_position_command_)
  {

	if( joint_position_command_ != last_joint_position_command_)
		return POSITION;
	else if( joint_velocity_command_ != last_joint_velocity_command_)
		return VELOCITY;
	else // default
		return last_joint_control_methods_;

  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //getURDF: Get the URDF XML from the parameter server.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  std::string MiaHWInterface::getURDF(std::string param_name) const
  {
  	std::string urdf_string;

  	// search and wait for robot_description on param server
  	while (urdf_string.empty())
  	{
  	std::string search_param_name;

  	if (nh_.searchParam(param_name, search_param_name))
  	{
  	  ROS_INFO_ONCE_NAMED("MiaHWInterface", "MiaHWInterface is waiting for model"
  		" URDF in parameter [%s] on the ROS param server.", search_param_name.c_str());

  	  nh_.getParam(search_param_name, urdf_string);
  	}
  	else
  	{
  	  ROS_INFO_ONCE_NAMED("MiaHWInterface", "MiaHWInterface  is waiting for model"
  		" URDF in parameter [%s] on the ROS param server.", robot_description_.c_str());

  	  nh_.getParam(param_name, urdf_string);
  	}

  	usleep(100000);
  	}
  	ROS_DEBUG_STREAM_NAMED("MiaHWInterface", "Recieved urdf from param server, parsing...");

  	return urdf_string;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //parseTransmissionsFromURDF: Get the transmissions declared into the URDF file.
  /////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool MiaHWInterface::parseTransmissionsFromURDF(const std::string& urdf_string)
  {

  	transmission_interface::TransmissionParser::parse(urdf_string, URDFtransmissions_);
  	return true;
  }

}  // namespace
